#include <fr3_husky_controller/servers/fr3_husky/move_to_joint_action_server.hpp>

#include <chrono>
#include <map>
#include <stdexcept>
#include <string>

#include <action_msgs/msg/goal_status.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/parameter_client.hpp>

namespace fr3_husky_controller::servers::fr3_husky
{

namespace
{
FR3HuskyModelUpdater& getFR3HuskyModelUpdater(ModelUpdaterBase& model_updater,
                                               const std::string& server_name)
{
    auto* p = dynamic_cast<FR3HuskyModelUpdater*>(&model_updater);
    if (!p)
        throw std::runtime_error("[" + server_name + "] requires FR3HuskyModelUpdater");
    return *p;
}
}  // namespace

// ============================================================
// Constructor / Destructor
// ============================================================

MoveToJoint::MoveToJoint(const std::string& name, const NodePtr& node,
                         ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
  fr3_husky_model_updater_(getFR3HuskyModelUpdater(model_updater, name))
{
    rclcpp::NodeOptions mgi_opts;
    mgi_opts.automatically_declare_parameters_from_overrides(true);
    moveit_node_ = rclcpp::Node::make_shared(name_ + "_mgi", mgi_opts);

    jtc_client_ = rclcpp_action::create_client<FJT>(
        node_, "fr3_husky_joint_trajectory_controller");

    auto qos = rclcpp::QoS(1).reliable().transient_local();
    jtc_status_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
        "fr3_husky_joint_trajectory_controller/_action/status",
        qos,
        [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg)
        {
            bool busy = false;
            for (const auto& gs : msg->status_list)
            {
                if (gs.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                    gs.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED)
                {
                    busy = true;
                    break;
                }
            }
            jtc_busy_.store(busy, std::memory_order_relaxed);
        });

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveToJoint created", name_.c_str());
}

MoveToJoint::~MoveToJoint()
{
    cancel_flag_.store(true, std::memory_order_relaxed);
    if (planning_thread_.joinable())
        planning_thread_.join();
}

// ============================================================
// Helpers
// ============================================================

std::string MoveToJoint::inferGroup(const std::vector<std::string>& joint_names)
{
    if (!joint_names.empty())
    {
        const std::string& n = joint_names[0];
        const auto pos = n.rfind("_joint");
        if (pos != std::string::npos)
            return n.substr(0, pos) + "_arm";
    }
    return "fr3_arm";
}

void MoveToJoint::writeHoldCommands()
{
    const size_t dof = model_updater_.manipulator_dof_;
    const Eigen::VectorXd qdot_zero = Eigen::VectorXd::Zero(dof);
    const Eigen::Vector2d wheel_zero = Eigen::Vector2d::Zero();

    if (model_updater_.HasEffortCommandInterface())
    {
        if (!fr3_husky_model_updater_.robot_controller_)
        {
            model_updater_.haltCommands();
            return;
        }
        const Eigen::VectorXd torque =
            fr3_husky_model_updater_.robot_controller_->moveManipulatorJointTorqueStep(
                q_hold_, qdot_zero, false);
        fr3_husky_model_updater_.torque_desired_total_ = torque - fr3_husky_model_updater_.g_total_;
        fr3_husky_model_updater_.writeCommand(fr3_husky_model_updater_.torque_desired_total_,
                                              wheel_zero);
    }
    else if (model_updater_.HasVelocityCommandInterface())
    {
        fr3_husky_model_updater_.qdot_desired_total_ = qdot_zero;
        fr3_husky_model_updater_.writeCommand(fr3_husky_model_updater_.qdot_desired_total_,
                                              wheel_zero);
    }
    else if (model_updater_.HasPositionCommandInterface())
    {
        fr3_husky_model_updater_.q_desired_total_ = q_hold_;
        fr3_husky_model_updater_.writeCommand(fr3_husky_model_updater_.q_desired_total_,
                                              wheel_zero);
    }
}

// ============================================================
// Background planning thread
// ============================================================

void MoveToJoint::runPlanning()
{
    const std::string group = inferGroup(goal_joint_names_);
    RCLCPP_INFO(node_->get_logger(), "[%s] planning group: %s", name_.c_str(), group.c_str());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_ok = false;
    std::string err_msg;

    // Forward robot_description{_semantic} from move_group to moveit_node_
    if (!moveit_node_->has_parameter("robot_description") ||
        !moveit_node_->has_parameter("robot_description_semantic"))
    {
        try
        {
            auto pc = std::make_shared<rclcpp::SyncParametersClient>(
                moveit_node_, "move_group");
            if (pc->wait_for_service(std::chrono::seconds(5)))
            {
                const std::vector<std::string> pnames{
                    "robot_description", "robot_description_semantic"};
                for (const auto& pname : pnames)
                {
                    if (moveit_node_->has_parameter(pname)) continue;
                    auto vals = pc->get_parameters({pname});
                    if (!vals.empty() &&
                        vals[0].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
                    {
                        moveit_node_->declare_parameter(pname, vals[0].get_parameter_value());
                    }
                }
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[%s] move_group param service not available within 5 s",
                            name_.c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] param forwarding failed: %s", name_.c_str(), e.what());
        }
    }

    if (!moveit_node_->has_parameter("robot_description_semantic"))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s] robot_description_semantic not available. "
                     "Is move_group running? (use launch_move_group:=true)",
                     name_.c_str());
        std::lock_guard<std::mutex> lk(msg_mutex_);
        plan_error_msg_ = "robot_description_semantic not available — start move_group first";
        plan_state_.store(PlanState::FAILED, std::memory_order_release);
        return;
    }

    if (!cancel_flag_.load(std::memory_order_relaxed))
    {
        try
        {
            moveit::planning_interface::MoveGroupInterface mgi(
                moveit_node_,
                group,
                /*tf_buffer=*/{},
                rclcpp::Duration::from_seconds(10.0));

            mgi.setPlanningTime(10.0);
            mgi.setStartStateToCurrentState();
            mgi.setMaxVelocityScalingFactor(goal_vel_scale_);
            mgi.setMaxAccelerationScalingFactor(goal_acc_scale_);

            std::map<std::string, double> target;
            for (size_t i = 0; i < goal_joint_names_.size(); ++i)
                target[goal_joint_names_[i]] = goal_target_positions_[i];

            if (!mgi.setJointValueTarget(target))
            {
                err_msg = "setJointValueTarget failed";
            }
            else if (cancel_flag_.load(std::memory_order_relaxed))
            {
                err_msg = "Cancelled before planning";
            }
            else
            {
                const auto ec = mgi.plan(plan);
                if (ec)
                {
                    plan_ok = true;
                    RCLCPP_INFO(node_->get_logger(),
                                "[%s] planning succeeded (%zu waypoints)",
                                name_.c_str(),
                                plan.trajectory_.joint_trajectory.points.size());
                }
                else
                {
                    err_msg = "MoveIt2 plan failed (error_code=" +
                              std::to_string(ec.val) + ")";
                }
            }
        }
        catch (const std::exception& e)
        {
            err_msg = std::string("MoveGroupInterface exception: ") + e.what();
        }
        catch (...)
        {
            err_msg = "Unknown exception in MoveGroupInterface";
        }
    }
    else
    {
        err_msg = "Cancelled before planning";
    }

    if (!plan_ok)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), err_msg.c_str());
        {
            std::lock_guard<std::mutex> lk(msg_mutex_);
            plan_error_msg_ = err_msg;
        }
        plan_state_.store(PlanState::FAILED, std::memory_order_release);
        return;
    }

    if (cancel_flag_.load(std::memory_order_relaxed))
    {
        std::lock_guard<std::mutex> lk(msg_mutex_);
        plan_error_msg_ = "Cancelled after planning";
        plan_state_.store(PlanState::FAILED, std::memory_order_release);
        return;
    }

    // Forward trajectory to fr3_husky_joint_trajectory_controller
    if (!jtc_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        std::lock_guard<std::mutex> lk(msg_mutex_);
        plan_error_msg_ = "fr3_husky_joint_trajectory_controller not available";
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), plan_error_msg_.c_str());
        plan_state_.store(PlanState::FAILED, std::memory_order_release);
        return;
    }

    FJT::Goal jtc_goal;
    jtc_goal.trajectory = plan.trajectory_.joint_trajectory;

    auto send_opts = rclcpp_action::Client<FJT>::SendGoalOptions();
    send_opts.goal_response_callback =
        [this](std::shared_ptr<GoalHandleFJT> gh)
        {
            if (!gh)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[%s] fr3_husky_joint_trajectory_controller rejected goal",
                             name_.c_str());
                std::lock_guard<std::mutex> lk(msg_mutex_);
                plan_error_msg_ = "JTC rejected the trajectory goal";
                plan_state_.store(PlanState::FAILED, std::memory_order_release);
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[%s] trajectory sent to fr3_husky_joint_trajectory_controller",
                            name_.c_str());
                plan_state_.store(PlanState::DONE, std::memory_order_release);
            }
        };

    jtc_client_->async_send_goal(jtc_goal, send_opts);

    for (int i = 0; i < 50 && plan_state_.load() == PlanState::PLANNING; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

// ============================================================
// Goal lifecycle
// ============================================================

bool MoveToJoint::acceptGoal(const ActionT::Goal& goal)
{
    if (goal.joint_names.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject: joint_names is empty", name_.c_str());
        return false;
    }
    if (goal.joint_names.size() != goal.target_positions.size())
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Reject: joint_names.size()=%zu != target_positions.size()=%zu",
                    name_.c_str(),
                    goal.joint_names.size(), goal.target_positions.size());
        return false;
    }
    if (jtc_busy_.load(std::memory_order_relaxed))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Reject: fr3_husky_joint_trajectory_controller is already executing",
                    name_.c_str());
        return false;
    }
    return true;
}

void MoveToJoint::onGoalAccepted(const ActionT::Goal& goal)
{
    goal_joint_names_      = goal.joint_names;
    goal_target_positions_ = goal.target_positions;
    goal_vel_scale_ = (goal.max_velocity_scaling_factor > 0.0)
                      ? goal.max_velocity_scaling_factor : 0.1;
    goal_acc_scale_ = (goal.max_acceleration_scaling_factor > 0.0)
                      ? goal.max_acceleration_scaling_factor : 0.1;

    cancel_flag_.store(false, std::memory_order_relaxed);
    plan_state_.store(PlanState::PLANNING, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(msg_mutex_);
        plan_error_msg_.clear();
    }

    if (planning_thread_.joinable())
        planning_thread_.join();

    planning_thread_ = std::thread([this] {
        try
        {
            runPlanning();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[%s] planning thread threw: %s", name_.c_str(), e.what());
            std::lock_guard<std::mutex> lk(msg_mutex_);
            plan_error_msg_ = e.what();
            plan_state_.store(PlanState::FAILED, std::memory_order_release);
        }
        catch (...)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[%s] planning thread threw unknown exception", name_.c_str());
            std::lock_guard<std::mutex> lk(msg_mutex_);
            plan_error_msg_ = "Unknown exception in planning thread";
            plan_state_.store(PlanState::FAILED, std::memory_order_release);
        }
    });

    RCLCPP_INFO(node_->get_logger(), "[%s] goal accepted — planning started", name_.c_str());
}

void MoveToJoint::onStart()
{
    fr3_husky_model_updater_.setInitFromCurrent();
    q_hold_           = fr3_husky_model_updater_.q_total_;
    result_error_code_ = ActionT::Result::ERROR_PLAN_FAILED;
    RCLCPP_INFO(node_->get_logger(),
                "[%s] started, holding position during planning", name_.c_str());
}

// ============================================================
// Control loop
// ============================================================

MoveToJoint::ComputeResult MoveToJoint::compute(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    const auto state = plan_state_.load(std::memory_order_acquire);

    if (state == PlanState::PLANNING)
    {
        writeHoldCommands();

        auto fb = std::make_shared<ActionT::Feedback>();
        fb->state          = 0;
        fb->progress       = 0.0;
        fb->status_message = "Planning trajectory with MoveIt2 ...";
        publishFeedback(fb);
        return ComputeResult::RUNNING;
    }

    if (state == PlanState::FAILED)
    {
        std::string msg;
        {
            std::lock_guard<std::mutex> lk(msg_mutex_);
            msg = plan_error_msg_;
        }
        RCLCPP_ERROR(node_->get_logger(), "[%s] aborting: %s", name_.c_str(), msg.c_str());
        return ComputeResult::ABORTED;
    }

    // DONE — trajectory sent to JTC; hand off
    auto fb = std::make_shared<ActionT::Feedback>();
    fb->state          = 1;
    fb->progress       = 0.1;
    fb->status_message = "Trajectory sent to fr3_husky_joint_trajectory_controller";
    publishFeedback(fb);

    result_error_code_ = 0;
    return ComputeResult::SUCCEEDED;
}

// ============================================================
// Stop / Result
// ============================================================

void MoveToJoint::onStop(StopReason reason)
{
    cancel_flag_.store(true, std::memory_order_relaxed);

    if (reason != StopReason::SUCCEEDED)
        model_updater_.haltCommands();

    const char* rs = (reason == StopReason::CANCELED)  ? "canceled"  :
                     (reason == StopReason::SUCCEEDED)  ? "succeeded" :
                     (reason == StopReason::ABORTED)    ? "aborted"   : "none";
    RCLCPP_INFO(node_->get_logger(), "[%s] stopped (%s)", name_.c_str(), rs);
}

MoveToJoint::ResultPtr MoveToJoint::makeResult(StopReason reason)
{
    auto result = std::make_shared<ActionT::Result>();
    if (reason == StopReason::SUCCEEDED)
    {
        result->success    = true;
        result->message    = "Planning succeeded; trajectory sent to "
                             "fr3_husky_joint_trajectory_controller";
        result->error_code = 0;
    }
    else if (reason == StopReason::CANCELED)
    {
        result->success    = false;
        result->message    = "Cancelled";
        result->error_code = 0;
    }
    else
    {
        result->success    = false;
        std::lock_guard<std::mutex> lk(msg_mutex_);
        result->message    = plan_error_msg_.empty() ? "Aborted" : plan_error_msg_;
        result->error_code = result_error_code_;
    }
    return result;
}

REGISTER_FR3_HUSKY_ACTION_SERVER(MoveToJoint, "fr3_husky_move_to_joint")

}  // namespace fr3_husky_controller::servers::fr3_husky

/*
# send goal 
ros2 action send_goal /fr3_husky_move_to_joint fr3_husky_msgs/action/MoveToJoint  \
    "{joint_names: [left_fr3_joint1, left_fr3_joint2, left_fr3_joint3, left_fr3_joint4, left_fr3_joint5, left_fr3_joint6, left_fr3_joint7],
    target_positions: [0., -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    max_velocity_scaling_factor: 0.1,
    max_acceleration_scaling_factor: 0.1}"
*/