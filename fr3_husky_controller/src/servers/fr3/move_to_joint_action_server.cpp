#include <fr3_husky_controller/servers/fr3/move_to_joint_action_server.hpp>

#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <action_msgs/msg/goal_status.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/parameter_client.hpp>

namespace fr3_husky_controller::servers::fr3
{

// ============================================================
// Anonymous helpers
// ============================================================

namespace
{

FR3ModelUpdater& getFR3ModelUpdater(ModelUpdaterBase& model_updater,
                                    const std::string& server_name)
{
    auto* p = dynamic_cast<FR3ModelUpdater*>(&model_updater);
    if (!p)
    {
        throw std::runtime_error("[" + server_name + "] requires FR3ModelUpdater");
    }
    return *p;
}

std::optional<std::string> extractStatusField(const std::string& message, const std::string& key)
{
    const std::string token = key + "=";
    const auto pos = message.find(token);
    if (pos == std::string::npos)
    {
        return std::nullopt;
    }

    const auto value_start = pos + token.size();
    const auto value_end = message.find(' ', value_start);
    if (value_end == std::string::npos)
    {
        return message.substr(value_start);
    }

    return message.substr(value_start, value_end - value_start);
}

}  // namespace

// ============================================================
// Constructor / Destructor
// ============================================================

MoveToJoint::MoveToJoint(const std::string& name, const NodePtr& node,
                         ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
    fr3_model_updater_(getFR3ModelUpdater(model_updater, name)),
    motion_gate_(getGlobalMotionGate())
{
    const auto clock_type = node_->get_clock()->get_clock_type();
    last_joint_state_stamp_ = rclcpp::Time(0, 0, clock_type);
    last_joint_state_receive_time_ = rclcpp::Time(0, 0, clock_type);

    require_fresh_joint_state_ = node_->declare_parameter<bool>(
        name_ + ".require_fresh_joint_state", true);
    max_joint_state_age_ms_ = node_->declare_parameter<int>(
        name_ + ".max_joint_state_age_ms", 150);
    require_fresh_moveit_state_ = node_->declare_parameter<bool>(
        name_ + ".require_fresh_moveit_state", false);
    max_moveit_state_age_ms_ = node_->declare_parameter<int>(
        name_ + ".max_moveit_state_age_ms", 150);
    require_cartesian_stopped_ = node_->declare_parameter<bool>(
        name_ + ".require_cartesian_stopped", true);
    cartesian_get_status_service_ = node_->declare_parameter<std::string>(
        name_ + ".cartesian_get_status_service", "/cartesian_executor/get_status");
    cartesian_settle_delay_ms_ = node_->declare_parameter<int>(
        name_ + ".cartesian_settle_delay_ms", 1500);
    const std::string require_trajectory_stopped_param =
        name_ + ".require_trajectory_stopped";

    if (node_->has_parameter(require_trajectory_stopped_param))
    {
        require_trajectory_stopped_ =
            node_->get_parameter(require_trajectory_stopped_param).as_bool();
    }
    else
    {
        require_trajectory_stopped_ =
            node_->declare_parameter<bool>(require_trajectory_stopped_param, true);
    }

    const std::string trajectory_get_status_service_param =
        name_ + ".trajectory_get_status_service";
    if (node_->has_parameter(trajectory_get_status_service_param))
    {
        trajectory_get_status_service_ =
            node_->get_parameter(trajectory_get_status_service_param).as_string();
    }
    else
    {
        trajectory_get_status_service_ =
            node_->declare_parameter<std::string>(
                trajectory_get_status_service_param, "/trajectory_executor/get_status");
    }

    const std::string trajectory_settle_delay_ms_param =
        name_ + ".trajectory_settle_delay_ms";
    if (node_->has_parameter(trajectory_settle_delay_ms_param))
    {
        trajectory_settle_delay_ms_ =
            node_->get_parameter(trajectory_settle_delay_ms_param).as_int();
    }
    else
    {
        trajectory_settle_delay_ms_ =
            node_->declare_parameter<int>(trajectory_settle_delay_ms_param, 1500);
    }

    wait_for_cartesian_status_timeout_ms_ = node_->declare_parameter<int>(
        name_ + ".wait_for_cartesian_status_timeout_ms", 300);
    reject_if_cartesian_status_unavailable_ = node_->declare_parameter<bool>(
        name_ + ".reject_if_cartesian_status_unavailable", true);
    joint_state_topic_ = node_->declare_parameter<std::string>(
        name_ + ".joint_state_topic", "/right_fr3/joint_states");

    // --- Dedicated node for MoveGroupInterface ---
    // MoveGroupInterface needs its own rclcpp::Node (lifecycle nodes are not
    // accepted). We do NOT assign it to a persistent executor: MoveGroupInterface
    // calls rclcpp::spin_until_future_complete() internally, which creates a
    // temporary SingleThreadedExecutor. Pre-spinning the node in a second executor
    // causes std::terminate() inside the MoveGroupInterface constructor.
    rclcpp::NodeOptions mgi_opts;
    mgi_opts.automatically_declare_parameters_from_overrides(true);
    moveit_node_ = rclcpp::Node::make_shared(name_ + "_mgi", mgi_opts);

    // --- Action client → fr3_joint_trajectory_controller ---
    jtc_client_ = rclcpp_action::create_client<FJT>(
        node_, "fr3_joint_trajectory_controller");
    cartesian_status_client_ = node_->create_client<std_srvs::srv::Trigger>(
        cartesian_get_status_service_);
    trajectory_status_client_ = node_->create_client<std_srvs::srv::Trigger>(
        trajectory_get_status_service_);

    // --- Subscribe to JTC action status to detect busy state ---
    // If the JTC is already executing a trajectory (e.g. the user started
    // fr3_moveit.launch.py and is sending motion commands via RViz), we
    // refuse new MoveToJoint goals so we do not interfere.
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    jtc_status_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
        "fr3_joint_trajectory_controller/_action/status",
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

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_,
        10,
        std::bind(&MoveToJoint::jointStateCallback, this, std::placeholders::_1));

    // --- Derive planning group and valid joint prefixes from robot_names ---
    const auto& rnames = model_updater.robot_names_;
    const auto& arm_id = model_updater.arm_id_;

    if (rnames.size() >= 2)
    {
        planning_group_ = "dual_fr3_arm";
        for (const auto& rn : rnames)
            valid_joint_prefixes_.insert(rn + "_" + arm_id + "_");
    }
    else if (rnames.size() == 1)
    {
        planning_group_ = rnames[0] + "_" + arm_id + "_arm";
        valid_joint_prefixes_.insert(rnames[0] + "_" + arm_id + "_");
    }
    else
    {
        planning_group_ = arm_id + "_arm";
    }

    // --- Create persistent MoveGroupInterface ---
    {
        bool params_ok = false;
        try
        {
            auto pc = std::make_shared<rclcpp::SyncParametersClient>(moveit_node_, "move_group");
            if (pc->wait_for_service(std::chrono::seconds(5)))
            {
                const std::vector<std::string> param_names{
                    "robot_description", "robot_description_semantic"};
                for (const auto& pname : param_names)
                {
                    if (moveit_node_->has_parameter(pname)) continue;
                    auto vals = pc->get_parameters({pname});
                    if (!vals.empty() &&
                        vals[0].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
                        moveit_node_->declare_parameter(pname, vals[0].get_parameter_value());
                }
                params_ok = moveit_node_->has_parameter("robot_description_semantic");
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[%s] move_group param service not available within 5 s — "
                            "MoveGroupInterface not created", name_.c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] param forwarding failed: %s", name_.c_str(), e.what());
        }

        if (params_ok)
        {
            try
            {
                mgi_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                    moveit_node_, planning_group_,
                    std::shared_ptr<tf2_ros::Buffer>{},
                    rclcpp::Duration::from_seconds(10.0));
                mgi_->startStateMonitor(std::max(1.0, max_moveit_state_age_ms_ / 1000.0));
                RCLCPP_INFO(node_->get_logger(),
                            "[%s] MoveGroupInterface ready", name_.c_str());
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[%s] MoveGroupInterface creation failed: %s",
                            name_.c_str(), e.what());
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] MoveToJoint created — group=%s", name_.c_str(),
                planning_group_.c_str());
}

MoveToJoint::~MoveToJoint()
{
    // Signal planning thread to stop.
    // Note: if the thread is blocked inside MoveGroupInterface::plan() there
    // is no way to interrupt it; we must wait for the planning timeout to expire
    // (set via mgi.setPlanningTime() inside runPlanning()).
    cancel_flag_.store(true, std::memory_order_relaxed);
    if (planning_thread_.joinable())
    {
        planning_thread_.join();
    }
}

// ============================================================
// Helpers
// ============================================================

void MoveToJoint::writeHoldCommands()
{
    const size_t dof = model_updater_.manipulator_dof_;
    const Eigen::VectorXd qdot_zero = Eigen::VectorXd::Zero(dof);

    if (model_updater_.HasEffortCommandInterface())
    {
        if (!fr3_model_updater_.robot_controller_)
        {
            model_updater_.haltCommands();
            return;
        }
        // PD + gravity at latched positions: τ = Kp(q_hold-q) + Kv(0-qdot) + g
        const Eigen::VectorXd torque =
            fr3_model_updater_.robot_controller_->moveJointTorqueStep(
                q_hold_, qdot_zero, false);
        fr3_model_updater_.torque_desired_total_ = torque - fr3_model_updater_.g_total_;
        fr3_model_updater_.writeCommand(fr3_model_updater_.torque_desired_total_);
    }
    else if (model_updater_.HasVelocityCommandInterface())
    {
        fr3_model_updater_.qdot_desired_total_ = qdot_zero;
        fr3_model_updater_.writeCommand(fr3_model_updater_.qdot_desired_total_);
    }
    else if (model_updater_.HasPositionCommandInterface())
    {
        fr3_model_updater_.q_desired_total_ = q_hold_;
        fr3_model_updater_.writeCommand(fr3_model_updater_.q_desired_total_);
    }
}

// ============================================================
// Background planning thread
// ============================================================

void MoveToJoint::runPlanning()
{
    RCLCPP_INFO(node_->get_logger(), "[%s] planning group: %s", name_.c_str(),
                planning_group_.c_str());

    // ---- Phase 1: MoveIt2 planning ----------------------------------------
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_ok = false;
    std::string err_msg;

    if (!mgi_)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s] MoveGroupInterface not available. "
                     "Is move_group running? (launch fr3_moveit.launch.py, not "
                     "fr3_action_controller.launch.py alone)",
                     name_.c_str());
        std::lock_guard<std::mutex> lk(msg_mutex_);
        plan_error_msg_ = "MoveGroupInterface not available — start move_group first";
        plan_state_.store(PlanState::FAILED, std::memory_order_release);
        return;
    }

    {
        std::string reason;
        RCLCPP_INFO(node_->get_logger(), "[fr3_move_to_joint] safety preflight start");
        if (!runPreflightSafetyChecks(&reason))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[fr3_move_to_joint] rejected by safety preflight: %s",
                         reason.c_str());
            {
                std::lock_guard<std::mutex> lk(msg_mutex_);
                plan_error_msg_ = "Safety preflight failed: " + reason;
            }
            plan_state_.store(PlanState::FAILED, std::memory_order_release);
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "[fr3_move_to_joint] safety preflight passed");
    }

    if (!cancel_flag_.load(std::memory_order_relaxed))
    {
        try
        {
            mgi_->setPlanningTime(10.0);
            mgi_->setStartStateToCurrentState();
            mgi_->setMaxVelocityScalingFactor(goal_vel_scale_);
            mgi_->setMaxAccelerationScalingFactor(goal_acc_scale_);

            // Build target: fill ALL group joints from hardware q_total_,
            // then override with the goal's specified joints.
            // Using q_total_ directly avoids depending on move_group's /joint_states
            // subscription (which can fail in MuJoCo / fake-hardware mode).
            std::map<std::string, double> target;
            {
                const auto& rnames = model_updater_.robot_names_;
                const auto& arm_id = model_updater_.arm_id_;
                const Eigen::VectorXd& q_hw = fr3_model_updater_.q_total_;

                // q_total_ layout: [rnames[0]_joint1..7, rnames[1]_joint1..7, ...]
                size_t q_idx = 0;
                for (const auto& rn : rnames)
                {
                    for (int j = 1; j <= FR3_DOF; ++j, ++q_idx)
                    {
                        const std::string jname = rn + "_" + arm_id + "_joint" + std::to_string(j);
                        target[jname] = (q_idx < static_cast<size_t>(q_hw.size()))
                                        ? q_hw[q_idx] : 0.0;
                    }
                }

                // Override with goal joints
                for (size_t i = 0; i < goal_joint_names_.size(); ++i)
                    target[goal_joint_names_[i]] = goal_target_positions_[i];
            }

            if (!mgi_->setJointValueTarget(target))
            {
                err_msg = "setJointValueTarget failed "
                          "(joint names or values out of bounds?)";
            }
            else if (cancel_flag_.load(std::memory_order_relaxed))
            {
                err_msg = "Cancelled before planning";
            }
            else
            {
                const auto ec = mgi_->plan(plan);
                if (ec)
                {
                    plan_ok = true;
                    #if ROS_DISTRO == 22
                        RCLCPP_INFO(node_->get_logger(),
                                    "[%s] planning succeeded (%zu waypoints)",
                                    name_.c_str(),
                                    plan.trajectory_.joint_trajectory.points.size());
                    #else
                        RCLCPP_INFO(node_->get_logger(),
                                    "[%s] planning succeeded (%zu waypoints)",
                                    name_.c_str(),
                                    plan.trajectory.joint_trajectory.points.size());
                    #endif
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

    {
        std::string reason;
        if (!runPostPlanSafetyChecks(&reason))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[fr3_move_to_joint] rejected by post-plan safety check: %s",
                         reason.c_str());
            {
                std::lock_guard<std::mutex> lk(msg_mutex_);
                plan_error_msg_ = "Post-plan safety check failed: " + reason;
            }
            plan_state_.store(PlanState::FAILED, std::memory_order_release);
            return;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[fr3_move_to_joint] post-plan safety check passed; preparing handoff");
    }

    // ---- Phase 2: Prepare handoff to fr3_joint_trajectory_controller ----
    if (!jtc_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        std::lock_guard<std::mutex> lk(msg_mutex_);
        plan_error_msg_ = "fr3_joint_trajectory_controller not available";
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(),
                     plan_error_msg_.c_str());
        plan_state_.store(PlanState::FAILED, std::memory_order_release);
        return;
    }

    plan_state_.store(PlanState::READY, std::memory_order_release);

    // Wait until MoveToJoint has fully finished before sending the lower-priority
    // JTC goal. This avoids the controller consuming JTC's activate request while
    // MoveToJoint is still the active_server_.
    while (isActive() && !cancel_flag_.load(std::memory_order_relaxed))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!handoff_requested_.load(std::memory_order_acquire) ||
        cancel_flag_.load(std::memory_order_relaxed))
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] skipping JTC handoff because the goal did not finish successfully",
                    name_.c_str());
        return;
    }

    FJT::Goal jtc_goal;
    #if ROS_DISTRO == 22
        jtc_goal.trajectory = plan.trajectory_.joint_trajectory;
    #else
        jtc_goal.trajectory = plan.trajectory.joint_trajectory;
    #endif

    auto send_opts = rclcpp_action::Client<FJT>::SendGoalOptions();
    send_opts.goal_response_callback =
        [this](std::shared_ptr<GoalHandleFJT> gh)
        {
            if (!gh)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[%s] fr3_joint_trajectory_controller rejected handoff goal",
                             name_.c_str());
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[%s] trajectory handed off to fr3_joint_trajectory_controller",
                            name_.c_str());
            }
        };

    jtc_client_->async_send_goal(jtc_goal, send_opts);
}

bool MoveToJoint::validateGoalInputs(std::string* reason) const
{
    if (goal_joint_names_.empty())
    {
        if (reason)
        {
            *reason = "joint_names is empty";
        }
        return false;
    }

    if (goal_joint_names_.size() != goal_target_positions_.size())
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "joint_names.size()=" << goal_joint_names_.size()
                << " != target_positions.size()=" << goal_target_positions_.size();
            *reason = oss.str();
        }
        return false;
    }

    if (!valid_joint_prefixes_.empty())
    {
        for (const auto& jname : goal_joint_names_)
        {
            bool ok = false;
            for (const auto& prefix : valid_joint_prefixes_)
            {
                if (jname.rfind(prefix, 0) == 0)
                {
                    ok = true;
                    break;
                }
            }
            if (!ok)
            {
                if (reason)
                {
                    std::ostringstream oss;
                    oss << "joint '" << jname << "' not in planning group '"
                        << planning_group_ << "'";
                    *reason = oss.str();
                }
                return false;
            }
        }
    }

    return true;
}

bool MoveToJoint::isJointStateFresh(std::string* reason) const
{
    if (!require_fresh_joint_state_)
    {
        return true;
    }

    rclcpp::Time reference_time;
    const auto now = node_->now();

    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (!have_joint_state_)
        {
            if (reason)
            {
                *reason = "joint state missing: no joint state received";
            }
            return false;
        }

        if (last_joint_state_stamp_.nanoseconds() > 0)
        {
            reference_time = last_joint_state_stamp_;
        }
        else
        {
            reference_time = last_joint_state_receive_time_;
        }
    }

    const auto age_ms = static_cast<int64_t>((now - reference_time).nanoseconds() / 1000000LL);

    // Reject timestamps that appear to be from the future (> 20 ms ahead).
    if (age_ms < -20)
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "joint state timestamp is in the future, age_ms=" << age_ms;
            *reason = oss.str();
        }
        return false;
    }

    if (age_ms > max_joint_state_age_ms_)
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "joint state stale, age=" << age_ms << " ms";
            *reason = oss.str();
        }
        return false;
    }

    RCLCPP_DEBUG(node_->get_logger(), "[fr3_move_to_joint] joint state age=%ld ms", age_ms);
    return true;
}

bool MoveToJoint::isMoveItCurrentStateFresh(std::string* reason)
{
    if (!require_fresh_moveit_state_)
    {
        return true;
    }

    if (!mgi_)
    {
        if (reason)
        {
            *reason = "MoveIt current state unavailable: MoveGroupInterface not available";
        }
        return false;
    }

    const double wait_seconds = std::max(0.001, max_moveit_state_age_ms_ / 1000.0);

    try
    {
        if (!mgi_->startStateMonitor(wait_seconds))
        {
            if (reason)
            {
                std::ostringstream oss;
                oss << "MoveIt current state unavailable: state monitor did not start within "
                    << max_moveit_state_age_ms_ << " ms";
                *reason = oss.str();
            }
            return false;
        }

        // getCurrentState(wait) is treated as an additional availability check.
        // Primary freshness protection comes from direct /joint_states freshness validation.
        const auto t_request = node_->now();
        auto current_state = mgi_->getCurrentState(wait_seconds);
        if (!current_state)
        {
            if (reason)
            {
                std::ostringstream oss;
                oss << "MoveIt current state stale/unavailable: failed to fetch current state within "
                    << max_moveit_state_age_ms_ << " ms";
                *reason = oss.str();
            }
            return false;
        }

        const auto wait_ms = (node_->now() - t_request).nanoseconds() / 1000000LL;
        RCLCPP_INFO(node_->get_logger(),
                    "[fr3_move_to_joint] MoveIt current state fresh (waited %ld ms, limit=%ld ms)",
                    static_cast<long>(wait_ms), static_cast<long>(max_moveit_state_age_ms_));
    }
    catch (const std::exception& e)
    {
        if (reason)
        {
            *reason = std::string("MoveIt current state stale/unavailable: ") + e.what();
        }
        return false;
    }

    return true;
}

bool MoveToJoint::isCartesianExecutorSafe(std::string* reason)
{
    if (!require_cartesian_stopped_)
    {
        return true;
    }

    if (!cartesian_status_client_)
    {
        if (reason)
        {
            *reason = "cartesian executor status client not available";
        }
        return false;
    }

    const auto timeout = std::chrono::milliseconds(wait_for_cartesian_status_timeout_ms_);
    if (!cartesian_status_client_->wait_for_service(timeout))
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "cartesian executor status service unavailable after "
                << wait_for_cartesian_status_timeout_ms_ << " ms";
            *reason = oss.str();
        }
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = cartesian_status_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "cartesian executor status request timed out after "
                << wait_for_cartesian_status_timeout_ms_ << " ms";
            *reason = oss.str();
        }
        RCLCPP_INFO(node_->get_logger(),
                    "[fr3_move_to_joint] rejected: Cartesian status service timeout");
        return false;
    }

    std::shared_ptr<std_srvs::srv::Trigger::Response> response;
    try
    {
        response = future.get();
    }
    catch (const std::exception& e)
    {
        if (reason)
        {
            *reason = std::string("cartesian executor status request failed: ") + e.what();
        }
        return false;
    }

    if (!response)
    {
        if (reason)
        {
            *reason = "cartesian executor status request returned no response";
        }
        return false;
    }

    const std::string& message = response->message;
    if (!response->success)
    {
        if (reason)
        {
            *reason = "cartesian executor not safe: " + message;
        }
        return false;
    }

    const auto state = extractStatusField(message, "state");
    if (!state)
    {
        if (reason)
        {
            *reason = "cartesian executor status parse failure: missing state";
        }
        return false;
    }

    if (*state == "ACTIVE" || *state == "CANCELING" || *state == "ABORTED")
    {
        if (reason)
        {
            *reason = "cartesian executor state=" + *state;
        }
        return false;
    }

    if (*state != "STOPPED")
    {
        if (reason)
        {
            *reason = "cartesian executor unsafe state=" + *state;
        }
        return false;
    }

    const auto stop_in_progress = extractStatusField(message, "stop_in_progress");
    if (stop_in_progress && *stop_in_progress == "true")
    {
        if (reason)
        {
            *reason = "cartesian executor stop_in_progress=true";
        }
        return false;
    }

    const auto last_stop_time = extractStatusField(message, "last_stop_time");
    if (!last_stop_time)
    {
        if (reason)
        {
            *reason = "cartesian executor status parse failure: missing last_stop_time";
        }
        return false;
    }

    double last_stop_seconds = 0.0;
    try
    {
        last_stop_seconds = std::stod(*last_stop_time);
    }
    catch (const std::exception&)
    {
        if (reason)
        {
            *reason = "invalid last_stop_time";
        }
        RCLCPP_INFO(node_->get_logger(),
                    "[fr3_move_to_joint] rejected: invalid last_stop_time");
        return false;
    }

    if (!std::isfinite(last_stop_seconds) || last_stop_seconds <= 0.0)
    {
        if (reason)
        {
            *reason = "invalid last_stop_time";
        }
        RCLCPP_INFO(node_->get_logger(),
                    "[fr3_move_to_joint] rejected: invalid last_stop_time");
        return false;
    }

    const auto now_seconds = node_->now().seconds();
    const double since_stop_ms = (now_seconds - last_stop_seconds) * 1000.0;
    if (since_stop_ms < -20.0)
    {
        if (reason)
        {
            *reason = "cartesian executor stop timestamp is in the future";
        }
        return false;
    }

    if (since_stop_ms < static_cast<double>(cartesian_settle_delay_ms_))
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "cartesian executor stopped only " << static_cast<int64_t>(since_stop_ms)
                << " ms ago";
            *reason = oss.str();
        }
        return false;
    }

    return true;
}

bool MoveToJoint::isTrajectoryExecutorSafe(std::string* reason)
{
    if (!require_trajectory_stopped_)
    {
        return true;
    }

    if (!trajectory_status_client_)
    {
        if (reason)
        {
            *reason = "trajectory executor status client not available";
        }
        return false;
    }

    const auto timeout = std::chrono::milliseconds(wait_for_cartesian_status_timeout_ms_);
    if (!trajectory_status_client_->wait_for_service(timeout))
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "trajectory executor status service unavailable after "
                << wait_for_cartesian_status_timeout_ms_ << " ms";
            *reason = oss.str();
        }
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = trajectory_status_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "trajectory executor status request timed out after "
                << wait_for_cartesian_status_timeout_ms_ << " ms";
            *reason = oss.str();
        }
        return false;
    }

    std::shared_ptr<std_srvs::srv::Trigger::Response> response;
    try
    {
        response = future.get();
    }
    catch (const std::exception& e)
    {
        if (reason)
        {
            *reason = std::string("trajectory executor status request failed: ") + e.what();
        }
        return false;
    }

    if (!response)
    {
        if (reason)
        {
            *reason = "trajectory executor status request returned no response";
        }
        return false;
    }

    const std::string& message = response->message;
    const auto state = extractStatusField(message, "state");
    const auto stop_in_progress = extractStatusField(message, "stop_in_progress");
    const auto last_stop_time = extractStatusField(message, "last_stop_time");

    if (!state || !stop_in_progress || !last_stop_time)
    {
        if (reason)
        {
            *reason = "trajectory executor status parse failure";
        }
        return false;
    }

    if (*state != "STOPPED" || *stop_in_progress == "true")
    {
        if (reason)
        {
            *reason = "trajectory executor not stopped";
        }
        return false;
    }

    double last_stop_seconds = 0.0;
    try
    {
        last_stop_seconds = std::stod(*last_stop_time);
    }
    catch (const std::exception&)
    {
        if (reason)
        {
            *reason = "invalid trajectory last_stop_time";
        }
        return false;
    }

    const auto now_seconds = node_->now().seconds();
    const double since_stop_ms = (now_seconds - last_stop_seconds) * 1000.0;
    if (since_stop_ms < static_cast<double>(trajectory_settle_delay_ms_))
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "trajectory executor stopped only " << static_cast<int64_t>(since_stop_ms)
                << " ms ago";
            *reason = oss.str();
        }
        return false;
    }

    return true;
}

bool MoveToJoint::runPreflightSafetyChecks(std::string* reason)
{
    std::string local_reason;

    if (!validateGoalInputs(&local_reason))
    {
        if (reason)
        {
            *reason = local_reason;
        }
        return false;
    }

    if (!isJointStateFresh(&local_reason))
    {
        if (reason)
        {
            *reason = local_reason;
        }
        return false;
    }

    if (!isCartesianExecutorSafe(&local_reason))
    {
        if (reason)
        {
            *reason = local_reason;
        }
        return false;
    }

    if (!isTrajectoryExecutorSafe(&local_reason))
    {
        if (reason)
        {
            *reason = local_reason;
        }
        return false;
    }

    if (!isMoveItCurrentStateFresh(&local_reason))
    {
        if (reason)
        {
            *reason = local_reason;
        }
        return false;
    }

    return true;
}

bool MoveToJoint::runPostPlanSafetyChecks(std::string* reason)
{
    std::string local_reason;

    if (!isJointStateFresh(&local_reason))
    {
        if (reason)
        {
            *reason = local_reason;
        }
        return false;
    }

    if (!isCartesianExecutorSafe(&local_reason))
    {
        if (reason)
        {
            *reason = local_reason;
        }
        return false;
    }

    if (!isTrajectoryExecutorSafe(&local_reason))
    {
        if (reason)
        {
            *reason = local_reason;
        }
        return false;
    }

    if (require_fresh_moveit_state_ && !isMoveItCurrentStateFresh(&local_reason))
    {
        if (reason)
        {
            *reason = local_reason;
        }
        return false;
    }

    return true;
}

void MoveToJoint::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    const auto receive_time = node_->now();
    const auto stamp = rclcpp::Time(msg->header.stamp);

    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    have_joint_state_ = true;
    last_joint_state_receive_time_ = receive_time;
    if (stamp.nanoseconds() > 0)
    {
        last_joint_state_stamp_ = stamp;
    }
    else
    {
        last_joint_state_stamp_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    }
}

// ============================================================
// Goal lifecycle
// ============================================================

bool MoveToJoint::acceptGoal(const ActionT::Goal& goal)
{
    if (goal.joint_names.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject: joint_names is empty",
                    name_.c_str());
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

    // Validate that every joint name belongs to the planning group.
    if (!valid_joint_prefixes_.empty())
    {
        for (const auto& jname : goal.joint_names)
        {
            bool ok = false;
            for (const auto& prefix : valid_joint_prefixes_)
            {
                if (jname.rfind(prefix, 0) == 0) { ok = true; break; }
            }
            if (!ok)
            {
                std::string valid;
                for (const auto& p : valid_joint_prefixes_) valid += p + "* ";
                RCLCPP_WARN(node_->get_logger(),
                            "[%s] Reject: joint '%s' not in group '%s' (valid: %s)",
                            name_.c_str(), jname.c_str(), planning_group_.c_str(),
                            valid.c_str());
                return false;
            }
        }
    }

    // Reject if fr3_joint_trajectory_controller is already executing.
    // This happens when the user launched fr3_moveit.launch.py and MoveIt
    // is actively sending motions through JTC.
    if (jtc_busy_.load(std::memory_order_relaxed))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Reject: fr3_joint_trajectory_controller is already "
                    "executing. MoveIt may be actively controlling the robot — "
                    "wait for the current motion to finish or cancel it first.",
                    name_.c_str());
        return false;
    }

    return true;
}

void MoveToJoint::onGoalAccepted(const ActionT::Goal& goal)
{
    // Cache goal
    goal_joint_names_     = goal.joint_names;
    goal_target_positions_ = goal.target_positions;
    goal_vel_scale_ = (goal.max_velocity_scaling_factor > 0.0)
                      ? goal.max_velocity_scaling_factor : 0.1;
    goal_acc_scale_ = (goal.max_acceleration_scaling_factor > 0.0)
                      ? goal.max_acceleration_scaling_factor : 0.1;

    // Reset per-goal state
    cancel_flag_.store(false, std::memory_order_relaxed);
    handoff_requested_.store(false, std::memory_order_relaxed);
    plan_state_.store(PlanState::PLANNING, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(msg_mutex_);
        plan_error_msg_.clear();
    }

    // Join previous planning thread if somehow still alive (shouldn't happen
    // in normal flow, but be safe)
    if (planning_thread_.joinable())
    {
        planning_thread_.join();
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] goal accepted — awaiting safety preflight",
                name_.c_str());
}

void MoveToJoint::onStart()
{
    std::string gate_reason;
    if (!motion_gate_->tryAcquire(MotionGate::Owner::MOVE_TO_JOINT, name_, &gate_reason))
    {
        result_error_code_ = ActionT::Result::ERROR_GOAL_REJECTED;
        {
            std::lock_guard<std::mutex> lk(msg_mutex_);
            plan_error_msg_ = gate_reason;
        }
        plan_state_.store(PlanState::FAILED, std::memory_order_release);
        RCLCPP_WARN(node_->get_logger(), "[%s] start rejected by motion gate: %s", name_.c_str(), gate_reason.c_str());
        return;
    }
    gate_acquired_ = true;

    // Latch current joint positions; writeHoldCommands() uses these every
    // control cycle while planning is in progress.
    fr3_model_updater_.setInitFromCurrent();
    q_hold_           = fr3_model_updater_.q_total_;
    result_error_code_ = ActionT::Result::ERROR_PLAN_FAILED;  // default until success

    // Launch background planning only after safety preflight passes.
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

    RCLCPP_INFO(node_->get_logger(), "[%s] started, holding position during planning",
                name_.c_str());
}

// ============================================================
// Control loop
// ============================================================

MoveToJoint::ComputeResult MoveToJoint::compute(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
{
    const auto state = plan_state_.load(std::memory_order_acquire);

    if (state == PlanState::PLANNING)
    {
        // Still planning — hold position and publish feedback
        writeHoldCommands();

        auto fb = std::make_shared<ActionT::Feedback>();
        fb->state          = 0;  // PLANNING
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

    // state == PlanState::READY
    // Planning is done and the background thread is waiting to hand off the
    // trajectory once this server is no longer active.
    auto fb = std::make_shared<ActionT::Feedback>();
    fb->state          = 1;  // EXECUTING
    fb->progress       = 0.1;
    fb->status_message = "Trajectory planned; handing off to fr3_joint_trajectory_controller";
    publishFeedback(fb);

    handoff_requested_.store(true, std::memory_order_release);
    result_error_code_ = 0;
    return ComputeResult::SUCCEEDED;
}

// ============================================================
// Stop / Result
// ============================================================

void MoveToJoint::onStop(StopReason reason)
{
    // Signal the background thread to exit early only on canceled/aborted goals.
    // On success it must stay alive long enough to hand the planned trajectory
    // to JTC after this server releases control.
    if (reason != StopReason::SUCCEEDED)
    {
        cancel_flag_.store(true, std::memory_order_relaxed);
    }

    // Only halt hardware if we are NOT handing off to JTC.
    // When SUCCEEDED, JTC takes over as active_server_ in the next cycle
    // and will write its own commands; halting here would cause a one-cycle
    // torque glitch.
    if (reason != StopReason::SUCCEEDED)
    {
        model_updater_.haltCommands();
    }

    const char* rs = (reason == StopReason::CANCELED)  ? "canceled"  :
                     (reason == StopReason::SUCCEEDED)  ? "succeeded" :
                     (reason == StopReason::ABORTED)    ? "aborted"   : "none";

    if (gate_acquired_)
    {
        motion_gate_->release(MotionGate::Owner::MOVE_TO_JOINT);
        gate_acquired_ = false;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] stopped (%s)", name_.c_str(), rs);
}

MoveToJoint::ResultPtr MoveToJoint::makeResult(StopReason reason)
{
    auto result = std::make_shared<ActionT::Result>();
    if (reason == StopReason::SUCCEEDED)
    {
        result->success    = true;
        result->message    = "Planning succeeded; trajectory ready for "
                             "fr3_joint_trajectory_controller handoff";
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

// Register this server into the global FR3 registry
REGISTER_FR3_ACTION_SERVER(MoveToJoint, "fr3_move_to_joint")

}  // namespace fr3_husky_controller::servers::fr3
/*
# send goal 
ros2 action send_goal /fr3_move_to_joint fr3_husky_msgs/action/MoveToJoint  \
    "{joint_names: [left_fr3_joint1, left_fr3_joint2, left_fr3_joint3, left_fr3_joint4, left_fr3_joint5, left_fr3_joint6, left_fr3_joint7],
    target_positions: [0., -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    max_velocity_scaling_factor: 0.1,
    max_acceleration_scaling_factor: 0.1}"

ros2 action send_goal /fr3_move_to_joint fr3_husky_msgs/action/MoveToJoint  \
    "{joint_names: [right_fr3_joint1, right_fr3_joint2, right_fr3_joint3, right_fr3_joint4, right_fr3_joint5, right_fr3_joint6, right_fr3_joint7],
    target_positions: [0., -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    max_velocity_scaling_factor: 0.1,
    max_acceleration_scaling_factor: 0.1}"

ros2 action send_goal /fr3_move_to_joint fr3_husky_msgs/action/MoveToJoint  \
    "{joint_names: [left_fr3_joint1, left_fr3_joint2, left_fr3_joint3, left_fr3_joint4, left_fr3_joint5, left_fr3_joint6, left_fr3_joint7, right_fr3_joint1, right_fr3_joint2, right_fr3_joint3, right_fr3_joint4, right_fr3_joint5, right_fr3_joint6, right_fr3_joint7],
    target_positions: [0., -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    max_velocity_scaling_factor: 0.1,
    max_acceleration_scaling_factor: 0.1}"
*/
