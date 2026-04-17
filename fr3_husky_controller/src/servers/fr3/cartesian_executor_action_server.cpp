#include <fr3_husky_controller/servers/fr3/cartesian_executor_action_server.hpp>

#include <fr3_husky_controller/utils/dyros_math.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cmath>

namespace fr3_husky_controller::servers::fr3
{

namespace
{
FR3ModelUpdater& getFR3ModelUpdater(ModelUpdaterBase& model_updater, const std::string& server_name)
{
    auto* fr3_model_updater = dynamic_cast<FR3ModelUpdater*>(&model_updater);
    if (!fr3_model_updater)
    {
        throw std::runtime_error("[" + server_name + "] requires FR3ModelUpdater");
    }
    return *fr3_model_updater;
}

double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

}  // namespace

CartesianExecutor::CartesianExecutor(
    const std::string& name,
    const NodePtr& node,
    ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
  fr3_model_updater_(getFR3ModelUpdater(model_updater, name))
{
    twist_topic_name_ = node_->declare_parameter<std::string>(
        name_ + ".twist_topic_name", "/cartesian_cmd/twist");
    reset_target_service_name_ = node_->declare_parameter<std::string>(
        name_ + ".reset_target_service_name", "/cartesian_executor/reset_target");

    vel_lpf_tau_ = node_->declare_parameter<double>(
        name_ + ".vel_lpf_tau", 0.03);
    cmd_timeout_sec_ = node_->declare_parameter<double>(
        name_ + ".cmd_timeout_sec", 0.20);

    workspace_min_ << 0.20, -0.45, 0.05;
    workspace_max_ << 0.75,  0.45, 0.65;

    {
        std::vector<double> workspace_min_vec = node_->declare_parameter<std::vector<double>>(
            name_ + ".workspace_min", {0.20, -0.45, 0.05});
        std::vector<double> workspace_max_vec = node_->declare_parameter<std::vector<double>>(
            name_ + ".workspace_max", {0.75, 0.45, 0.65});

        if (workspace_min_vec.size() == 3 && workspace_max_vec.size() == 3)
        {
            workspace_min_ << workspace_min_vec[0], workspace_min_vec[1], workspace_min_vec[2];
            workspace_max_ << workspace_max_vec[0], workspace_max_vec[1], workspace_max_vec[2];
        }
        else
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "[%s] workspace_min/max parameter size invalid, using defaults.",
                name_.c_str());
        }
    }

    twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        twist_topic_name_,
        10,
        std::bind(&CartesianExecutor::subTwistCallback, this, std::placeholders::_1));

    reset_target_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        reset_target_service_name_,
        std::bind(
            &CartesianExecutor::handleResetTarget,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    filtered_lin_vel_cmd_.setZero();
    target_pose_.setIdentity();
    target_rotation_.setIdentity();

    ee_data.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] CartesianExecutor created", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "[%s] twist topic: %s", name_.c_str(), twist_topic_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "[%s] reset service: %s", name_.c_str(), reset_target_service_name_.c_str());
}

bool CartesianExecutor::acceptGoal(const ActionT::Goal& goal)
{
    if (!model_updater_.HasEffortCommandInterface())
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] Reject action: effort command interface is required",
            name_.c_str());
        return false;
    }

    if (goal.mode < 0 || goal.mode > 3)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] Reject action: mode must be 0 to 3 (0: CLIK, 1: OSF, 2: QPIK, 3: QPID). goal.mode=%d",
            name_.c_str(),
            static_cast<int>(goal.mode));
        return false;
    }

    if (!fr3_model_updater_.robot_data_->hasLinkFrame(goal.ee_name))
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] Reject action: ee_name [%s] is not in URDF.",
            name_.c_str(),
            goal.ee_name.c_str());
        return false;
    }

    return true;
}

void CartesianExecutor::onGoalAccepted(const ActionT::Goal& goal)
{
    control_mode_ = goal.mode;
    control_ee_name_ = goal.ee_name;

    // Keep same behavior as old PS4 server: fixed orientation for now.
    move_ori_ = false;

    // These are kept for compatibility with the OmegaHaptic action type.
    // In this executor, the input manager should usually do the teleop-side scaling already.
    hapic_pos_multiplier_     = static_cast<double>(goal.hapic_pos_multiplier);
    hapic_ori_multiplier_     = static_cast<double>(goal.hapic_ori_multiplier);
    hapic_lin_vel_multiplier_ = static_cast<double>(goal.hapic_lin_vel_multiplier);
    hapic_ang_vel_multiplier_ = static_cast<double>(goal.hapic_ang_vel_multiplier);

    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] Goal accepted: mode=%d, ee_name=%s",
        name_.c_str(),
        control_mode_,
        control_ee_name_.c_str());
}

void CartesianExecutor::onStart()
{
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_lin_vel_cmd_.setZero();
        have_twist_cmd_ = false;
        latest_cmd_stamp_ = node_->now();
    }

    filtered_lin_vel_cmd_.setZero();
    reset_target_requested_ = true;

    ee_data.clear();
    ee_data[control_ee_name_] = drc::TaskSpaceData::Zero();
    ee_data[control_ee_name_].x = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    ee_data[control_ee_name_].xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    ee_data[control_ee_name_].xddot.setZero();
    ee_data[control_ee_name_].setInit();
    ee_data[control_ee_name_].setDesired();

    target_pose_ = ee_data[control_ee_name_].x;
    target_rotation_ = target_pose_.linear();

    RCLCPP_INFO(node_->get_logger(), "[%s] started", name_.c_str());
}

CartesianExecutor::ComputeResult CartesianExecutor::compute(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/)
{
    ee_data[control_ee_name_].x    = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    ee_data[control_ee_name_].xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    ee_data[control_ee_name_].xddot.setZero();

    if (reset_target_requested_)
    {
        reset_target_requested_ = false;
        ee_data[control_ee_name_].setInit();
        target_pose_ = ee_data[control_ee_name_].x;
        target_rotation_ = target_pose_.linear();
        filtered_lin_vel_cmd_.setZero();

        RCLCPP_INFO(node_->get_logger(), "[%s] target pose reset to current EE pose", name_.c_str());
    }

    Eigen::Vector3d raw_lin_cmd_local = Eigen::Vector3d::Zero();

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);

        if (have_twist_cmd_)
        {
            const double cmd_age = (time - latest_cmd_stamp_).seconds();
            if (cmd_age <= cmd_timeout_sec_)
            {
                // Incoming twist is interpreted in the TCP / EE local frame.
                raw_lin_cmd_local = latest_lin_vel_cmd_ * hapic_lin_vel_multiplier_;
            }
            else
            {
                raw_lin_cmd_local.setZero();
            }
        }
    }

    // Rotate TCP-local command into the base frame.
    const Eigen::Matrix3d R_base_ee = ee_data[control_ee_name_].x.linear();
    const Eigen::Vector3d raw_lin_cmd = R_base_ee * raw_lin_cmd_local;

    // Simple first-order low-pass filter.
    const double alpha = std::exp(-fr3_model_updater_.dt_ / vel_lpf_tau_);
    filtered_lin_vel_cmd_ = alpha * filtered_lin_vel_cmd_ + (1.0 - alpha) * raw_lin_cmd;

    Eigen::Vector6d target_vel = Eigen::Vector6d::Zero();

    target_pose_.translation() += filtered_lin_vel_cmd_ * fr3_model_updater_.dt_;

    Eigen::Vector3d p = target_pose_.translation();
    p.x() = clamp(p.x(), workspace_min_.x(), workspace_max_.x());
    p.y() = clamp(p.y(), workspace_min_.y(), workspace_max_.y());
    p.z() = clamp(p.z(), workspace_min_.z(), workspace_max_.z());
    target_pose_.translation() = p;

    target_pose_.linear() = target_rotation_;
    target_vel.head<3>() = filtered_lin_vel_cmd_;

    ee_data[control_ee_name_].x_desired = target_pose_;
    ee_data[control_ee_name_].xdot_desired = target_vel;

    bool is_qp_solved = true;
    std::string time_verbose;

    switch (control_mode_)
    {
        case 0: // CLIK
            fr3_model_updater_.robot_controller_->CLIKStep(
                ee_data,
                fr3_model_updater_.qdot_desired_total_);

            fr3_model_updater_.q_desired_total_ =
                fr3_model_updater_.q_total_ +
                fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;

            fr3_model_updater_.torque_desired_total_ =
                fr3_model_updater_.robot_controller_->moveJointTorqueStep(
                    fr3_model_updater_.q_desired_total_,
                    fr3_model_updater_.qdot_desired_total_,
                    false);
            break;

        case 1: // OSF
            fr3_model_updater_.robot_controller_->OSFStep(
                ee_data,
                fr3_model_updater_.torque_desired_total_);
            break;

        case 2: // QPIK
            is_qp_solved = fr3_model_updater_.robot_controller_->QPIKStep(
                ee_data,
                fr3_model_updater_.qdot_desired_total_,
                time_verbose);

            if (!is_qp_solved)
            {
                fr3_model_updater_.qdot_desired_total_.setZero();
            }

            fr3_model_updater_.q_desired_total_ =
                fr3_model_updater_.q_total_ +
                fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;

            fr3_model_updater_.torque_desired_total_ =
                fr3_model_updater_.robot_controller_->moveJointTorqueStep(
                    fr3_model_updater_.q_desired_total_,
                    fr3_model_updater_.qdot_desired_total_,
                    false);
            break;

        case 3: // QPID
            is_qp_solved = fr3_model_updater_.robot_controller_->QPIDStep(
                ee_data,
                fr3_model_updater_.torque_desired_total_,
                time_verbose);

            if (!is_qp_solved)
            {
                fr3_model_updater_.torque_desired_total_ =
                    fr3_model_updater_.robot_data_->getGravity();
            }
            break;

        default:
            break;
    }

    fr3_model_updater_.writeCommand(
        fr3_model_updater_.torque_desired_total_ - fr3_model_updater_.g_total_);

    auto fb = std::make_shared<ActionT::Feedback>();
    fb->is_qp_solved = is_qp_solved;
    fb->time_verbose = time_verbose;
    publishFeedback(fb);

    return ComputeResult::RUNNING;
}

void CartesianExecutor::onStop(StopReason reason)
{
    model_updater_.haltCommands();

    filtered_lin_vel_cmd_.setZero();

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_lin_vel_cmd_.setZero();
        have_twist_cmd_ = false;
    }

    const char* reason_str = "none";
    if (reason == StopReason::CANCELED)  reason_str = "canceled";
    if (reason == StopReason::SUCCEEDED) reason_str = "succeeded";
    if (reason == StopReason::ABORTED)   reason_str = "aborted";

    RCLCPP_INFO(node_->get_logger(), "[%s] stopped (%s)", name_.c_str(), reason_str);
}

CartesianExecutor::ResultPtr CartesianExecutor::makeResult(StopReason /*reason*/)
{
    auto result = std::make_shared<ActionT::Result>();
    result->is_completed = true;
    return result;
}

void CartesianExecutor::subTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    latest_lin_vel_cmd_.x() = msg->twist.linear.x;
    latest_lin_vel_cmd_.y() = msg->twist.linear.y;
    latest_lin_vel_cmd_.z() = msg->twist.linear.z;
    latest_cmd_stamp_ = node_->now();
    have_twist_cmd_ = true;
}

void CartesianExecutor::handleResetTarget(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    reset_target_requested_ = true;

    response->success = true;
    response->message = "Cartesian executor target reset requested.";

    RCLCPP_INFO(node_->get_logger(), "[%s] target reset requested", name_.c_str());
}

// Register this server into global registry
REGISTER_FR3_ACTION_SERVER(CartesianExecutor, "cartesian_executor")

}  // namespace fr3_husky_controller::servers::fr3