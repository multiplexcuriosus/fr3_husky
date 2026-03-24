#include <fr3_husky_controller/servers/fr3/ps4_haptic_action_server.hpp>
#include <fr3_husky_controller/utils/dyros_math.h>

#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

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

double applyDeadzone(double x, double deadzone)
{
    if (std::abs(x) < deadzone) return 0.0;
    return x;
}

double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

}  // namespace

Ps4Haptic::Ps4Haptic(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
  fr3_model_updater_(getFR3ModelUpdater(model_updater, name))
{
    joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&Ps4Haptic::subJoyCallback, this, std::placeholders::_1));

    joy_axes_.assign(8, 0.0f);
    joy_buttons_.assign(16, 0);

    filtered_lin_vel_cmd_.setZero();
    target_pose_.setIdentity();
    target_rotation_.setIdentity();

    deadzone_ = 0.08;
    max_vx_ = 0.08;   // m/s
    max_vy_ = 0.08;   // m/s
    max_vz_ = 0.05;   // m/s
    vel_lpf_tau_ = 0.03;  // seconds, conservative smoothing

    // workspace clamp in base/world frame
    workspace_min_ << 0.20, -0.45, 0.05;
    workspace_max_ << 0.75,  0.45, 0.65;

    deadman_button_idx_ = 5;  // R1 on many DS4 mappings
    left_stick_x_idx_   = 0;  // left/right
    left_stick_y_idx_   = 1;  // up/down
    right_stick_y_idx_  = 4;  // up/down on many mappings

    is_mouse_mode_on_ = false;
    prev_deadman_pressed_ = false;

    ee_data.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] Ps4Haptic created", name_.c_str());
}

bool Ps4Haptic::acceptGoal(const ActionT::Goal& goal)
{
    if (!model_updater_.HasEffortCommandInterface())
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject action: effort command interface is required",
                    name_.c_str());
        return false;
    }

    if (goal.mode < 0 || goal.mode > 3)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Reject action: mode must be 0 to 3 (0: CLIK, 1: OSF, 2: QPIK, 3: QPID). goal.mode=%d",
                    name_.c_str(), static_cast<int>(goal.mode));
        return false;
    }

    if (!fr3_model_updater_.robot_data_->hasLinkFrame(goal.ee_name))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Reject action: ee_name [%s] is not in URDF.",
                    name_.c_str(), goal.ee_name.c_str());
        return false;
    }

    return true;
}

void Ps4Haptic::onGoalAccepted(const ActionT::Goal& goal)
{
    control_mode_ = goal.mode;
    control_ee_name_ = goal.ee_name;

    // We intentionally ignore orientation teleop for PS4 for now.
    move_ori_ = false;

    // Reuse these as translational speed scalings if you want action-level tuning.
    hapic_pos_multiplier_     = static_cast<double>(goal.hapic_pos_multiplier);
    hapic_ori_multiplier_     = static_cast<double>(goal.hapic_ori_multiplier);
    hapic_lin_vel_multiplier_ = static_cast<double>(goal.hapic_lin_vel_multiplier);
    hapic_ang_vel_multiplier_ = static_cast<double>(goal.hapic_ang_vel_multiplier);
}

void Ps4Haptic::onStart()
{
    {
        std::lock_guard<std::mutex> lock(joy_mutex_);
        std::fill(joy_axes_.begin(), joy_axes_.end(), 0.0f);
        std::fill(joy_buttons_.begin(), joy_buttons_.end(), 0);
    }

    filtered_lin_vel_cmd_.setZero();
    prev_deadman_pressed_ = false;
    is_mouse_mode_on_ = false;

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

Ps4Haptic::ComputeResult Ps4Haptic::compute(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    ee_data[control_ee_name_].x    = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    ee_data[control_ee_name_].xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    ee_data[control_ee_name_].xddot.setZero();

    std::vector<float> axes_local;
    std::vector<int> buttons_local;
    {
        std::lock_guard<std::mutex> lock(joy_mutex_);
        axes_local = joy_axes_;
        buttons_local = joy_buttons_;
    }

    auto getAxis = [&](int idx) -> double
    {
        if (idx < 0 || idx >= static_cast<int>(axes_local.size())) return 0.0;
        return static_cast<double>(axes_local[idx]);
    };

    auto getButton = [&](int idx) -> bool
    {
        if (idx < 0 || idx >= static_cast<int>(buttons_local.size())) return false;
        return buttons_local[idx] != 0;
    };

    const bool deadman_pressed = getButton(deadman_button_idx_);

    if (!is_mouse_mode_on_ && deadman_pressed)
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] PS4 teleop activated", name_.c_str());
        is_mouse_mode_on_ = true;

        ee_data[control_ee_name_].setInit();
        target_pose_ = ee_data[control_ee_name_].x;
        target_rotation_ = target_pose_.linear();
        filtered_lin_vel_cmd_.setZero();
    }
    else if (is_mouse_mode_on_ && !deadman_pressed)
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] PS4 teleop deactivated", name_.c_str());
        is_mouse_mode_on_ = false;

        ee_data[control_ee_name_].setInit();
        filtered_lin_vel_cmd_.setZero();
    }

    Eigen::Vector3d raw_lin_cmd = Eigen::Vector3d::Zero();

    // Typical DS4 mapping:
    // left stick y -> forward/back
    // left stick x -> left/right
    // right stick y -> z
    //
    // Signs may need one quick flip after you test.
    const double lx = applyDeadzone(getAxis(left_stick_x_idx_), deadzone_);
    const double ly = applyDeadzone(getAxis(left_stick_y_idx_), deadzone_);
    const double ry = applyDeadzone(getAxis(right_stick_y_idx_), deadzone_);

    raw_lin_cmd.x() = (ly) * max_vx_ * hapic_lin_vel_multiplier_;
    raw_lin_cmd.y() = (lx) * max_vy_ * hapic_lin_vel_multiplier_;
    raw_lin_cmd.z() = (ry) * max_vz_ * hapic_lin_vel_multiplier_;

    // Low-pass filter commanded Cartesian velocity
    filtered_lin_vel_cmd_ = dyros_math::lowPassFilter(
        raw_lin_cmd, filtered_lin_vel_cmd_, fr3_model_updater_.dt_, vel_lpf_tau_);

    Eigen::Vector6d target_vel = Eigen::Vector6d::Zero();

    if (is_mouse_mode_on_)
    {
        target_pose_.translation() += filtered_lin_vel_cmd_ * fr3_model_updater_.dt_;

        // clamp to safe workspace
        Eigen::Vector3d p = target_pose_.translation();
        p.x() = clamp(p.x(), workspace_min_.x(), workspace_max_.x());
        p.y() = clamp(p.y(), workspace_min_.y(), workspace_max_.y());
        p.z() = clamp(p.z(), workspace_min_.z(), workspace_max_.z());
        target_pose_.translation() = p;

        // keep fixed orientation
        target_pose_.linear() = target_rotation_;

        target_vel.head<3>() = filtered_lin_vel_cmd_;
    }
    else
    {
        // hold pose while inactive
        target_pose_ = ee_data[control_ee_name_].x;
        target_rotation_ = target_pose_.linear();
        filtered_lin_vel_cmd_.setZero();
        target_vel.setZero();
    }

    ee_data[control_ee_name_].x_desired = target_pose_;
    ee_data[control_ee_name_].xdot_desired = target_vel;

    bool is_qp_solved = true;
    std::string time_verbose = "";

    switch (control_mode_)
    {
        case 0: // CLIK
             fr3_model_updater_.robot_controller_->CLIKStep(ee_data, fr3_model_updater_.qdot_desired_total_);
            fr3_model_updater_.q_desired_total_ =
                fr3_model_updater_.q_total_ + fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;
            fr3_model_updater_.torque_desired_total_ =
                fr3_model_updater_.robot_controller_->moveJointTorqueStep(
                    fr3_model_updater_.q_desired_total_,
                    fr3_model_updater_.qdot_desired_total_,
                    false);
            break;

        case 1: // OSF
                fr3_model_updater_.robot_controller_->OSFStep(ee_data, fr3_model_updater_.torque_desired_total_);
            break;

        case 2: // QPIK
            is_qp_solved = fr3_model_updater_.robot_controller_->QPIKStep(
                ee_data, fr3_model_updater_.qdot_desired_total_, time_verbose);
            if (!is_qp_solved) fr3_model_updater_.qdot_desired_total_.setZero();

            fr3_model_updater_.q_desired_total_ =
                fr3_model_updater_.q_total_ + fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;
            fr3_model_updater_.torque_desired_total_ =
                fr3_model_updater_.robot_controller_->moveJointTorqueStep(
                    fr3_model_updater_.q_desired_total_,
                    fr3_model_updater_.qdot_desired_total_,
                    false);
            break;

        case 3: // QPID
            is_qp_solved = fr3_model_updater_.robot_controller_->QPIDStep(
                ee_data, fr3_model_updater_.torque_desired_total_, time_verbose);
            if (!is_qp_solved)
                fr3_model_updater_.torque_desired_total_ = fr3_model_updater_.robot_data_->getGravity();
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

    prev_deadman_pressed_ = deadman_pressed;
    return ComputeResult::RUNNING;
}

void Ps4Haptic::onStop(StopReason reason)
{
    model_updater_.haltCommands();

    const char* reason_str = "none";
    if (reason == StopReason::CANCELED)  reason_str = "canceled";
    if (reason == StopReason::SUCCEEDED) reason_str = "succeeded";
    if (reason == StopReason::ABORTED)   reason_str = "aborted";

    RCLCPP_INFO(node_->get_logger(), "[%s] stopped (%s)", name_.c_str(), reason_str);
}

Ps4Haptic::ResultPtr Ps4Haptic::makeResult(StopReason /*reason*/)
{
    auto result = std::make_shared<ActionT::Result>();
    result->is_completed = true;
    return result;
}

void Ps4Haptic::subJoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joy_mutex_);
    joy_axes_ = msg->axes;
    joy_buttons_ = msg->buttons;
}

// Register this server into global registry
REGISTER_FR3_ACTION_SERVER(Ps4Haptic, "ps4_haptic")

}  // namespace fr3_husky_controller::servers::fr3