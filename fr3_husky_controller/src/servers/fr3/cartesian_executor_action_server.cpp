#include <fr3_husky_controller/servers/fr3/cartesian_executor_action_server.hpp>

#include <fr3_husky_controller/utils/dyros_math.h>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <iomanip>
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

double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

template <typename T, typename NodeT>
T declareOrGetParameter(
    const std::shared_ptr<NodeT>& node,
    const std::string& param_name,
    const T& default_value)
{
    if (node->has_parameter(param_name))
    {
        return node->get_parameter(param_name).template get_value<T>();
    }
    return node->template declare_parameter<T>(param_name, default_value);
}

}  // namespace

CartesianExecutor::CartesianExecutor(
    const std::string& name,
    const NodePtr& node,
    ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
    fr3_model_updater_(getFR3ModelUpdater(model_updater, name)),
    motion_gate_(getGlobalMotionGate())
{
    const auto clock_type = node_->get_clock()->get_clock_type();
    last_start_time_ = rclcpp::Time(0, 0, clock_type);
    last_stop_time_ = rclcpp::Time(0, 0, clock_type);
    last_command_time_ = rclcpp::Time(0, 0, clock_type);
    last_status_publish_time_ = rclcpp::Time(0, 0, clock_type);

    twist_topic_name_ = node_->declare_parameter<std::string>(
        name_ + ".twist_topic_name", "/cartesian_cmd/twist");
    reset_target_service_name_ = node_->declare_parameter<std::string>(
        name_ + ".reset_target_service_name", "/cartesian_executor/reset_target");
    status_topic_name_ = node_->declare_parameter<std::string>(
        name_ + ".status_topic_name", "/cartesian_executor/status");
    get_status_service_name_ = node_->declare_parameter<std::string>(
        name_ + ".get_status_service_name", "/cartesian_executor/get_status");

    vel_lpf_tau_ = node_->declare_parameter<double>(
        name_ + ".vel_lpf_tau", 0.03);
    cmd_timeout_sec_ = node_->declare_parameter<double>(
        name_ + ".cmd_timeout_sec", 0.20);
    only_x_axis_ = declareOrGetParameter<bool>(
        node_, name_ + ".only_x_axis", false);
    abort_on_repeated_qp_failure_ = node_->declare_parameter<bool>(
        name_ + ".abort_on_repeated_qp_failure", false);
    max_consecutive_qp_failures_ = node_->declare_parameter<int>(
        name_ + ".max_consecutive_qp_failures", 20);
    if (max_consecutive_qp_failures_ < 1)
    {
        max_consecutive_qp_failures_ = 1;
    }
    linear_twist_frame_ = node_->declare_parameter<std::string>(
        name_ + ".linear_twist_frame", "base");
    angular_twist_frame_ = node_->declare_parameter<std::string>(
        name_ + ".angular_twist_frame", "ee");

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

    status_pub_ = node_->create_publisher<std_msgs::msg::String>(status_topic_name_, 10);

    reset_target_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        reset_target_service_name_,
        std::bind(
            &CartesianExecutor::handleResetTarget,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    get_status_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        get_status_service_name_,
        std::bind(
            &CartesianExecutor::handleGetStatus,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    filtered_lin_vel_cmd_.setZero();
    filtered_ang_vel_cmd_.setZero();
    target_pose_.setIdentity();
    target_rotation_.setIdentity();

    timing_last_report_time_ = std::chrono::steady_clock::now();

    ee_data.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] CartesianExecutor created", name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "[%s] twist topic: %s", name_.c_str(), twist_topic_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "[%s] reset service: %s", name_.c_str(), reset_target_service_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "[%s] status topic: %s", name_.c_str(), status_topic_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "[%s] get_status service: %s", name_.c_str(), get_status_service_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "[%s] linear twist frame: %s", name_.c_str(), linear_twist_frame_.c_str());
    RCLCPP_INFO(node_->get_logger(), "[%s] angular twist frame: %s", name_.c_str(), angular_twist_frame_.c_str());

    publishStatus(true);
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

    // Respect orientation motion control from the incoming teleop action goal.
    move_ori_ = goal.move_orientation;

    // These are kept for compatibility with the OmegaHaptic action type.
    // In this executor, the input manager should usually do the teleop-side scaling already.
    haptic_pos_multiplier_     = static_cast<double>(goal.haptic_pos_multiplier);
    haptic_ori_multiplier_     = static_cast<double>(goal.haptic_ori_multiplier);
    haptic_lin_vel_multiplier_ = static_cast<double>(goal.haptic_lin_vel_multiplier);
    haptic_ang_vel_multiplier_ = static_cast<double>(goal.haptic_ang_vel_multiplier);

    RCLCPP_INFO(
        node_->get_logger(),
        "[cartesian_executor] goal accepted: mode=%d, ee=%s, move_orientation=%s, lin_mult=%.3f, ang_mult=%.3f",
        control_mode_,
        control_ee_name_.c_str(),
        move_ori_ ? "true" : "false",
        haptic_lin_vel_multiplier_,
        haptic_ang_vel_multiplier_);
}

void CartesianExecutor::onStart()
{
    std::string gate_reason;
    if (!motion_gate_->tryAcquire(MotionGate::Owner::CARTESIAN_EXECUTOR, name_, &gate_reason))
    {
        start_failed_ = true;
        start_failure_reason_ = gate_reason;
        gate_acquired_ = false;

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            executor_state_ = ExecutorState::STOPPED;
            stop_in_progress_ = false;
            last_stop_time_ = node_->now();
            last_stop_reason_ = LastStopReason::ABORTED;
        }

        publishStatus(true);
        RCLCPP_WARN(node_->get_logger(), "[%s] start rejected by motion gate: %s", name_.c_str(), gate_reason.c_str());
        return;
    }
    else
    {
        start_failed_ = false;
        start_failure_reason_.clear();
        gate_acquired_ = true;
    }

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_lin_vel_cmd_.setZero();
        latest_ang_vel_cmd_.setZero();
        have_twist_cmd_ = false;
        latest_cmd_stamp_ = node_->now();
    }

    filtered_lin_vel_cmd_.setZero();
    filtered_ang_vel_cmd_.setZero();
    reset_target_requested_ = true;

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        executor_state_ = ExecutorState::ACTIVE;
        stop_in_progress_ = false;
        last_start_time_ = node_->now();
        last_stop_reason_ = LastStopReason::NONE;
    }

    consecutive_qp_failures_ = 0;

    ee_data.clear();
    ee_data[control_ee_name_] = drc::TaskSpaceData::Zero();
    ee_data[control_ee_name_].x = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    ee_data[control_ee_name_].xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    ee_data[control_ee_name_].xddot.setZero();
    ee_data[control_ee_name_].setInit();
    ee_data[control_ee_name_].setDesired();

    target_pose_ = ee_data[control_ee_name_].x;
    target_rotation_ = target_pose_.linear();

    RCLCPP_INFO(node_->get_logger(), "[cartesian_executor] state ACTIVE");
    publishStatus(true);

    RCLCPP_INFO(node_->get_logger(), "[%s] started", name_.c_str());
}

CartesianExecutor::ComputeResult CartesianExecutor::compute(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/)
{
    if (start_failed_)
    {
        RCLCPP_ERROR(node_->get_logger(), "[%s] aborting due to start failure: %s", name_.c_str(), start_failure_reason_.c_str());
        return ComputeResult::ABORTED;
    }

    const auto timing_start = std::chrono::steady_clock::now();

    bool should_publish_status = false;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (last_status_publish_time_.nanoseconds() == 0 ||
            (time - last_status_publish_time_) >= rclcpp::Duration::from_seconds(status_publish_period_.count() / 1000.0))
        {
            last_status_publish_time_ = time;
            should_publish_status = true;
        }
    }

    // if (should_publish_status)
    // {
    //     publishStatus(false);
    // }

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
        filtered_ang_vel_cmd_.setZero();

        RCLCPP_INFO(node_->get_logger(), "[%s] target pose reset to current EE pose", name_.c_str());
    }

    Eigen::Vector3d raw_lin_cmd_input = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_ang_cmd_input = Eigen::Vector3d::Zero();

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);

        if (have_twist_cmd_)
        {
            const double cmd_age = (time - latest_cmd_stamp_).seconds();
            if (cmd_age <= cmd_timeout_sec_)
            {
                // Incoming twist frame is configurable.
                // Internally, raw_lin_cmd and raw_ang_cmd are converted to base-frame commands.
                raw_lin_cmd_input = latest_lin_vel_cmd_ * haptic_lin_vel_multiplier_;
                if (move_ori_)
                {
                    raw_ang_cmd_input = latest_ang_vel_cmd_ * haptic_ang_vel_multiplier_;
                }
            }
            else
            {
                raw_lin_cmd_input.setZero();
                raw_ang_cmd_input.setZero();
                latest_lin_vel_cmd_.setZero();
                latest_ang_vel_cmd_.setZero();
                filtered_lin_vel_cmd_.setZero();
                filtered_ang_vel_cmd_.setZero();
                have_twist_cmd_ = false;
            }
        }
    }

    const Eigen::Matrix3d R_base_ee = ee_data[control_ee_name_].x.linear();
    Eigen::Vector3d raw_lin_cmd = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_ang_cmd = Eigen::Vector3d::Zero();

    if (linear_twist_frame_ == "ee")
    {
        raw_lin_cmd = R_base_ee * raw_lin_cmd_input;
    }
    else if (linear_twist_frame_ == "base")
    {
        raw_lin_cmd = raw_lin_cmd_input;
    }
    else
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] invalid linear_twist_frame '%s'; expected 'ee' or 'base'. Zeroing linear command.",
            name_.c_str(),
            linear_twist_frame_.c_str());
    }

    if (angular_twist_frame_ == "ee")
    {
        raw_ang_cmd = R_base_ee * raw_ang_cmd_input;
    }
    else if (angular_twist_frame_ == "base")
    {
        raw_ang_cmd = raw_ang_cmd_input;
    }
    else
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] invalid angular_twist_frame '%s'; expected 'ee' or 'base'. Zeroing angular command.",
            name_.c_str(),
            angular_twist_frame_.c_str());
    }

    // Simple first-order low-pass filter.
    const double alpha = std::exp(-fr3_model_updater_.dt_ / vel_lpf_tau_);
    filtered_lin_vel_cmd_ = alpha * filtered_lin_vel_cmd_ + (1.0 - alpha) * raw_lin_cmd*haptic_lin_vel_multiplier_;
    filtered_ang_vel_cmd_ = alpha * filtered_ang_vel_cmd_ + (1.0 - alpha) * raw_ang_cmd;


    if (only_x_axis_)
    {
        filtered_lin_vel_cmd_(1) = 0.0;  // y
        filtered_lin_vel_cmd_(2) = 0.0;  // z
    }

    Eigen::Vector6d target_vel = Eigen::Vector6d::Zero();

    target_pose_.translation() += filtered_lin_vel_cmd_ * fr3_model_updater_.dt_;

    Eigen::Vector3d p = target_pose_.translation();
    p.x() = clamp(p.x(), workspace_min_.x(), workspace_max_.x());
    p.y() = clamp(p.y(), workspace_min_.y(), workspace_max_.y());
    p.z() = clamp(p.z(), workspace_min_.z(), workspace_max_.z());
    target_pose_.translation() = p;

    
    if (move_ori_)
    {
        const double angle = filtered_ang_vel_cmd_.norm() * fr3_model_updater_.dt_;
        if (angle > 1e-9)
        {
            const Eigen::Vector3d axis = filtered_ang_vel_cmd_.normalized();
            const Eigen::Matrix3d dR = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

            // filtered_ang_vel_cmd_ is in base frame, so left-multiply.
            target_rotation_ = dR * target_rotation_;

            Eigen::Quaterniond q(target_rotation_);
            q.normalize();
            target_rotation_ = q.toRotationMatrix();
        }
    }

    target_pose_.linear() = target_rotation_;
    target_vel.head<3>() = filtered_lin_vel_cmd_;
    if (move_ori_)
    {
        target_vel.tail<3>() = filtered_ang_vel_cmd_;
    }

    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[cartesian_executor] move_ori=%s linear_twist_frame=%s angular_twist_frame=%s raw_lin_input=[%.4f %.4f %.4f] raw_lin_base=[%.4f %.4f %.4f] raw_ang_input=[%.4f %.4f %.4f] raw_ang_base=[%.4f %.4f %.4f] filtered_ang_norm=%.4f",
        move_ori_ ? "true" : "false",
        linear_twist_frame_.c_str(),
        angular_twist_frame_.c_str(),
        raw_lin_cmd_input.x(),
        raw_lin_cmd_input.y(),
        raw_lin_cmd_input.z(),
        raw_lin_cmd.x(),
        raw_lin_cmd.y(),
        raw_lin_cmd.z(),
        raw_ang_cmd_input.x(),
        raw_ang_cmd_input.y(),
        raw_ang_cmd_input.z(),
        raw_ang_cmd.x(),
        raw_ang_cmd.y(),
        raw_ang_cmd.z(),
        filtered_ang_vel_cmd_.norm());

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

    if (control_mode_ == 2 || control_mode_ == 3)
    {
        if (is_qp_solved)
        {
            consecutive_qp_failures_ = 0;
        }
        else
        {
            consecutive_qp_failures_++;
            if (abort_on_repeated_qp_failure_ &&
                consecutive_qp_failures_ >= max_consecutive_qp_failures_)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "[%s] aborting: repeated QP failures (%d >= %d)",
                    name_.c_str(),
                    consecutive_qp_failures_,
                    max_consecutive_qp_failures_);
                return ComputeResult::ABORTED;
            }
        }
    }
    else
    {
        consecutive_qp_failures_ = 0;
    }

    fr3_model_updater_.writeCommand(
        fr3_model_updater_.torque_desired_total_ - fr3_model_updater_.g_total_);

    // auto fb = std::make_shared<ActionT::Feedback>();
    // fb->is_qp_solved = is_qp_solved;
    // fb->time_verbose = time_verbose;
    // publishFeedback(fb);

    updateComputeTimingDebug(timing_start);

    return ComputeResult::RUNNING;
}

void CartesianExecutor::updateComputeTimingDebug(
    std::chrono::steady_clock::time_point start_time)
{
    if (!enable_compute_timing_debug_)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double compute_us =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count());

    if (compute_us > timing_max_compute_us_)
    {
        timing_max_compute_us_ = compute_us;
    }
    if (compute_us > 800.0)
    {
        ++timing_overrun_800us_;
    }
    if (compute_us > 1000.0)
    {
        ++timing_overrun_1000us_;
    }
    ++timing_call_count_;

    const double elapsed_report_s =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - timing_last_report_time_).count())
        * 1e-6;

    if (elapsed_report_s >= 1.0)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "[cartesian_executor timing] compute_us_current=%.1f compute_us_max=%.1f"
            " calls=%" PRIu64 " overruns_gt_800us=%" PRIu64 " overruns_gt_1000us=%" PRIu64,
            compute_us,
            timing_max_compute_us_,
            timing_call_count_,
            timing_overrun_800us_,
            timing_overrun_1000us_);

        timing_last_report_time_ = now;
        timing_max_compute_us_   = 0.0;
        timing_overrun_800us_    = 0;
        timing_overrun_1000us_   = 0;
        timing_call_count_       = 0;
    }
}

void CartesianExecutor::onStop(StopReason reason)
{
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        executor_state_ = ExecutorState::CANCELING;
        stop_in_progress_ = true;
    }
    RCLCPP_INFO(node_->get_logger(), "[cartesian_executor] state CANCELING");
    publishStatus(true);

    model_updater_.haltCommands();

    filtered_lin_vel_cmd_.setZero();
    filtered_ang_vel_cmd_.setZero();

    if (!control_ee_name_.empty() && fr3_model_updater_.robot_data_->hasLinkFrame(control_ee_name_))
    {
        const auto current_pose = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
        target_pose_ = current_pose;
        target_rotation_ = current_pose.linear();
    }

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_lin_vel_cmd_.setZero();
        latest_ang_vel_cmd_.setZero();
        have_twist_cmd_ = false;
    }

    const auto stop_time = node_->now();
    LastStopReason final_reason = LastStopReason::NONE;
    if (reason == StopReason::SUCCEEDED)
    {
        final_reason = LastStopReason::SUCCEEDED;
    }
    else if (reason == StopReason::CANCELED)
    {
        final_reason = LastStopReason::CANCELED;
    }
    else if (reason == StopReason::ABORTED)
    {
        final_reason = LastStopReason::ABORTED;
    }

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_stop_time_ = stop_time;
        stop_in_progress_ = false;
        executor_state_ = ExecutorState::STOPPED;
        last_stop_reason_ = final_reason;
    }

    const char* reason_str = "none";
    if (reason == StopReason::CANCELED)  reason_str = "canceled";
    if (reason == StopReason::SUCCEEDED) reason_str = "succeeded";
    if (reason == StopReason::ABORTED)   reason_str = "aborted";

    RCLCPP_INFO(
        node_->get_logger(),
        "[cartesian_executor] state STOPPED, last_stop_time=%.6f",
        stop_time.seconds());
    publishStatus(true);

    if (gate_acquired_)
    {
        motion_gate_->release(MotionGate::Owner::CARTESIAN_EXECUTOR);
        gate_acquired_ = false;
    }

    consecutive_qp_failures_ = 0;

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
    const auto now = node_->now();

    std::lock_guard<std::mutex> lock(cmd_mutex_);

    latest_lin_vel_cmd_.x() = msg->twist.linear.x;
    latest_lin_vel_cmd_.y() = msg->twist.linear.y;
    latest_lin_vel_cmd_.z() = msg->twist.linear.z;
    latest_ang_vel_cmd_.x() = msg->twist.angular.x;
    latest_ang_vel_cmd_.y() = msg->twist.angular.y;
    latest_ang_vel_cmd_.z() = msg->twist.angular.z;
    latest_cmd_stamp_ = now;
    have_twist_cmd_ = true;

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        last_command_time_ = now;
    }
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

void CartesianExecutor::handleGetStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    ExecutorState state;

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        state = executor_state_;
    }

    response->success = (state != ExecutorState::ACTIVE && state != ExecutorState::CANCELING);
    response->message = buildStatusMessage();
    RCLCPP_INFO(node_->get_logger(), "[cartesian_executor] status: %s", response->message.c_str());
}

void CartesianExecutor::publishStatus(bool log_status)
{
    if (!status_pub_)
    {
        return;
    }

    std_msgs::msg::String msg;
    msg.data = buildStatusMessage();
    status_pub_->publish(msg);

    if (log_status)
    {
        RCLCPP_INFO(node_->get_logger(), "[cartesian_executor] status: %s", msg.data.c_str());
    }
}

std::string CartesianExecutor::buildStatusMessage() const
{
    ExecutorState executor_state;
    rclcpp::Time last_start_time;
    rclcpp::Time last_stop_time;
    LastStopReason last_stop_reason;
    bool stop_in_progress;

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        executor_state = executor_state_;
        last_start_time = last_start_time_;
        last_stop_time = last_stop_time_;
        last_stop_reason = last_stop_reason_;
        stop_in_progress = stop_in_progress_;
    }

    const auto now = node_->now();
    const auto last_cmd_age_ms = getLastCommandAgeMs(now);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "state=" << executorStateToString(executor_state)
        << " last_stop_reason=" << lastStopReasonToString(last_stop_reason)
        << " last_start_time=" << last_start_time.seconds()
        << " last_stop_time=" << last_stop_time.seconds()
        << " stop_in_progress=" << (stop_in_progress ? "true" : "false")
        << " last_cmd_age_ms=" << last_cmd_age_ms;
    return oss.str();
}

std::string CartesianExecutor::executorStateToString(ExecutorState state) const
{
    switch (state)
    {
        case ExecutorState::STOPPED:
            return "STOPPED";
        case ExecutorState::ACTIVE:
            return "ACTIVE";
        case ExecutorState::CANCELING:
            return "CANCELING";
        case ExecutorState::ABORTED:
            return "ABORTED";
        default:
            return "UNKNOWN";
    }
}

std::string CartesianExecutor::lastStopReasonToString(LastStopReason reason) const
{
    switch (reason)
    {
        case LastStopReason::NONE:
            return "NONE";
        case LastStopReason::SUCCEEDED:
            return "SUCCEEDED";
        case LastStopReason::CANCELED:
            return "CANCELED";
        case LastStopReason::ABORTED:
            return "ABORTED";
        default:
            return "UNKNOWN";
    }
}

int64_t CartesianExecutor::getLastCommandAgeMs(const rclcpp::Time& now) const
{
    rclcpp::Time last_command_time;

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_command_time = last_command_time_;
    }

    if (last_command_time.nanoseconds() == 0)
    {
        return -1;
    }

    return static_cast<int64_t>((now - last_command_time).nanoseconds() / 1000000LL);
}

// Register this server into global registry
REGISTER_FR3_ACTION_SERVER(CartesianExecutor, "cartesian_executor")

}  // namespace fr3_husky_controller::servers::fr3