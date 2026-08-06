#include <fr3_husky_controller/servers/fr3/trajectory_executor_action_server.hpp>
#include <fr3_husky_controller/servers/fr3/trajectory_line_math.hpp>

#include <algorithm>
#include <cmath>
#include <cinttypes>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fr3_husky_controller::servers::fr3
{

namespace
{
FR3ModelUpdater& getFR3ModelUpdater(ModelUpdaterBase& model_updater, const std::string& server_name)
{
    auto* p = dynamic_cast<FR3ModelUpdater*>(&model_updater);
    if (!p)
    {
        throw std::runtime_error("[" + server_name + "] requires FR3ModelUpdater");
    }
    return *p;
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

double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

Eigen::Matrix3d rpyDegToRot(double roll_deg, double pitch_deg, double yaw_deg)
{
    constexpr double kPi = 3.14159265358979323846;
    const double r = roll_deg * kPi / 180.0;
    const double p = pitch_deg * kPi / 180.0;
    const double y = yaw_deg * kPi / 180.0;

    const Eigen::AngleAxisd Rx(r, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd Ry(p, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd Rz(y, Eigen::Vector3d::UnitZ());
    return (Rz * Ry * Rx).toRotationMatrix();
}

std::string vecToString(const Eigen::Vector3d& v)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "[" << v.x() << " " << v.y() << " " << v.z() << "]";
    return oss.str();
}

std::string cleanFrame(std::string frame)
{
    while (!frame.empty() && frame.front() == '/')
    {
        frame.erase(frame.begin());
    }
    while (!frame.empty() && frame.back() == '/')
    {
        frame.pop_back();
    }
    return frame;
}

bool sameFrame(const std::string& a, const std::string& b)
{
    const std::string aa = cleanFrame(a);
    const std::string bb = cleanFrame(b);
    return aa.empty() || bb.empty() || aa == bb;
}

}  // namespace

TrajectoryExecutor::TrajectoryExecutor(
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
    last_feedback_time_ = rclcpp::Time(0, 0, clock_type);

    ee_name_default_ = declareOrGetParameter<std::string>(
        node_, name_ + ".ee_name", "right_fr3_link8");
    middle_line_ee_name_ = ee_name_default_;

    line_frame_ = declareOrGetParameter<std::string>(
        node_, name_ + ".line_frame", "base");
    line_frame_ = cleanFrame(line_frame_);
    if (line_frame_.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] line_frame parameter empty; using 'base'",
                    name_.c_str());
        line_frame_ = "base";
    }

    const auto axis = declareOrGetParameter<std::vector<double>>(
        node_, name_ + ".line_axis", std::vector<double>{1.0, 0.0, 0.0});
    if (axis.size() == 3)
    {
        line_axis_ = Eigen::Vector3d(axis[0], axis[1], axis[2]);
    }
    bool initial_axis_valid = true;
    bool axis_flipped = false;
    Eigen::Vector3d canonical_axis = line_axis_;
    if (!canonicalizeLineAxis(line_axis_, &canonical_axis, &axis_flipped))
    {
        initial_axis_valid = false;
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] invalid initial line_axis parameter; middle line marked invalid until updated",
                    name_.c_str());
        line_axis_ = Eigen::Vector3d::UnitX();
    }
    else
    {
        line_axis_ = canonical_axis;
        if (axis_flipped)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[%s] canonicalized initial line_axis to point toward base +x",
                        name_.c_str());
        }
    }

    line_half_length_ = declareOrGetParameter<double>(
        node_, name_ + ".line_half_length", 0.20);

    const auto center_xyz = declareOrGetParameter<std::vector<double>>(
        node_, name_ + ".line_center_xyz", std::vector<double>{0.50, 0.0, 0.20});
    line_center_pose_.setIdentity();
    if (center_xyz.size() == 3 &&
        std::isfinite(center_xyz[0]) && std::isfinite(center_xyz[1]) && std::isfinite(center_xyz[2]))
    {
        line_center_pose_.translation() = Eigen::Vector3d(center_xyz[0], center_xyz[1], center_xyz[2]);
    }

    target_orientation_mode_ = declareOrGetParameter<std::string>(
        node_, name_ + ".target_orientation_mode", "captured");
    base_orientation_rpy_deg_ = declareOrGetParameter<std::vector<double>>(
        node_, name_ + ".base_orientation_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});
    if (base_orientation_rpy_deg_.size() != 3)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] base_orientation_rpy_deg must have size 3; using [0,0,0]",
                    name_.c_str());
        base_orientation_rpy_deg_ = {0.0, 0.0, 0.0};
    }
    line_center_pose_.linear() = makeBaseOrientationFromParams();

    use_velocity_feedforward_ = declareOrGetParameter<bool>(
        node_, name_ + ".use_velocity_feedforward", true);

    hard_v_max_ = declareOrGetParameter<double>(node_, name_ + ".hard_v_max", 1.9);
    hard_a_max_ = declareOrGetParameter<double>(node_, name_ + ".hard_a_max", 5.0);
    hard_j_max_ = declareOrGetParameter<double>(node_, name_ + ".hard_j_max", 30.0);
    hard_min_safety_z_ = declareOrGetParameter<double>(node_, name_ + ".hard_min_safety_z", 0.05);
    min_line_half_length_ = declareOrGetParameter<double>(node_, name_ + ".min_line_half_length", 0.005);
    max_line_half_length_ = declareOrGetParameter<double>(node_, name_ + ".max_line_half_length", 0.40);
    safety_margin_z_ = declareOrGetParameter<double>(node_, name_ + ".safety_margin_z", 0.005);
    reject_line_services_while_active_ = declareOrGetParameter<bool>(
        node_, name_ + ".reject_line_services_while_active", true);
    allow_service_update_safety_min_z_ = declareOrGetParameter<bool>(
        node_, name_ + ".allow_service_update_safety_min_z", false);

    line_half_length_ = std::clamp(line_half_length_, min_line_half_length_, max_line_half_length_);

    safety_min_z_ = declareOrGetParameter<double>(node_, name_ + ".safety_min_z", 0.08);
    if (!std::isfinite(safety_min_z_) || safety_min_z_ < hard_min_safety_z_)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] safety_min_z invalid %.6f; clamping to hard_min_safety_z %.6f",
                    name_.c_str(), safety_min_z_, hard_min_safety_z_);
        safety_min_z_ = hard_min_safety_z_;
    }

    max_tracking_error_pos_ = declareOrGetParameter<double>(node_, name_ + ".max_tracking_error_pos", 0.02);
    max_tracking_error_z_ = declareOrGetParameter<double>(node_, name_ + ".max_tracking_error_z", 0.005);

    default_v_max_slow_ = declareOrGetParameter<double>(node_, name_ + ".default_v_max_slow", 0.10);
    default_a_max_slow_ = declareOrGetParameter<double>(node_, name_ + ".default_a_max_slow", 0.30);
    default_j_max_slow_ = declareOrGetParameter<double>(node_, name_ + ".default_j_max_slow", 10.0);
    default_v_max_fast_ = declareOrGetParameter<double>(node_, name_ + ".default_v_max_fast", 0.40);
    default_a_max_fast_ = declareOrGetParameter<double>(node_, name_ + ".default_a_max_fast", 1.00);
    default_j_max_fast_ = declareOrGetParameter<double>(node_, name_ + ".default_j_max_fast", 30.0);

    require_cartesian_stopped_ = declareOrGetParameter<bool>(node_, name_ + ".require_cartesian_stopped", false);
    require_cartesian_status_service_check_ = declareOrGetParameter<bool>(node_, name_ + ".require_cartesian_status_service_check", false);

    control_mode_ = declareOrGetParameter<int>(node_, name_ + ".control_mode", 0);
    abort_on_repeated_qp_failure_ = declareOrGetParameter<bool>(
        node_, name_ + ".abort_on_repeated_qp_failure", false);
    max_consecutive_qp_failures_ = declareOrGetParameter<int>(
        node_, name_ + ".max_consecutive_qp_failures", 20);
    max_consecutive_qp_failures_ = std::max(1, max_consecutive_qp_failures_);

    const auto workspace_min_vec = declareOrGetParameter<std::vector<double>>(
        node_, name_ + ".workspace_min", std::vector<double>{0.05, -0.4, 0.05});
    const auto workspace_max_vec = declareOrGetParameter<std::vector<double>>(
        node_, name_ + ".workspace_max", std::vector<double>{0.95, 0.4, 0.65});
    if (workspace_min_vec.size() == 3 && workspace_max_vec.size() == 3)
    {
        workspace_min_ = Eigen::Vector3d(workspace_min_vec[0], workspace_min_vec[1], workspace_min_vec[2]);
        workspace_max_ = Eigen::Vector3d(workspace_max_vec[0], workspace_max_vec[1], workspace_max_vec[2]);
    }

    enable_compute_timing_debug_ = declareOrGetParameter<bool>(
        node_, name_ + ".enable_compute_timing_debug", false);
    feedback_period_sec_ = declareOrGetParameter<double>(
        node_, name_ + ".feedback_period_sec", 0.02);
    if (!std::isfinite(feedback_period_sec_) || feedback_period_sec_ <= 0.0)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] invalid feedback_period_sec=%.6f; using default 0.02",
                    name_.c_str(), feedback_period_sec_);
        feedback_period_sec_ = 0.02;
    }

    publish_current_tcp_s_ = declareOrGetParameter<bool>(
        node_, name_ + ".publish_current_tcp_s", true);
    current_tcp_s_publish_rate_hz_ = declareOrGetParameter<double>(
        node_, name_ + ".current_tcp_s_publish_rate_hz", 30.0);
    if (!std::isfinite(current_tcp_s_publish_rate_hz_) || current_tcp_s_publish_rate_hz_ <= 0.0)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] invalid current_tcp_s_publish_rate_hz=%.6f; using default 30.0 Hz",
                    name_.c_str(), current_tcp_s_publish_rate_hz_);
        current_tcp_s_publish_rate_hz_ = 30.0;
    }

    timing_last_report_time_ = std::chrono::steady_clock::now();

    get_status_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/trajectory_executor/get_status",
        std::bind(&TrajectoryExecutor::handleGetStatus, this, std::placeholders::_1, std::placeholders::_2));

    capture_center_srv_ = node_->create_service<fr3_husky_msgs::srv::CaptureLineCenter>(
        "/trajectory_executor/capture_line_center",
        std::bind(&TrajectoryExecutor::handleCaptureLineCenter, this, std::placeholders::_1, std::placeholders::_2));

    set_center_srv_ = node_->create_service<fr3_husky_msgs::srv::SetLineCenter>(
        "/trajectory_executor/set_line_center",
        std::bind(&TrajectoryExecutor::handleSetLineCenter, this, std::placeholders::_1, std::placeholders::_2));

    set_params_srv_ = node_->create_service<fr3_husky_msgs::srv::SetLineParams>(
        "/trajectory_executor/set_line_params",
        std::bind(&TrajectoryExecutor::handleSetLineParams, this, std::placeholders::_1, std::placeholders::_2));

    project_point_srv_ = node_->create_service<fr3_husky_msgs::srv::ProjectPointToLine>(
        "/trajectory_executor/project_point_to_line",
        std::bind(&TrajectoryExecutor::handleProjectPointToLine,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    executed_goto_s_pub_ =
        node_->create_publisher<std_msgs::msg::Float64>(
            "/trajectory_executor/executed_goto_s",
            rclcpp::QoS(10).reliable());

    executed_goto_s_target_base_pub_ =
        node_->create_publisher<geometry_msgs::msg::PointStamped>(
            "/trajectory_executor/executed_goto_s_target_base",
            rclcpp::QoS(10).reliable());

    const auto middle_line_qos =
        rclcpp::QoS(rclcpp::KeepLast(1))
            .reliable()
            .transient_local();

    middle_line_state_pub_ =
        node_->create_publisher<fr3_husky_msgs::msg::MiddleLine>(
            "/trajectory_executor/middle_line_state",
            middle_line_qos);

    if (publish_current_tcp_s_)
    {
        current_tcp_s_pub_ =
            node_->create_publisher<std_msgs::msg::Float64>(
                "/middle_line/current_tcp_s",
                rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

        const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / current_tcp_s_publish_rate_hz_));
        current_tcp_s_timer_ = node_->create_wall_timer(
            period_ns,
            std::bind(&TrajectoryExecutor::publishCurrentTcpS, this));
    }

    refreshTrajectoryLimitParamsFromServer(false);
    trajectory_limit_refresh_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() { refreshTrajectoryLimitParamsFromServer(true); });

    std::string geometry_reason;
    line_valid_ = initial_axis_valid &&
                  validateLineGeometry(line_center_pose_.translation(), line_axis_, line_half_length_, &geometry_reason);
    if (!line_valid_)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] initial line geometry needs attention: %s",
                    name_.c_str(), geometry_reason.c_str());
    }
    else
    {
        publishMiddleLineState();
    }

    RCLCPP_INFO(node_->get_logger(),
                "[%s] TrajectoryExecutor created: ee_default=%s line_frame=%s control_mode=%d orientation_mode=%s base_rpy=[%.1f %.1f %.1f]",
                name_.c_str(), ee_name_default_.c_str(), line_frame_.c_str(), control_mode_, target_orientation_mode_.c_str(),
                base_orientation_rpy_deg_[0], base_orientation_rpy_deg_[1], base_orientation_rpy_deg_[2]);
}

bool TrajectoryExecutor::acceptGoal(const ActionT::Goal& goal)
{
    refreshTrajectoryLimitParamsFromServer(false);

    double default_v_max_slow = 0.0;
    double default_a_max_slow = 0.0;
    double default_j_max_slow = 0.0;
    double default_v_max_fast = 0.0;
    double default_a_max_fast = 0.0;
    double default_j_max_fast = 0.0;
    double hard_v_max = 0.0;
    double hard_a_max = 0.0;
    double hard_j_max = 0.0;
    {
        std::lock_guard<std::mutex> lock(trajectory_limits_mutex_);
        default_v_max_slow = default_v_max_slow_;
        default_a_max_slow = default_a_max_slow_;
        default_j_max_slow = default_j_max_slow_;
        default_v_max_fast = default_v_max_fast_;
        default_a_max_fast = default_a_max_fast_;
        default_j_max_fast = default_j_max_fast_;
        hard_v_max = hard_v_max_;
        hard_a_max = hard_a_max_;
        hard_j_max = hard_j_max_;
    }

    if (!model_updater_.HasEffortCommandInterface())
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject: effort command interface required", name_.c_str());
        return false;
    }

    if (goal.command < ActionT::Goal::CMD_GOTO_CENTER || goal.command > ActionT::Goal::CMD_GOTO_S)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject: invalid command=%u", name_.c_str(), goal.command);
        return false;
    }

    const std::string ee = goal.ee_name.empty() ? ee_name_default_ : goal.ee_name;
    if (!fr3_model_updater_.robot_data_->hasLinkFrame(ee))
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject: ee_name '%s' not in robot model", name_.c_str(), ee.c_str());
        return false;
    }

    const double resolved_v_max =
        (goal.v_max > 0.0) ? goal.v_max :
        ((goal.command == ActionT::Goal::CMD_FAST_SWEEP) ? default_v_max_fast : default_v_max_slow);
    const double resolved_a_max =
        (goal.a_max > 0.0) ? goal.a_max :
        ((goal.command == ActionT::Goal::CMD_FAST_SWEEP) ? default_a_max_fast : default_a_max_slow);
    const double resolved_j_max =
        (goal.j_max > 0.0) ? goal.j_max :
        ((goal.command == ActionT::Goal::CMD_FAST_SWEEP) ? default_j_max_fast : default_j_max_slow);

    if (!std::isfinite(resolved_v_max) || !std::isfinite(resolved_a_max) || !std::isfinite(resolved_j_max) ||
        resolved_v_max <= 0.0 || resolved_a_max <= 0.0 || resolved_j_max <= 0.0)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Reject: invalid limits after defaults v=%.6f a=%.6f j=%.6f",
                    name_.c_str(), resolved_v_max, resolved_a_max, resolved_j_max);
        return false;
    }

    if (resolved_v_max > hard_v_max || resolved_a_max > hard_a_max || resolved_j_max > hard_j_max)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Reject: limits exceed hard bounds v=%.6f a=%.6f j=%.6f hard=[%.6f %.6f %.6f]",
                    name_.c_str(), resolved_v_max, resolved_a_max, resolved_j_max,
                    hard_v_max, hard_a_max, hard_j_max);
        return false;
    }

    if (goal.command == ActionT::Goal::CMD_GOTO_S)
    {
        if (!std::isfinite(goal.target_s))
        {
            RCLCPP_WARN(node_->get_logger(), "[%s] Reject: target_s is non-finite", name_.c_str());
            return false;
        }

        std::lock_guard<std::mutex> lock(line_mutex_);
        if (std::abs(goal.target_s) > line_half_length_ + 1e-6)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] Reject: target_s=%.6f outside line bounds [-%.6f, %.6f]",
                        name_.c_str(), goal.target_s, line_half_length_, line_half_length_);
            return false;
        }
    }

    std::string reason;
    if (!motion_gate_->tryAcquire(MotionGate::Owner::TRAJECTORY_EXECUTOR, name_, &reason))
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject: %s", name_.c_str(), reason.c_str());
        return false;
    }

    if (!checkCartesianExecutorSafe(&reason))
    {
        motion_gate_->release(MotionGate::Owner::TRAJECTORY_EXECUTOR);
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject: %s", name_.c_str(), reason.c_str());
        return false;
    }

    gate_acquired_ = true;
    return true;
}

void TrajectoryExecutor::onGoalAccepted(const ActionT::Goal& goal)
{
    goal_ = goal;
    start_failed_ = false;
    start_failure_reason_.clear();
    segment_index_ = 0;
    segment_time_ = 0.0;
    overall_progress_ = 0.0;
    last_error_message_.clear();
    last_phase_.clear();
    last_s_des_ = 0.0;
    last_sdot_des_ = 0.0;
    consecutive_qp_failures_ = 0;

    if (goal_.ee_name.empty())
    {
        goal_.ee_name = ee_name_default_;
    }

    double default_v_max_slow = 0.0;
    double default_a_max_slow = 0.0;
    double default_j_max_slow = 0.0;
    double default_v_max_fast = 0.0;
    double default_a_max_fast = 0.0;
    double default_j_max_fast = 0.0;
    {
        std::lock_guard<std::mutex> lock(trajectory_limits_mutex_);
        default_v_max_slow = default_v_max_slow_;
        default_a_max_slow = default_a_max_slow_;
        default_j_max_slow = default_j_max_slow_;
        default_v_max_fast = default_v_max_fast_;
        default_a_max_fast = default_a_max_fast_;
        default_j_max_fast = default_j_max_fast_;
    }

    if (goal_.v_max <= 0.0)
    {
        goal_.v_max = (goal_.command == ActionT::Goal::CMD_FAST_SWEEP) ? default_v_max_fast : default_v_max_slow;
    }
    if (goal_.a_max <= 0.0)
    {
        goal_.a_max = (goal_.command == ActionT::Goal::CMD_FAST_SWEEP) ? default_a_max_fast : default_a_max_slow;
    }
    if (goal_.j_max <= 0.0)
    {
        goal_.j_max = (goal_.command == ActionT::Goal::CMD_FAST_SWEEP) ? default_j_max_fast : default_j_max_slow;
    }
    if (goal_.repetitions == 0)
    {
        goal_.repetitions = 1;
    }

    control_ee_name_ = goal_.ee_name;

    std::lock_guard<std::mutex> lock(line_mutex_);
    const Eigen::Vector3d start = line_center_pose_.translation() - line_half_length_ * line_axis_;
    const Eigen::Vector3d end = line_center_pose_.translation() + line_half_length_ * line_axis_;

    RCLCPP_INFO(node_->get_logger(),
                "[%s] goal accepted cmd=%s profile=%s ee=%s target_s=%.4f v=%.3f a=%.3f j=%.3f safety_min_z=%.3f center=%s start=%s end=%s",
                name_.c_str(), commandToString(goal_.command).c_str(), goal_.profile_name.c_str(),
                goal_.ee_name.c_str(), goal_.target_s, goal_.v_max, goal_.a_max, goal_.j_max,
                safety_min_z_, vecToString(line_center_pose_.translation()).c_str(),
                vecToString(start).c_str(), vecToString(end).c_str());
}

void TrajectoryExecutor::onStart()
{
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        executor_state_ = ExecutorState::ACTIVE;
        stop_in_progress_ = false;
        last_start_time_ = node_->now();
        last_stop_reason_ = LastStopReason::NONE;
    }

    first_command_written_ = false;
    last_feedback_time_ = node_->now();
    timing_call_count_ = 0;
    timing_overrun_800us_ = 0;
    timing_overrun_1000us_ = 0;
    timing_max_compute_us_ = 0.0;
    timing_last_report_time_ = std::chrono::steady_clock::now();
    resetMotionStats();

    if (start_failed_)
    {
        last_error_message_ = start_failure_reason_;
        RCLCPP_ERROR(node_->get_logger(), "[%s] start failed: %s", name_.c_str(), last_error_message_.c_str());
        return;
    }

    const auto current_pose = fr3_model_updater_.robot_data_->getPose(control_ee_name_);

    {
        std::lock_guard<std::mutex> lock(line_mutex_);
        if (target_orientation_mode_ == "captured" && !have_captured_orientation_)
        {
            line_center_pose_.linear() = current_pose.linear();
            have_captured_orientation_ = true;
        }
    }

    std::string reason;
    if (!buildSegments(&reason))
    {
        last_error_message_ = reason;
        RCLCPP_ERROR(node_->get_logger(), "[%s] start failed: %s", name_.c_str(), reason.c_str());
        return;
    }

    ee_data_.clear();
    ee_data_[control_ee_name_] = drc::TaskSpaceData::Zero();
    ee_data_[control_ee_name_].x = current_pose;
    ee_data_[control_ee_name_].xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    ee_data_[control_ee_name_].xddot.setZero();
    ee_data_[control_ee_name_].setInit();
    ee_data_[control_ee_name_].setDesired();

    target_pose_ = current_pose;
    target_rotation_ = resolveActiveOrientation(current_pose);
    target_pose_.linear() = target_rotation_;

    if (!jerk_unused_logged_)
    {
        jerk_unused_logged_ = true;
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] j_max accepted but not used yet; profile is trapezoidal/triangular.",
                    name_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(),
                "[%s] state ACTIVE mode=%d ee=%s segments=%zu orientation_mode=%s",
                name_.c_str(), control_mode_, control_ee_name_.c_str(), segments_.size(),
                target_orientation_mode_.c_str());
}

TrajectoryExecutor::ComputeResult TrajectoryExecutor::compute(
    const rclcpp::Time& time,
    const rclcpp::Duration& period)
{
    if (start_failed_)
    {
        return ComputeResult::ABORTED;
    }

    if (!last_error_message_.empty())
    {
        return ComputeResult::ABORTED;
    }

    if (segment_index_ >= segments_.size())
    {
        return ComputeResult::SUCCEEDED;
    }

    const auto timing_start = std::chrono::steady_clock::now();

    auto& seg = segments_[segment_index_];
    const double dt = std::max(0.0, period.seconds());
    segment_time_ += dt;

    Eigen::Vector3d desired_pos = seg.p1;
    Eigen::Vector3d desired_vel = Eigen::Vector3d::Zero();
    double s_des = seg.length;
    double sdot_des = 0.0;
    double seg_progress = 1.0;

    if (seg.hold_only)
    {
        desired_pos = seg.p0;
        desired_vel.setZero();
        s_des = 0.0;
        sdot_des = 0.0;
        seg_progress = (seg.t_total > 1e-9) ? std::clamp(segment_time_ / seg.t_total, 0.0, 1.0) : 1.0;
    }
    else if (seg.length > 1e-9)
    {
        const double tau = std::clamp(segment_time_, 0.0, seg.t_total);
        const double d_acc = 0.5 * seg.a_max * seg.t_acc * seg.t_acc;

        if (tau < seg.t_acc)
        {
            s_des = 0.5 * seg.a_max * tau * tau;
            sdot_des = seg.a_max * tau;
        }
        else if (tau < (seg.t_acc + seg.t_flat))
        {
            s_des = d_acc + seg.v_max * (tau - seg.t_acc);
            sdot_des = seg.v_max;
        }
        else
        {
            const double td = tau - seg.t_acc - seg.t_flat;
            s_des = d_acc + seg.v_max * seg.t_flat + seg.v_max * td - 0.5 * seg.a_max * td * td;
            sdot_des = std::max(0.0, seg.v_max - seg.a_max * td);
        }

        s_des = std::clamp(s_des, 0.0, seg.length);
        desired_pos = seg.p0 + seg.dir * s_des;
        desired_vel = seg.dir * sdot_des;
        seg_progress = (seg.t_total > 1e-9) ? std::clamp(tau / seg.t_total, 0.0, 1.0) : 1.0;
    }

    last_phase_ = seg.phase;
    last_s_des_ = s_des;
    last_sdot_des_ = sdot_des;

    const bool include_motion_stats = !seg.hold_only && seg.length > 1e-9;

    if (!computeCartesianCommand(desired_pos, desired_vel, dt, include_motion_stats))
    {
        return ComputeResult::ABORTED;
    }

    overall_progress_ = (static_cast<double>(segment_index_) + seg_progress) /
                        std::max(1.0, static_cast<double>(segments_.size()));

    if ((time - last_feedback_time_).seconds() >= feedback_period_sec_)
    {
        auto fb = std::make_shared<ActionT::Feedback>();
        fb->phase = seg.phase;
        fb->progress = std::clamp(overall_progress_, 0.0, 1.0);
        fb->s_des = s_des;
        fb->sdot_des = sdot_des;
        fb->tracking_error_pos = last_tracking_error_pos_;
        fb->tracking_error_z = last_tracking_error_z_;
        publishFeedback(fb);
        last_feedback_time_ = time;
    }

    if (segment_time_ >= seg.t_total)
    {
        segment_index_++;
        segment_time_ = 0.0;
    }

    updateComputeTimingDebug(timing_start);
    (void)time;
    return ComputeResult::RUNNING;
}

bool TrajectoryExecutor::computeCartesianCommand(
    const Eigen::Vector3d& desired_pos,
    const Eigen::Vector3d& desired_vel,
    double dt,
    bool include_motion_stats)
{
    auto it = ee_data_.find(control_ee_name_);
    if (it == ee_data_.end())
    {
        last_error_message_ = "TaskSpaceData missing for ee=" + control_ee_name_;
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), last_error_message_.c_str());
        return false;
    }

    auto& task = it->second;
    task.x = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    task.xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    task.xddot.setZero();

    const Eigen::Vector3d current_pos = task.x.translation();
    const Eigen::Vector3d current_vel = task.xdot.head<3>();
    if (std::isfinite(dt) && dt > 0.0)
    {
        updateMotionStats(current_pos, current_vel, dt, include_motion_stats);
    }

    const Eigen::Vector3d err = current_pos - desired_pos;
    const double tracking_error_pos = err.norm();
    const double tracking_error_z = std::abs(err.z());

    last_current_pos_ = current_pos;
    last_desired_pos_ = desired_pos;
    last_tracking_error_pos_ = tracking_error_pos;
    last_tracking_error_z_ = tracking_error_z;

    if (current_pos.z() < safety_min_z_)
    {
        last_error_message_ = makeAbortDetail(
            "current z below safety_min_z", last_phase_, current_pos, desired_pos,
            tracking_error_pos, tracking_error_z);
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), last_error_message_.c_str());
        return false;
    }

    if (desired_pos.z() < (safety_min_z_ + safety_margin_z_))
    {
        last_error_message_ = makeAbortDetail(
            "desired z below safety_min_z + safety_margin_z", last_phase_, current_pos, desired_pos,
            tracking_error_pos, tracking_error_z);
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), last_error_message_.c_str());
        return false;
    }

    if (tracking_error_pos > (2.0 * max_tracking_error_pos_) ||
        tracking_error_z > (2.0 * max_tracking_error_z_))
    {
        last_error_message_ = makeAbortDetail(
            "gross tracking error", last_phase_, current_pos, desired_pos,
            tracking_error_pos, tracking_error_z);
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), last_error_message_.c_str());
        return false;
    }

    target_pose_.translation() = desired_pos;
    target_pose_.linear() = resolveActiveOrientation(task.x);
    target_rotation_ = target_pose_.linear();

    Eigen::Vector3d p = target_pose_.translation();
    p.x() = clamp(p.x(), workspace_min_.x(), workspace_max_.x());
    p.y() = clamp(p.y(), workspace_min_.y(), workspace_max_.y());
    p.z() = clamp(p.z(), workspace_min_.z(), workspace_max_.z());
    target_pose_.translation() = p;

    Eigen::Vector6d target_vel = Eigen::Vector6d::Zero();
    if (use_velocity_feedforward_)
    {
        // This matches CartesianExecutor's convention: head<3> = linear, tail<3> = angular.
        target_vel.head<3>() = desired_vel;
    }

    task.x_desired = target_pose_;
    task.xdot_desired = target_vel;

    const Eigen::Matrix3d R_err = task.x.linear().transpose() * target_pose_.linear();
    const Eigen::AngleAxisd aa(R_err);
    const double ori_err_deg = aa.angle() * 180.0 / 3.14159265358979323846;

    if (rcutils_logging_logger_is_enabled_for(node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
    {
        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            200,
            "[%s debug] mode=%d phase=%s cur=%s des=%s des-cur=%s ori_err_deg=%.3f v_ff=%s",
            name_.c_str(), control_mode_, last_phase_.c_str(),
            vecToString(current_pos).c_str(), vecToString(target_pose_.translation()).c_str(),
            vecToString(target_pose_.translation() - current_pos).c_str(), ori_err_deg,
            use_velocity_feedforward_ ? "true" : "false");
    }

    bool is_qp_solved = true;
    std::string time_verbose;

    switch (control_mode_)
    {
        case 0: // CLIK
            fr3_model_updater_.robot_controller_->CLIKStep(
                ee_data_,
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
                ee_data_,
                fr3_model_updater_.torque_desired_total_);
            break;

        case 2: // QPIK
            is_qp_solved = fr3_model_updater_.robot_controller_->QPIKStep(
                ee_data_,
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
                ee_data_,
                fr3_model_updater_.torque_desired_total_,
                time_verbose);

            if (!is_qp_solved)
            {
                fr3_model_updater_.torque_desired_total_ =
                    fr3_model_updater_.robot_data_->getGravity();
            }
            break;

        default:
            is_qp_solved = false;
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
                last_error_message_ = "repeated QP failures";
                RCLCPP_ERROR(node_->get_logger(),
                             "[%s] aborting: repeated QP failures (%d >= %d)",
                             name_.c_str(), consecutive_qp_failures_, max_consecutive_qp_failures_);
                return false;
            }
        }
    }
    else
    {
        consecutive_qp_failures_ = 0;
    }

    if (!is_qp_solved)
    {
        last_error_message_ = makeAbortDetail(
            "controller step failed", last_phase_, current_pos, desired_pos,
            tracking_error_pos, tracking_error_z);
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), last_error_message_.c_str());
        return false;
    }

    if (tracking_error_pos > max_tracking_error_pos_)
    {
        last_error_message_ = makeAbortDetail(
            "tracking_error_pos exceeded", last_phase_, current_pos, desired_pos,
            tracking_error_pos, tracking_error_z);
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), last_error_message_.c_str());
        return false;
    }
    if (tracking_error_z > max_tracking_error_z_)
    {
        last_error_message_ = makeAbortDetail(
            "tracking_error_z exceeded", last_phase_, current_pos, desired_pos,
            tracking_error_pos, tracking_error_z);
        RCLCPP_ERROR(node_->get_logger(), "[%s] %s", name_.c_str(), last_error_message_.c_str());
        return false;
    }

    // Deliberately identical to CartesianExecutor's command convention.
    if (!first_command_written_)
    {
        if (goal_.command == ActionT::Goal::CMD_GOTO_S)
        {
            const Eigen::Vector3d executed_target =
                active_center_ + active_axis_ * goal_.target_s;

            geometry_msgs::msg::PointStamped target_msg;
            target_msg.header.stamp = node_->now();
            target_msg.header.frame_id = line_frame_;
            target_msg.point.x = executed_target.x();
            target_msg.point.y = executed_target.y();
            target_msg.point.z = executed_target.z();
            executed_goto_s_target_base_pub_->publish(target_msg);

            std_msgs::msg::Float64 msg;
            msg.data = goal_.target_s;
            executed_goto_s_pub_->publish(msg);
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] first command write t=%.6f command=%s target_s=%.6f "
            "target=%s frame=%s phase=%s s_des=%.6f sdot_des=%.6f",
            name_.c_str(),
            node_->now().seconds(),
            commandToString(goal_.command).c_str(),
            goal_.target_s,
            vecToString(active_center_ + active_axis_ * goal_.target_s).c_str(),
            line_frame_.c_str(),
            last_phase_.c_str(),
            last_s_des_,
            last_sdot_des_);
        first_command_written_ = true;
    }

    fr3_model_updater_.writeCommand(
        fr3_model_updater_.torque_desired_total_ - fr3_model_updater_.g_total_);

    return true;
}

void TrajectoryExecutor::onStop(StopReason reason)
{
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        executor_state_ = ExecutorState::CANCELING;
        stop_in_progress_ = true;
    }

    model_updater_.haltCommands();

    if (!control_ee_name_.empty() && fr3_model_updater_.robot_data_->hasLinkFrame(control_ee_name_))
    {
        const auto current_pose = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
        target_pose_ = current_pose;
        target_rotation_ = current_pose.linear();
    }

    const auto stop_time = node_->now();
    LastStopReason final_stop_reason = LastStopReason::NONE;
    if (reason == StopReason::SUCCEEDED)
    {
        final_stop_reason = LastStopReason::SUCCEEDED;
    }
    else if (reason == StopReason::CANCELED)
    {
        final_stop_reason = LastStopReason::CANCELED;
    }
    else if (reason == StopReason::ABORTED)
    {
        final_stop_reason = LastStopReason::ABORTED;
    }

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        last_stop_time_ = stop_time;
        stop_in_progress_ = false;
        executor_state_ = ExecutorState::STOPPED;
        last_stop_reason_ = final_stop_reason;
    }

    if (gate_acquired_)
    {
        motion_gate_->release(MotionGate::Owner::TRAJECTORY_EXECUTOR);
        gate_acquired_ = false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[%s] stopped reason=%s | %s",
                name_.c_str(),
                lastStopReasonToString(final_stop_reason).c_str(),
                formatMotionStatsSummary().c_str());
}

TrajectoryExecutor::ResultPtr TrajectoryExecutor::makeResult(StopReason reason)
{
    auto result = std::make_shared<ActionT::Result>();
    const std::string stats_summary = formatMotionStatsSummary();
    const bool has_motion_stats = motion_stats_.moving_time > 1e-9;
    if (reason == StopReason::SUCCEEDED)
    {
        result->success = true;
        result->message = "trajectory completed | " + stats_summary;
        result->final_state = 0;
    }
    else if (reason == StopReason::CANCELED)
    {
        result->success = false;
        result->message = "trajectory canceled";
        if (has_motion_stats)
        {
            result->message += " | " + stats_summary;
        }
        result->final_state = 2;
    }
    else
    {
        result->success = false;
        result->message = last_error_message_.empty() ? "trajectory aborted" : "trajectory aborted: " + last_error_message_;
        if (has_motion_stats)
        {
            result->message += " | " + stats_summary;
        }
        result->final_state = 1;
    }
    return result;
}

bool TrajectoryExecutor::checkCartesianExecutorSafe(std::string* reason)
{
    (void)reason;

    if (require_cartesian_status_service_check_)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] ignoring require_cartesian_status_service_check=true; MotionGate is primary mutual exclusion.",
                    name_.c_str());
    }

    if (require_cartesian_stopped_)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] cartesian stopped check satisfied by MotionGate ownership.",
                    name_.c_str());
    }

    return true;
}

bool TrajectoryExecutor::buildSegments(std::string* reason)
{
    double default_v_max_slow = 0.0;
    double default_a_max_slow = 0.0;
    {
        std::lock_guard<std::mutex> lock(trajectory_limits_mutex_);
        default_v_max_slow = default_v_max_slow_;
        default_a_max_slow = default_a_max_slow_;
    }

    const auto current_pose = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    const Eigen::Vector3d current = current_pose.translation();

    {
        std::lock_guard<std::mutex> lock(line_mutex_);
        if (!line_valid_)
        {
            if (reason) { *reason = "middle line is not valid"; }
            return false;
        }
        active_center_ = line_center_pose_.translation();
        active_axis_ = line_axis_;
        active_half_length_ = std::clamp(line_half_length_, min_line_half_length_, max_line_half_length_);
        active_orientation_ = resolveActiveOrientation(current_pose);
    }

    std::string geom_reason;
    if (!validateLineGeometry(active_center_, active_axis_, active_half_length_, &geom_reason))
    {
        if (reason) { *reason = geom_reason; }
        return false;
    }
    active_axis_.normalize();

    const Eigen::Vector3d center = active_center_;
    const Eigen::Vector3d start = linePoint(-active_half_length_);
    const Eigen::Vector3d end = linePoint(+active_half_length_);

    segments_.clear();

    switch (goal_.command)
    {
        case ActionT::Goal::CMD_GOTO_CENTER:
            segments_.push_back(makeSegment("goto_center", current, center, goal_.v_max, goal_.a_max));
            if (goal_.hold_after_sec > 0.0) { segments_.push_back(makeHold("hold_center", center, goal_.hold_after_sec)); }
            break;

        case ActionT::Goal::CMD_GOTO_START:
            segments_.push_back(makeSegment("goto_start", current, start, goal_.v_max, goal_.a_max));
            if (goal_.hold_after_sec > 0.0) { segments_.push_back(makeHold("hold_start", start, goal_.hold_after_sec)); }
            break;

        case ActionT::Goal::CMD_GOTO_END:
            segments_.push_back(makeSegment("goto_end", current, end, goal_.v_max, goal_.a_max));
            if (goal_.hold_after_sec > 0.0) { segments_.push_back(makeHold("hold_end", end, goal_.hold_after_sec)); }
            break;

        case ActionT::Goal::CMD_GOTO_S:
        {
            if (!std::isfinite(goal_.target_s) ||
                std::abs(goal_.target_s) > active_half_length_ + 1e-6)
            {
                if (reason)
                {
                    std::ostringstream oss;
                    oss << "target_s=" << goal_.target_s
                        << " outside active line half length=" << active_half_length_;
                    *reason = oss.str();
                }
                return false;
            }

            const Eigen::Vector3d target = linePoint(goal_.target_s);

            RCLCPP_WARN(
                node_->get_logger(),
                "[%s GOTO_S DEBUG] current=%s target=%s delta=%s "
                "center=%s axis=%s target_s=%.6f",
                name_.c_str(),
                vecToString(current).c_str(),
                vecToString(target).c_str(),
                vecToString(target - current).c_str(),
                vecToString(active_center_).c_str(),
                vecToString(active_axis_).c_str(),
                goal_.target_s);

            segments_.push_back(
                makeSegment("goto_s", current, target, goal_.v_max, goal_.a_max));

            if (goal_.hold_after_sec > 0.0)
            {
                segments_.push_back(makeHold("hold_s", target, goal_.hold_after_sec));
            }
            break;
        }

        case ActionT::Goal::CMD_PHASE_TEST:
            segments_.push_back(makeSegment("phase_center", current, center, goal_.v_max, goal_.a_max));
            segments_.push_back(makeSegment("phase_start", center, start, goal_.v_max, goal_.a_max));
            segments_.push_back(makeSegment("phase_center_return_1", start, center, goal_.v_max, goal_.a_max));
            segments_.push_back(makeSegment("phase_end", center, end, goal_.v_max, goal_.a_max));
            segments_.push_back(makeSegment("phase_center_return_2", end, center, goal_.v_max, goal_.a_max));
            if (goal_.hold_after_sec > 0.0) { segments_.push_back(makeHold("hold_center", center, goal_.hold_after_sec)); }
            break;

        case ActionT::Goal::CMD_FAST_SWEEP:
        {
            segments_.push_back(makeSegment("fast_setup_start", current, start, default_v_max_slow, default_a_max_slow));
            if (goal_.hold_before_sec > 0.0) { segments_.push_back(makeHold("hold_before", start, goal_.hold_before_sec)); }

            Eigen::Vector3d from = start;
            Eigen::Vector3d to = end;
            for (uint32_t i = 0; i < goal_.repetitions; ++i)
            {
                std::ostringstream phase;
                phase << "fast_sweep_" << (i + 1);
                segments_.push_back(makeSegment(phase.str(), from, to, goal_.v_max, goal_.a_max));
                std::swap(from, to);
            }

            if (goal_.hold_after_sec > 0.0)
            {
                // Hold at the actual final side after odd/even repetition count.
                const Eigen::Vector3d final_p = (goal_.repetitions % 2 == 1) ? end : start;
                segments_.push_back(makeHold("hold_after", final_p, goal_.hold_after_sec));
            }
            break;
        }

        default:
            if (reason) { *reason = "unsupported command"; }
            return false;
    }

    if (segments_.empty())
    {
        if (reason) { *reason = "no segments generated"; }
        return false;
    }

    return true;
}

void TrajectoryExecutor::refreshTrajectoryLimitParamsFromServer(bool log_changes)
{
    double prev_default_v_max_slow = 0.0;
    double prev_default_a_max_slow = 0.0;
    double prev_default_j_max_slow = 0.0;
    double prev_default_v_max_fast = 0.0;
    double prev_default_a_max_fast = 0.0;
    double prev_default_j_max_fast = 0.0;
    double prev_hard_v_max = 0.0;
    double prev_hard_a_max = 0.0;
    double prev_hard_j_max = 0.0;

    {
        std::lock_guard<std::mutex> lock(trajectory_limits_mutex_);
        prev_default_v_max_slow = default_v_max_slow_;
        prev_default_a_max_slow = default_a_max_slow_;
        prev_default_j_max_slow = default_j_max_slow_;
        prev_default_v_max_fast = default_v_max_fast_;
        prev_default_a_max_fast = default_a_max_fast_;
        prev_default_j_max_fast = default_j_max_fast_;
        prev_hard_v_max = hard_v_max_;
        prev_hard_a_max = hard_a_max_;
        prev_hard_j_max = hard_j_max_;
    }

    double next_default_v_max_slow = prev_default_v_max_slow;
    double next_default_a_max_slow = prev_default_a_max_slow;
    double next_default_j_max_slow = prev_default_j_max_slow;
    double next_default_v_max_fast = prev_default_v_max_fast;
    double next_default_a_max_fast = prev_default_a_max_fast;
    double next_default_j_max_fast = prev_default_j_max_fast;
    double next_hard_v_max = prev_hard_v_max;
    double next_hard_a_max = prev_hard_a_max;
    double next_hard_j_max = prev_hard_j_max;

    (void)node_->get_parameter(name_ + ".hard_v_max", next_hard_v_max);
    (void)node_->get_parameter(name_ + ".hard_a_max", next_hard_a_max);
    (void)node_->get_parameter(name_ + ".hard_j_max", next_hard_j_max);
    (void)node_->get_parameter(name_ + ".default_v_max_slow", next_default_v_max_slow);
    (void)node_->get_parameter(name_ + ".default_a_max_slow", next_default_a_max_slow);
    (void)node_->get_parameter(name_ + ".default_j_max_slow", next_default_j_max_slow);
    (void)node_->get_parameter(name_ + ".default_v_max_fast", next_default_v_max_fast);
    (void)node_->get_parameter(name_ + ".default_a_max_fast", next_default_a_max_fast);
    (void)node_->get_parameter(name_ + ".default_j_max_fast", next_default_j_max_fast);

    auto sanitize_positive = [&](double value, double fallback, const char* param_name) -> double
    {
        if (std::isfinite(value) && value > 0.0)
        {
            return value;
        }
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] invalid %s=%.6f; keeping previous %.6f",
            name_.c_str(),
            param_name,
            value,
            fallback);
        return fallback;
    };

    next_hard_v_max = sanitize_positive(next_hard_v_max, prev_hard_v_max, "hard_v_max");
    next_hard_a_max = sanitize_positive(next_hard_a_max, prev_hard_a_max, "hard_a_max");
    next_hard_j_max = sanitize_positive(next_hard_j_max, prev_hard_j_max, "hard_j_max");

    next_default_v_max_slow = sanitize_positive(next_default_v_max_slow, prev_default_v_max_slow, "default_v_max_slow");
    next_default_a_max_slow = sanitize_positive(next_default_a_max_slow, prev_default_a_max_slow, "default_a_max_slow");
    next_default_j_max_slow = sanitize_positive(next_default_j_max_slow, prev_default_j_max_slow, "default_j_max_slow");
    next_default_v_max_fast = sanitize_positive(next_default_v_max_fast, prev_default_v_max_fast, "default_v_max_fast");
    next_default_a_max_fast = sanitize_positive(next_default_a_max_fast, prev_default_a_max_fast, "default_a_max_fast");
    next_default_j_max_fast = sanitize_positive(next_default_j_max_fast, prev_default_j_max_fast, "default_j_max_fast");

    auto clamp_to_hard = [&](double value, double hard_limit, const char* param_name, const char* hard_name) -> double
    {
        if (value <= hard_limit)
        {
            return value;
        }
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] clamping %s=%.6f to %s=%.6f",
            name_.c_str(),
            param_name,
            value,
            hard_name,
            hard_limit);
        return hard_limit;
    };

    next_default_v_max_slow = clamp_to_hard(next_default_v_max_slow, next_hard_v_max, "default_v_max_slow", "hard_v_max");
    next_default_a_max_slow = clamp_to_hard(next_default_a_max_slow, next_hard_a_max, "default_a_max_slow", "hard_a_max");
    next_default_j_max_slow = clamp_to_hard(next_default_j_max_slow, next_hard_j_max, "default_j_max_slow", "hard_j_max");
    next_default_v_max_fast = clamp_to_hard(next_default_v_max_fast, next_hard_v_max, "default_v_max_fast", "hard_v_max");
    next_default_a_max_fast = clamp_to_hard(next_default_a_max_fast, next_hard_a_max, "default_a_max_fast", "hard_a_max");
    next_default_j_max_fast = clamp_to_hard(next_default_j_max_fast, next_hard_j_max, "default_j_max_fast", "hard_j_max");

    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(trajectory_limits_mutex_);
        changed =
            default_v_max_slow_ != next_default_v_max_slow ||
            default_a_max_slow_ != next_default_a_max_slow ||
            default_j_max_slow_ != next_default_j_max_slow ||
            default_v_max_fast_ != next_default_v_max_fast ||
            default_a_max_fast_ != next_default_a_max_fast ||
            default_j_max_fast_ != next_default_j_max_fast ||
            hard_v_max_ != next_hard_v_max ||
            hard_a_max_ != next_hard_a_max ||
            hard_j_max_ != next_hard_j_max;

        default_v_max_slow_ = next_default_v_max_slow;
        default_a_max_slow_ = next_default_a_max_slow;
        default_j_max_slow_ = next_default_j_max_slow;
        default_v_max_fast_ = next_default_v_max_fast;
        default_a_max_fast_ = next_default_a_max_fast;
        default_j_max_fast_ = next_default_j_max_fast;
        hard_v_max_ = next_hard_v_max;
        hard_a_max_ = next_hard_a_max;
        hard_j_max_ = next_hard_j_max;
    }

    if (log_changes && changed)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] refreshed limits slow[v=%.3f a=%.3f j=%.3f] fast[v=%.3f a=%.3f j=%.3f] hard[v=%.3f a=%.3f j=%.3f]",
            name_.c_str(),
            next_default_v_max_slow,
            next_default_a_max_slow,
            next_default_j_max_slow,
            next_default_v_max_fast,
            next_default_a_max_fast,
            next_default_j_max_fast,
            next_hard_v_max,
            next_hard_a_max,
            next_hard_j_max);
    }
}

TrajectoryExecutor::TrajSegment TrajectoryExecutor::makeSegment(
    const std::string& phase,
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    double v_max,
    double a_max) const
{
    TrajSegment seg;
    seg.phase = phase;
    seg.p0 = p0;
    seg.p1 = p1;
    const Eigen::Vector3d delta = p1 - p0;
    seg.length = delta.norm();
    seg.v_max = std::max(1e-4, v_max);
    seg.a_max = std::max(1e-4, a_max);

    if (seg.length > 1e-9)
    {
        seg.dir = delta / seg.length;
        seg.t_acc = seg.v_max / seg.a_max;
        const double d_acc = 0.5 * seg.a_max * seg.t_acc * seg.t_acc;
        if (2.0 * d_acc >= seg.length)
        {
            seg.t_acc = std::sqrt(seg.length / seg.a_max);
            seg.t_flat = 0.0;
            seg.v_max = seg.a_max * seg.t_acc;
        }
        else
        {
            seg.t_flat = (seg.length - 2.0 * d_acc) / seg.v_max;
        }
        seg.t_total = 2.0 * seg.t_acc + seg.t_flat;
    }
    else
    {
        seg.dir = Eigen::Vector3d::UnitX();
        seg.t_total = 0.0;
    }

    return seg;
}

TrajectoryExecutor::TrajSegment TrajectoryExecutor::makeHold(
    const std::string& phase,
    const Eigen::Vector3d& p,
    double hold_sec) const
{
    TrajSegment seg;
    seg.phase = phase;
    seg.p0 = p;
    seg.p1 = p;
    seg.hold_only = true;
    seg.t_total = std::max(0.0, hold_sec);
    return seg;
}

Eigen::Matrix3d TrajectoryExecutor::makeBaseOrientationFromParams() const
{
    if (base_orientation_rpy_deg_.size() != 3)
    {
        return Eigen::Matrix3d::Identity();
    }
    return rpyDegToRot(base_orientation_rpy_deg_[0], base_orientation_rpy_deg_[1], base_orientation_rpy_deg_[2]);
}

Eigen::Matrix3d TrajectoryExecutor::resolveActiveOrientation(const Eigen::Affine3d& current_pose) const
{
    if (target_orientation_mode_ == "base_rpy")
    {
        return makeBaseOrientationFromParams();
    }
    if (target_orientation_mode_ == "captured")
    {
        return line_center_pose_.linear();
    }
    if (target_orientation_mode_ == "current")
    {
        return current_pose.linear();
    }

    RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "[%s] invalid target_orientation_mode='%s'; using base_rpy",
        name_.c_str(), target_orientation_mode_.c_str());
    return makeBaseOrientationFromParams();
}

Eigen::Vector3d TrajectoryExecutor::linePoint(double s) const
{
    return active_center_ + active_axis_ * s;
}

double TrajectoryExecutor::commandTargetS(uint8_t command) const
{
    if (command == ActionT::Goal::CMD_GOTO_START) { return -active_half_length_; }
    if (command == ActionT::Goal::CMD_GOTO_END) { return active_half_length_; }
    if (command == ActionT::Goal::CMD_GOTO_CENTER) { return 0.0; }
    if (command == ActionT::Goal::CMD_GOTO_S) { return goal_.target_s; }
    return 0.0;
}

void TrajectoryExecutor::resetMotionStats()
{
    motion_stats_ = MotionStats{};
}

void TrajectoryExecutor::updateMotionStats(
    const Eigen::Vector3d& current_pos,
    const Eigen::Vector3d& current_vel,
    double dt,
    bool include_sample)
{
    if (!finite3(current_pos) || !finite3(current_vel) || !std::isfinite(dt) || dt <= 0.0)
    {
        return;
    }

    if (!motion_stats_.initialized)
    {
        motion_stats_.last_pos = current_pos;
        motion_stats_.last_vel = current_vel;
        motion_stats_.initialized = true;
        return;
    }

    if (!include_sample)
    {
        motion_stats_.last_pos = current_pos;
        motion_stats_.last_vel = current_vel;
        return;
    }

    const double speed = current_vel.norm();
    const double ds = (current_pos - motion_stats_.last_pos).norm();
    const Eigen::Vector3d acc_vec = (current_vel - motion_stats_.last_vel) / dt;
    const double acc = acc_vec.norm();

    motion_stats_.moving_time += dt;
    motion_stats_.path_length += ds;
    motion_stats_.speed_integral += speed * dt;
    motion_stats_.acc_integral += acc * dt;
    motion_stats_.peak_speed = std::max(motion_stats_.peak_speed, speed);
    motion_stats_.peak_acc = std::max(motion_stats_.peak_acc, acc);
    motion_stats_.speed_samples++;
    motion_stats_.acc_samples++;

    motion_stats_.last_pos = current_pos;
    motion_stats_.last_vel = current_vel;
}

std::string TrajectoryExecutor::formatMotionStatsSummary() const
{
    if (motion_stats_.moving_time <= 1e-9)
    {
        return "motion_stats: no moving samples";
    }

    const double avg_speed = motion_stats_.speed_integral / motion_stats_.moving_time;
    const double avg_acc = motion_stats_.acc_integral / motion_stats_.moving_time;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "motion_stats: moving_time=" << motion_stats_.moving_time << "s"
        << " path=" << motion_stats_.path_length << "m"
        << " avg_speed=" << avg_speed << "m/s"
        << " peak_speed=" << motion_stats_.peak_speed << "m/s"
        << " avg_acc=" << avg_acc << "m/s^2"
        << " peak_acc=" << motion_stats_.peak_acc << "m/s^2";
    return oss.str();
}

bool TrajectoryExecutor::canonicalizeLineAxis(
    const Eigen::Vector3d& input_axis,
    Eigen::Vector3d* canonical_axis,
    bool* was_flipped) const
{
    return canonicalizeAxisToBaseX(input_axis, canonical_axis, was_flipped, 1e-9, 1e-6);
}

void TrajectoryExecutor::publishMiddleLineState()
{
    Eigen::Vector3d center;
    Eigen::Vector3d direction;
    double half_length = 0.0;
    uint64_t revision = 0;
    std::string ee_name;
    std::string frame_id;
    bool line_valid = false;

    {
        std::lock_guard<std::mutex> lock(line_mutex_);
        center = line_center_pose_.translation();
        direction = line_axis_;
        half_length = line_half_length_;
        revision = middle_line_revision_;
        ee_name = middle_line_ee_name_;
        frame_id = line_frame_;
        line_valid = line_valid_;
    }

    if (!finite3(center) ||
        !finite3(direction) ||
        direction.norm() < 1e-9 ||
        !std::isfinite(half_length) ||
        half_length <= 0.0)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] refusing to publish malformed middle-line state payload",
            name_.c_str());
        return;
    }

    direction.normalize();

    fr3_husky_msgs::msg::MiddleLine msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_id;

    msg.center.x = center.x();
    msg.center.y = center.y();
    msg.center.z = center.z();

    msg.direction.x = direction.x();
    msg.direction.y = direction.y();
    msg.direction.z = direction.z();

    msg.half_length = half_length;
    msg.ee_name = ee_name;
    msg.revision = revision;
    msg.valid = line_valid;

    middle_line_state_pub_->publish(msg);

    const Eigen::Vector3d start = center - half_length * direction;
    const Eigen::Vector3d end = center + half_length * direction;

    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] published middle line revision=%" PRIu64
        " frame=%s ee=%s center=%s direction=%s half_length=%.4f valid=%s start=%s end=%s",
        name_.c_str(),
        revision,
        frame_id.c_str(),
        ee_name.c_str(),
        vecToString(center).c_str(),
        vecToString(direction).c_str(),
        half_length,
        line_valid ? "true" : "false",
        vecToString(start).c_str(),
        vecToString(end).c_str());
}

void TrajectoryExecutor::publishCurrentTcpS()
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    double half_length = 0.0;
    bool line_valid = false;
    std::string ee_name;
    std::string line_frame;

    {
        std::lock_guard<std::mutex> lock(line_mutex_);
        center = line_center_pose_.translation();
        axis = line_axis_;
        half_length = line_half_length_;
        line_valid = line_valid_;
        ee_name = middle_line_ee_name_;
        line_frame = line_frame_;
    }

    if (!line_valid)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] current_tcp_s publish skipped: line geometry not valid",
            name_.c_str());
        return;
    }

    if (!sameFrame(line_frame, "base"))
    {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] current_tcp_s publish skipped: line_frame '%s' does not match expected robot-base frame 'base'",
            name_.c_str(),
            line_frame.c_str());
        return;
    }

    if (!finite3(center) || !finite3(axis))
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] current_tcp_s publish skipped: non-finite center or axis",
            name_.c_str());
        return;
    }

    const double axis_norm = axis.norm();
    if (!std::isfinite(axis_norm) || axis_norm < 1e-9)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] current_tcp_s publish skipped: axis norm too small",
            name_.c_str());
        return;
    }
    axis /= axis_norm;

    if (ee_name.empty())
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] current_tcp_s publish skipped: middle_line_ee_name is empty",
            name_.c_str());
        return;
    }

    if (!fr3_model_updater_.robot_data_->hasLinkFrame(ee_name))
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] current_tcp_s publish skipped: ee '%s' not found in robot model",
            name_.c_str(),
            ee_name.c_str());
        return;
    }

    const Eigen::Affine3d tcp_pose = fr3_model_updater_.robot_data_->getPose(ee_name);
    const Eigen::Vector3d tcp_position = tcp_pose.translation();
    if (!finite3(tcp_position))
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] current_tcp_s publish skipped: measured TCP position is non-finite",
            name_.c_str());
        return;
    }

    double current_tcp_s = 0.0;
    if (!projectTcpToLineS(tcp_position, center, axis, &current_tcp_s) || !std::isfinite(current_tcp_s))
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "[%s] current_tcp_s publish skipped: projection failed",
            name_.c_str());
        return;
    }

    if (std::isfinite(half_length) && half_length > 0.0 && std::abs(current_tcp_s) > half_length + 1e-6)
    {
        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "[%s] measured current_tcp_s=%.6f outside line half_length=%.6f (published unclamped)",
            name_.c_str(),
            current_tcp_s,
            half_length);
    }

    std_msgs::msg::Float64 msg;
    msg.data = current_tcp_s;
    current_tcp_s_pub_->publish(msg);
}

bool TrajectoryExecutor::geometryMutationBlocked() const
{
    if (!reject_line_services_while_active_)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(status_mutex_);
    return executor_state_ == ExecutorState::ACTIVE || stop_in_progress_;
}

bool TrajectoryExecutor::validateLineGeometry(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& axis,
    double half_length,
    std::string* reason) const
{
    if (!finite3(center) || !finite3(axis) || !std::isfinite(half_length))
    {
        if (reason) { *reason = "line geometry contains non-finite values"; }
        return false;
    }
    if (axis.norm() < 1e-9)
    {
        if (reason) { *reason = "line axis norm too small"; }
        return false;
    }
    if (half_length < min_line_half_length_ || half_length > max_line_half_length_)
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "half_length=" << half_length << " outside [" << min_line_half_length_ << ", " << max_line_half_length_ << "]";
            *reason = oss.str();
        }
        return false;
    }

    const Eigen::Vector3d n_axis = axis.normalized();
    const double dot_x = n_axis.dot(Eigen::Vector3d::UnitX());
    if (!std::isfinite(dot_x) || std::abs(dot_x) < 1e-6)
    {
        if (reason) { *reason = "line axis base-x component too small for canonical sign"; }
        return false;
    }
    if (dot_x < 0.0)
    {
        if (reason) { *reason = "line axis must point toward base +x"; }
        return false;
    }

    const Eigen::Vector3d start = center - half_length * n_axis;
    const Eigen::Vector3d end = center + half_length * n_axis;

    auto in_workspace = [&](const Eigen::Vector3d& p) -> bool
    {
        return p.x() >= workspace_min_.x() && p.x() <= workspace_max_.x() &&
               p.y() >= workspace_min_.y() && p.y() <= workspace_max_.y() &&
               p.z() >= workspace_min_.z() && p.z() <= workspace_max_.z();
    };

    if (!in_workspace(center) || !in_workspace(start) || !in_workspace(end))
    {
        if (reason)
        {
            std::ostringstream oss;
            oss << "line center/start/end outside workspace. center=" << vecToString(center)
                << " start=" << vecToString(start) << " end=" << vecToString(end);
            *reason = oss.str();
        }
        return false;
    }

    if (center.z() < safety_min_z_ + safety_margin_z_ ||
        start.z() < safety_min_z_ + safety_margin_z_ ||
        end.z() < safety_min_z_ + safety_margin_z_)
    {
        if (reason) { *reason = "line z below safety_min_z + margin"; }
        return false;
    }

    return true;
}

std::string TrajectoryExecutor::commandToString(uint8_t command) const
{
    switch (command)
    {
        case ActionT::Goal::CMD_GOTO_CENTER: return "CMD_GOTO_CENTER";
        case ActionT::Goal::CMD_GOTO_START: return "CMD_GOTO_START";
        case ActionT::Goal::CMD_GOTO_END: return "CMD_GOTO_END";
        case ActionT::Goal::CMD_PHASE_TEST: return "CMD_PHASE_TEST";
        case ActionT::Goal::CMD_FAST_SWEEP: return "CMD_FAST_SWEEP";
        case ActionT::Goal::CMD_GOTO_S: return "CMD_GOTO_S";
        default: return "UNKNOWN";
    }
}

std::string TrajectoryExecutor::makeAbortDetail(
    const std::string& reason,
    const std::string& phase,
    const Eigen::Vector3d& current,
    const Eigen::Vector3d& desired,
    double tracking_error_pos,
    double tracking_error_z) const
{
    std::ostringstream oss;
    oss << reason
        << " phase=" << phase
        << " current=" << vecToString(current)
        << " desired=" << vecToString(desired)
        << " tracking_error_pos=" << tracking_error_pos
        << " tracking_error_z=" << tracking_error_z;
    return oss.str();
}

std::string TrajectoryExecutor::buildStatusMessage() const
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    std::ostringstream oss;
    oss << "state=" << executorStateToString(executor_state_)
        << " last_start_time=" << std::fixed << std::setprecision(6) << last_start_time_.seconds()
        << " last_stop_time=" << last_stop_time_.seconds()
        << " last_stop_reason=" << lastStopReasonToString(last_stop_reason_)
        << " stop_in_progress=" << (stop_in_progress_ ? "true" : "false")
        << " phase=" << last_phase_
        << " current=" << vecToString(last_current_pos_)
        << " desired=" << vecToString(last_desired_pos_)
        << " err_pos=" << last_tracking_error_pos_
        << " err_z=" << last_tracking_error_z_
        << " s_des=" << last_s_des_
        << " sdot_des=" << last_sdot_des_;
    return oss.str();
}

std::string TrajectoryExecutor::executorStateToString(ExecutorState state) const
{
    switch (state)
    {
        case ExecutorState::STOPPED: return "STOPPED";
        case ExecutorState::ACTIVE: return "ACTIVE";
        case ExecutorState::CANCELING: return "CANCELING";
        case ExecutorState::ABORTED: return "ABORTED";
        default: return "UNKNOWN";
    }
}

std::string TrajectoryExecutor::lastStopReasonToString(LastStopReason reason) const
{
    switch (reason)
    {
        case LastStopReason::NONE: return "NONE";
        case LastStopReason::SUCCEEDED: return "SUCCEEDED";
        case LastStopReason::CANCELED: return "CANCELED";
        case LastStopReason::ABORTED: return "ABORTED";
        default: return "UNKNOWN";
    }
}

void TrajectoryExecutor::handleGetStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    response->success = true;
    response->message = buildStatusMessage();
}

void TrajectoryExecutor::handleCaptureLineCenter(
    const std::shared_ptr<fr3_husky_msgs::srv::CaptureLineCenter::Request> request,
    std::shared_ptr<fr3_husky_msgs::srv::CaptureLineCenter::Response> response)
{
    if (geometryMutationBlocked())
    {
        response->success = false;
        response->message = "trajectory active; line geometry not changed";
        return;
    }

    const std::string ee = request->ee_name.empty() ? ee_name_default_ : request->ee_name;
    if (!fr3_model_updater_.robot_data_->hasLinkFrame(ee))
    {
        response->success = false;
        response->message = "invalid ee_name";
        return;
    }

    const auto pose = fr3_model_updater_.robot_data_->getPose(ee);
    Eigen::Affine3d new_center = line_center_pose_;
    new_center.translation() = pose.translation();

    if (request->keep_current_orientation)
    {
        new_center.linear() = pose.linear();
    }
    else
    {
        new_center.linear() = makeBaseOrientationFromParams();
    }

    std::string geom_reason;
    if (!validateLineGeometry(new_center.translation(), line_axis_, line_half_length_, &geom_reason))
    {
        response->success = false;
        response->message = "capture rejected: " + geom_reason;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(line_mutex_);
        line_center_pose_ = new_center;
        have_captured_orientation_ = request->keep_current_orientation;
        middle_line_ee_name_ = ee;
        line_valid_ = true;
        ++middle_line_revision_;
    }

    publishMiddleLineState();

    response->success = true;
    response->message = "captured line center";
}

void TrajectoryExecutor::handleSetLineCenter(
    const std::shared_ptr<fr3_husky_msgs::srv::SetLineCenter::Request> request,
    std::shared_ptr<fr3_husky_msgs::srv::SetLineCenter::Response> response)
{
    if (geometryMutationBlocked())
    {
        response->success = false;
        response->message = "trajectory active; line geometry not changed";
        return;
    }

    Eigen::Affine3d new_center = line_center_pose_;
    new_center.translation() = Eigen::Vector3d(request->x, request->y, request->z);

    const std::string ee = request->ee_name.empty() ? ee_name_default_ : request->ee_name;

    if (request->use_current_orientation)
    {
        if (!fr3_model_updater_.robot_data_->hasLinkFrame(ee))
        {
            response->success = false;
            response->message = "invalid ee_name for current orientation";
            return;
        }
        new_center.linear() = fr3_model_updater_.robot_data_->getPose(ee).linear();
    }
    else if (target_orientation_mode_ == "base_rpy")
    {
        new_center.linear() = makeBaseOrientationFromParams();
    }

    std::string geom_reason;
    if (!validateLineGeometry(new_center.translation(), line_axis_, line_half_length_, &geom_reason))
    {
        response->success = false;
        response->message = "set center rejected: " + geom_reason;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(line_mutex_);
        line_center_pose_ = new_center;
        have_captured_orientation_ = request->use_current_orientation;

        if (request->use_current_orientation)
        {
            middle_line_ee_name_ = ee;
        }

        line_valid_ = true;
        ++middle_line_revision_;
    }

    publishMiddleLineState();

    response->success = true;
    response->message = "set line center";
}

void TrajectoryExecutor::handleSetLineParams(
    const std::shared_ptr<fr3_husky_msgs::srv::SetLineParams::Request> request,
    std::shared_ptr<fr3_husky_msgs::srv::SetLineParams::Response> response)
{
    if (geometryMutationBlocked())
    {
        response->success = false;
        response->message = "trajectory active; line geometry not changed";
        return;
    }

    if (!std::isfinite(request->axis_x) || !std::isfinite(request->axis_y) || !std::isfinite(request->axis_z) ||
        !std::isfinite(request->half_length))
    {
        response->success = false;
        response->message = "axis and half_length must be finite";
        return;
    }

    Eigen::Vector3d axis(request->axis_x, request->axis_y, request->axis_z);
    bool axis_flipped = false;
    Eigen::Vector3d canonical_axis = axis;
    if (!canonicalizeLineAxis(axis, &canonical_axis, &axis_flipped))
    {
        response->success = false;
        response->message = "axis must be finite/non-zero and have non-zero +x component in base frame";
        return;
    }

    if (axis_flipped)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] set_line_params canonicalized axis sign to base +x",
                    name_.c_str());
    }

    if (request->half_length <= 0.0)
    {
        response->success = false;
        response->message = "half_length must be > 0";
        return;
    }
    const double half_length = std::clamp(request->half_length, min_line_half_length_, max_line_half_length_);

    double updated_safety_min_z = safety_min_z_;
    if (allow_service_update_safety_min_z_)
    {
        if (!std::isfinite(request->safety_min_z) || request->safety_min_z < hard_min_safety_z_)
        {
            response->success = false;
            response->message = "requested safety_min_z invalid or below hard minimum";
            return;
        }
        updated_safety_min_z = request->safety_min_z;
    }
    else if (std::isfinite(request->safety_min_z) && std::abs(request->safety_min_z - safety_min_z_) > 1e-9)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] ignoring requested safety_min_z=%.6f because allow_service_update_safety_min_z=false",
                    name_.c_str(), request->safety_min_z);
    }

    Eigen::Vector3d center_snapshot;
    {
        std::lock_guard<std::mutex> lock(line_mutex_);
        center_snapshot = line_center_pose_.translation();
    }

    const double old_safety = safety_min_z_;
    safety_min_z_ = updated_safety_min_z;
    std::string geom_reason;
    const bool ok = validateLineGeometry(center_snapshot, canonical_axis, half_length, &geom_reason);
    safety_min_z_ = old_safety;
    if (!ok)
    {
        response->success = false;
        response->message = "set params rejected: " + geom_reason;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(line_mutex_);
        line_axis_ = canonical_axis;
        line_half_length_ = half_length;
        safety_min_z_ = updated_safety_min_z;
        line_valid_ = true;
        ++middle_line_revision_;
    }

    publishMiddleLineState();

    response->success = true;
    response->message = "set line params";
}

void TrajectoryExecutor::handleProjectPointToLine(
    const std::shared_ptr<fr3_husky_msgs::srv::ProjectPointToLine::Request> request,
    std::shared_ptr<fr3_husky_msgs::srv::ProjectPointToLine::Response> response)
{
    auto fail = [&](const std::string& message)
    {
        response->success = false;
        response->message = message;
        response->s = 0.0;
        response->cross_track_error_m = 0.0;
        response->line_half_length_m = 0.0;

        const auto stamp = node_->now();

        response->projected_point.header.stamp = stamp;
        response->projected_point.header.frame_id = line_frame_;
        response->projected_point.point.x = 0.0;
        response->projected_point.point.y = 0.0;
        response->projected_point.point.z = 0.0;

        response->line_center.header.stamp = stamp;
        response->line_center.header.frame_id = line_frame_;
        response->line_center.point.x = 0.0;
        response->line_center.point.y = 0.0;
        response->line_center.point.z = 0.0;
    };

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (executor_state_ == ExecutorState::ACTIVE || stop_in_progress_)
        {
            fail("trajectory active/canceling; projection rejected to avoid racing active line snapshot");
            return;
        }
    }

    const std::string request_frame = cleanFrame(request->point.header.frame_id);
    if (!sameFrame(request_frame, line_frame_))
    {
        std::ostringstream oss;
        oss << "frame mismatch: request point frame_id='" << request_frame
            << "', expected line_frame='" << line_frame_ << "'";
        fail(oss.str());
        return;
    }

    const Eigen::Vector3d p(
        request->point.point.x,
        request->point.point.y,
        request->point.point.z);

    if (!finite3(p))
    {
        fail("request point contains non-finite values");
        return;
    }

    Eigen::Vector3d center;
    Eigen::Vector3d axis;
    double half_length = 0.0;
    bool line_valid = false;

    {
        std::lock_guard<std::mutex> lock(line_mutex_);
        center = line_center_pose_.translation();
        axis = line_axis_;
        half_length = std::clamp(line_half_length_, min_line_half_length_, max_line_half_length_);
        line_valid = line_valid_;
    }

    if (!line_valid)
    {
        fail("middle line is not valid");
        return;
    }

    if (!finite3(center) || !finite3(axis) || axis.norm() < 1e-9 || !std::isfinite(half_length))
    {
        fail("internal line geometry is invalid/non-finite");
        return;
    }

    axis.normalize();

    std::string geom_reason;
    if (!validateLineGeometry(center, axis, half_length, &geom_reason))
    {
        fail("line geometry validation failed: " + geom_reason);
        return;
    }

    const double s = axis.dot(p - center);
    const Eigen::Vector3d projected = center + axis * s;
    Eigen::Vector3d cross_track_delta = p - projected;

    // The incoming interception point represents the ball location,
    // while the configured line represents the TCP center. Their z
    // coordinates are intentionally different, so validate only the
    // horizontal alignment.
    cross_track_delta.z() = 0.0;

    const double cross_track_error = cross_track_delta.norm();

    if (!request->allow_out_of_bounds && std::abs(s) > half_length + 1e-6)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "projected s=" << s
            << " outside line bounds [-" << half_length << ", " << half_length << "]";
        fail(oss.str());
        return;
    }

    if (std::isfinite(request->max_cross_track_error_m) &&
        request->max_cross_track_error_m > 0.0 &&
        cross_track_error > request->max_cross_track_error_m)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "cross-track error=" << cross_track_error
            << " exceeds max_cross_track_error_m=" << request->max_cross_track_error_m
            << "; p=" << vecToString(p)
            << " projected=" << vecToString(projected);
        fail(oss.str());
        return;
    }

    const auto stamp = node_->now();

    response->success = true;

    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "projected point to line: s=" << s
            << " cross_track_error=" << cross_track_error
            << " half_length=" << half_length;
        response->message = oss.str();
    }

    response->s = s;
    response->cross_track_error_m = cross_track_error;
    response->line_half_length_m = half_length;

    response->projected_point.header.stamp = stamp;
    response->projected_point.header.frame_id = line_frame_;
    response->projected_point.point.x = projected.x();
    response->projected_point.point.y = projected.y();
    response->projected_point.point.z = projected.z();

    response->line_center.header.stamp = stamp;
    response->line_center.header.frame_id = line_frame_;
    response->line_center.point.x = center.x();
    response->line_center.point.y = center.y();
    response->line_center.point.z = center.z();

    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[%s] projected s=%.6f cross_track=%.6f half_length=%.6f",
        name_.c_str(),
        s,
        cross_track_error,
        half_length);
}

void TrajectoryExecutor::updateComputeTimingDebug(std::chrono::steady_clock::time_point start_time)
{
    if (!enable_compute_timing_debug_)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double compute_us = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count());

    timing_max_compute_us_ = std::max(timing_max_compute_us_, compute_us);
    if (compute_us > 800.0) { ++timing_overrun_800us_; }
    if (compute_us > 1000.0) { ++timing_overrun_1000us_; }
    ++timing_call_count_;

    const double elapsed_report_s = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(now - timing_last_report_time_).count()) * 1e-6;

    if (elapsed_report_s >= 1.0)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[trajectory_executor timing] compute_us_current=%.1f compute_us_max=%.1f calls=%" PRIu64
                    " overruns_gt_800us=%" PRIu64 " overruns_gt_1000us=%" PRIu64,
                    compute_us, timing_max_compute_us_, timing_call_count_,
                    timing_overrun_800us_, timing_overrun_1000us_);

        timing_last_report_time_ = now;
        timing_max_compute_us_ = 0.0;
        timing_overrun_800us_ = 0;
        timing_overrun_1000us_ = 0;
        timing_call_count_ = 0;
    }
}

REGISTER_FR3_ACTION_SERVER(TrajectoryExecutor, "trajectory_executor")


}  // namespace fr3_husky_controller::servers::fr3