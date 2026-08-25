#include <fr3_husky_controller/servers/fr3/cont_tracker_action_server.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace fr3_husky_controller::servers::fr3
{

namespace
{

FR3ModelUpdater& getFR3ModelUpdater(ModelUpdaterBase& model_updater, const std::string& server_name)
{
    auto* updater = dynamic_cast<FR3ModelUpdater*>(&model_updater);
    if (!updater)
    {
        throw std::runtime_error("[" + server_name + "] requires FR3ModelUpdater");
    }
    return *updater;
}

template <typename T, typename NodeT>
T declareOrGetParameter(
    const std::shared_ptr<NodeT>& node,
    const std::string& parameter_name,
    const T& default_value)
{
    if (node->has_parameter(parameter_name))
    {
        return node->get_parameter(parameter_name).template get_value<T>();
    }
    return node->template declare_parameter<T>(parameter_name, default_value);
}

double clamp(double value, double lower, double upper)
{
    return std::max(lower, std::min(upper, value));
}

double sign(double value)
{
    return (value > 0.0) - (value < 0.0);
}

bool finite3(const Eigen::Vector3d& value)
{
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

Eigen::Matrix3d rpyDegToRot(double roll_deg, double pitch_deg, double yaw_deg)
{
    constexpr double kPi = 3.14159265358979323846;
    const Eigen::AngleAxisd roll(roll_deg * kPi / 180.0, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(pitch_deg * kPi / 180.0, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}

void copyTextToArray(const std::string& src, std::array<char, 64>* dst, const char* fallback)
{
    if (!dst)
    {
        return;
    }
    std::memset(dst->data(), 0, dst->size());
    const char* text = src.empty() ? fallback : src.c_str();
    std::snprintf(dst->data(), dst->size(), "%s", text);
}

}  // namespace

ContTracker::ContTracker(
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

    ee_name_default_ = declareOrGetParameter<std::string>(
        node_, name_ + ".ee_name", "right_fr3_link8");

    default_v_max_ = declareOrGetParameter<double>(node_, name_ + ".default_v_max", 0.40);
    default_a_max_ = declareOrGetParameter<double>(node_, name_ + ".default_a_max", 1.00);
    hard_v_max_ = declareOrGetParameter<double>(node_, name_ + ".hard_v_max", 1.90);
    hard_a_max_ = declareOrGetParameter<double>(node_, name_ + ".hard_a_max", 5.00);

    safety_min_z_ = declareOrGetParameter<double>(node_, name_ + ".safety_min_z", 0.08);
    hard_min_safety_z_ = declareOrGetParameter<double>(node_, name_ + ".hard_min_safety_z", 0.05);
    safety_margin_z_ = declareOrGetParameter<double>(node_, name_ + ".safety_margin_z", 0.005);
    max_tracking_error_pos_ = declareOrGetParameter<double>(
        node_, name_ + ".max_tracking_error_pos", 0.02);
    max_tracking_error_z_ = declareOrGetParameter<double>(
        node_, name_ + ".max_tracking_error_z", 0.005);
    max_start_cross_track_m_ = declareOrGetParameter<double>(
        node_, name_ + ".max_start_cross_track_m", 0.015);

    target_timeout_sec_ = declareOrGetParameter<double>(
        node_, name_ + ".target_timeout_sec", 0.25);
    max_session_sec_ = declareOrGetParameter<double>(
        node_, name_ + ".max_session_sec", 5.0);
    terminal_brake_timeout_sec_ = declareOrGetParameter<double>(
        node_, name_ + ".terminal_brake_timeout_sec", 1.0);
    target_deadband_m_ = declareOrGetParameter<double>(
        node_, name_ + ".target_deadband_m", 0.002);
    holding_velocity_mps_ = declareOrGetParameter<double>(
        node_, name_ + ".holding_velocity_mps", 0.005);
    max_control_period_sec_ = declareOrGetParameter<double>(
        node_, name_ + ".max_control_period_sec", 0.02);
    feedback_period_sec_ = declareOrGetParameter<double>(
        node_, name_ + ".feedback_period_sec", 0.02);

    const bool had_abort_param = node_->has_parameter(name_ + ".abort_on_stale_target");
    const bool had_stale_behavior_param = node_->has_parameter(name_ + ".stale_target_behavior");

    abort_on_stale_target_ = declareOrGetParameter<bool>(
        node_, name_ + ".abort_on_stale_target", true);
    const std::string stale_behavior_param = declareOrGetParameter<std::string>(
        node_, name_ + ".stale_target_behavior", "hold_last");
    if (stale_behavior_param == "brake")
    {
        stale_target_behavior_ = StaleTargetBehavior::BRAKE;
    }
    else if (stale_behavior_param == "abort")
    {
        stale_target_behavior_ = StaleTargetBehavior::ABORT;
    }
    else
    {
        stale_target_behavior_ = StaleTargetBehavior::HOLD_LAST;
    }

    if (!had_stale_behavior_param && had_abort_param && abort_on_stale_target_)
    {
        stale_target_behavior_ = StaleTargetBehavior::ABORT;
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] deprecated abort_on_stale_target=true compatibility applied; set stale_target_behavior explicitly",
            name_.c_str());
    }

    use_velocity_feedforward_ = declareOrGetParameter<bool>(
        node_, name_ + ".use_velocity_feedforward", true);
    allow_clamped_targets_ = declareOrGetParameter<bool>(
        node_, name_ + ".allow_clamped_targets", false);
    control_mode_ = declareOrGetParameter<int>(node_, name_ + ".control_mode", 0);
    abort_on_repeated_qp_failure_ = declareOrGetParameter<bool>(
        node_, name_ + ".abort_on_repeated_qp_failure", false);
    max_consecutive_qp_failures_ = std::max(
        1,
        declareOrGetParameter<int>(node_, name_ + ".max_consecutive_qp_failures", 20));

    min_line_half_length_ = declareOrGetParameter<double>(
        node_, name_ + ".min_line_half_length", 0.005);
    max_line_half_length_ = declareOrGetParameter<double>(
        node_, name_ + ".max_line_half_length", 0.40);

    const auto workspace_min = declareOrGetParameter<std::vector<double>>(
        node_, name_ + ".workspace_min", std::vector<double>{0.05, -0.4, 0.05});
    const auto workspace_max = declareOrGetParameter<std::vector<double>>(
        node_, name_ + ".workspace_max", std::vector<double>{0.95, 0.4, 0.65});
    if (workspace_min.size() == 3 && workspace_max.size() == 3)
    {
        workspace_min_ = Eigen::Vector3d(workspace_min[0], workspace_min[1], workspace_min[2]);
        workspace_max_ = Eigen::Vector3d(workspace_max[0], workspace_max[1], workspace_max[2]);
    }

    target_orientation_mode_ = declareOrGetParameter<std::string>(
        node_, name_ + ".target_orientation_mode", "captured");
    base_orientation_rpy_deg_ = declareOrGetParameter<std::vector<double>>(
        node_, name_ + ".base_orientation_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});
    if (base_orientation_rpy_deg_.size() != 3)
    {
        base_orientation_rpy_deg_ = {0.0, 0.0, 0.0};
    }

    if (!std::isfinite(safety_min_z_) || safety_min_z_ < hard_min_safety_z_)
    {
        safety_min_z_ = hard_min_safety_z_;
    }
    target_timeout_sec_ = std::max(0.01, target_timeout_sec_);
    max_session_sec_ = std::max(target_timeout_sec_, max_session_sec_);
    terminal_brake_timeout_sec_ = std::max(0.1, terminal_brake_timeout_sec_);
    target_deadband_m_ = std::max(0.0, target_deadband_m_);
    holding_velocity_mps_ = std::max(1e-4, holding_velocity_mps_);
    max_control_period_sec_ = std::max(1e-3, max_control_period_sec_);
    feedback_period_sec_ = std::max(1e-3, feedback_period_sec_);

    const auto line_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    middle_line_sub_ = node_->create_subscription<fr3_husky_msgs::msg::MiddleLine>(
        "/trajectory_executor/middle_line_state",
        line_qos,
        std::bind(&ContTracker::onMiddleLine, this, std::placeholders::_1));

    const auto target_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    target_s_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/cont_tracker/target_s",
        target_qos,
        std::bind(&ContTracker::onTargetS, this, std::placeholders::_1));

    accepted_target_s_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
        "/cont_tracker/accepted_target_s", rclcpp::QoS(10).reliable());
    accepted_target_base_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
        "/cont_tracker/accepted_target_base", rclcpp::QoS(10).reliable());
    track_s_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
        "/cont_tracker/track_s",
        rclcpp::QoS(100).reliable());
    get_status_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "/cont_tracker/get_status",
        std::bind(&ContTracker::handleGetStatus, this, std::placeholders::_1, std::placeholders::_2));

    telemetry_timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(feedback_period_sec_)),
        std::bind(&ContTracker::telemetryTimerCallback, this));

    line_slot_a_.valid = false;
    line_slot_b_.valid = false;
}

void ContTracker::onMiddleLine(const fr3_husky_msgs::msg::MiddleLine::SharedPtr msg)
{
    LineSnapshot line;
    line.center = Eigen::Vector3d(msg->center.x, msg->center.y, msg->center.z);
    line.axis = Eigen::Vector3d(msg->direction.x, msg->direction.y, msg->direction.z);
    line.half_length = msg->half_length;
    copyTextToArray(msg->header.frame_id, &line.frame_id, "base");
    copyTextToArray(msg->ee_name, &line.ee_name, "");
    line.revision = msg->revision;
    line.valid = msg->valid && finite3(line.center) && finite3(line.axis) &&
                 line.axis.norm() > 1e-9 && std::isfinite(line.half_length) &&
                 line.half_length > 0.0;
    if (line.valid)
    {
        line.axis.normalize();
    }

    const std::uint64_t before = line_sequence_.load(std::memory_order_relaxed);
    line_sequence_.store(before + 1, std::memory_order_release);
    if (((before / 2U) % 2U) == 0U)
    {
        line_slot_b_ = line;
    }
    else
    {
        line_slot_a_ = line;
    }
    line_sequence_.store(before + 2, std::memory_order_release);
}

void ContTracker::onTargetS(const std_msgs::msg::Float64::SharedPtr msg)
{
    if (!session_accepts_updates_.load(std::memory_order_acquire))
    {
        return;
    }

    const double bound = active_target_bound_m_.load(std::memory_order_relaxed);
    double target_s = msg->data;
    if (!std::isfinite(target_s) || !std::isfinite(bound) || bound <= 0.0)
    {
        return;
    }

    if (std::abs(target_s) > bound + 1e-9)
    {
        if (!allow_clamped_targets_)
        {
            return;
        }
        target_s = clamp(target_s, -bound, bound);
    }

    target_sequence_.fetch_add(1, std::memory_order_acq_rel);
    target_s_atomic_.store(target_s, std::memory_order_relaxed);
    target_receipt_ns_atomic_.store(1, std::memory_order_relaxed);
    target_sequence_.fetch_add(1, std::memory_order_release);

    publishTrackS(target_s);
}

bool ContTracker::acceptGoal(const ActionT::Goal& goal)
{
    if (!model_updater_.HasEffortCommandInterface())
    {
        return false;
    }

    if (goal.command != ActionT::Goal::CMD_GOTO_S)
    {
        return false;
    }

    const std::string ee = goal.ee_name.empty() ? ee_name_default_ : goal.ee_name;
    if (!fr3_model_updater_.robot_data_->hasLinkFrame(ee))
    {
        return false;
    }

    LineSnapshot line;
    if (!readLatestLine(&line))
    {
        return false;
    }
    TrackerError err = TrackerError::NONE;
    if (!validateLineGeometry(line, &err))
    {
        return false;
    }
    if (!std::isfinite(goal.target_s) || std::abs(goal.target_s) > line.half_length + 1e-9)
    {
        return false;
    }

    const double v_max = goal.v_max > 0.0 ? goal.v_max : default_v_max_;
    const double a_max = goal.a_max > 0.0 ? goal.a_max : default_a_max_;
    if (!std::isfinite(v_max) || !std::isfinite(a_max) ||
        v_max <= 0.0 || a_max <= 0.0 ||
        v_max > hard_v_max_ || a_max > hard_a_max_)
    {
        return false;
    }

    return true;
}

void ContTracker::onGoalAccepted(const ActionT::Goal& goal)
{
    goal_ = goal;
    if (goal_.ee_name.empty())
    {
        goal_.ee_name = ee_name_default_;
    }
    if (goal_.v_max <= 0.0) { goal_.v_max = default_v_max_; }
    if (goal_.a_max <= 0.0) { goal_.a_max = default_a_max_; }

    publishTrackS(goal_.target_s);

    control_ee_name_ = goal_.ee_name;
    ee_data_.clear();
    ee_data_[control_ee_name_] = drc::TaskSpaceData::Zero();
    ee_task_it_ = ee_data_.find(control_ee_name_);

    session_accepts_updates_.store(false, std::memory_order_release);
    active_target_bound_m_.store(0.0, std::memory_order_relaxed);
    start_failed_ = false;
    start_error_ = TrackerError::NONE;
    rt_error_ = TrackerError::NONE;
    phase_ = TrackerPhase::ACCEPTED;
    first_command_written_ = false;
    terminal_mode_ = TerminalMode::NONE;
    stale_braking_ = false;
    stale_hold_active_ = false;
    consecutive_qp_failures_ = 0;
    consumed_target_sequence_ = 0;
    session_elapsed_sec_ = 0.0;
    target_age_sec_ = 0.0;
    terminal_brake_elapsed_sec_ = 0.0;
}

void ContTracker::onStart()
{
    const auto gate_result = motion_gate_->tryAcquireRT(MotionGate::Owner::CONT_TRACKER);
    if (gate_result == MotionGate::AcquireResult::BUSY)
    {
        start_failed_ = true;
        start_error_ = TrackerError::START_FAILED;
        storeRtError(TrackerError::START_FAILED);
        session_active_.store(false, std::memory_order_release);
        const MotionGate::Owner current_owner = motion_gate_->owner();
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s] start rejected by motion gate owner=%s name=%s",
                     name_.c_str(),
                     MotionGate::ownerToString(current_owner).c_str(),
                     motion_gate_->ownerName(current_owner).c_str());
        finalizeGoal(StopReason::ABORTED);
        return;
    }
    gate_acquired_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] gate acquired for activation", name_.c_str());

    executor_state_.store(ExecutorState::ACTIVE, std::memory_order_release);
    stop_in_progress_.store(false, std::memory_order_release);
    last_stop_reason_.store(LastStopReason::NONE, std::memory_order_release);

    resetMotionStats();

    TrackerError error = TrackerError::NONE;
    if (!snapshotAndValidateLine(&error) || !initializeReference(&error))
    {
        start_failed_ = true;
        start_error_ = error;
        storeRtError(error == TrackerError::NONE ? TrackerError::START_FAILED : error);
        session_active_.store(false, std::memory_order_release);
        finalizeGoal(StopReason::ABORTED);
        return;
    }

    const auto current_pose = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    if (ee_task_it_ == ee_data_.end())
    {
        start_failed_ = true;
        start_error_ = TrackerError::TASK_DATA_MISSING;
        storeRtError(TrackerError::TASK_DATA_MISSING);
        session_active_.store(false, std::memory_order_release);
        finalizeGoal(StopReason::ABORTED);
        return;
    }

    ee_task_it_->second.x = current_pose;
    ee_task_it_->second.xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    ee_task_it_->second.xddot.setZero();
    ee_task_it_->second.setInit();
    ee_task_it_->second.setDesired();

    active_orientation_ = resolveActiveOrientation(current_pose);
    target_pose_ = current_pose;
    target_pose_.linear() = active_orientation_;

    accepted_target_s_ = goal_.target_s;
    braking_target_s_ = accepted_target_s_;
    writeInitialTarget(goal_.target_s, 1);
    consumed_target_sequence_ = target_sequence_.load(std::memory_order_acquire);

    active_target_bound_m_.store(active_line_.half_length, std::memory_order_relaxed);
    session_accepts_updates_.store(true, std::memory_order_release);
    session_active_.store(true, std::memory_order_release);

    phase_ = referenceIsHolding() ? TrackerPhase::HOLDING : TrackerPhase::TRACKING;
    writeTelemetrySnapshot(true);
}

ContTracker::ComputeResult ContTracker::compute(
    const rclcpp::Time&,
    const rclcpp::Duration& period)
{
    if (start_failed_)
    {
        return ComputeResult::ABORTED;
    }

    const double dt = period.seconds();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > max_control_period_sec_)
    {
        storeRtError(TrackerError::INVALID_DT);
        return ComputeResult::ABORTED;
    }

    session_elapsed_sec_ += dt;
    target_age_sec_ += dt;
    if (terminal_mode_ != TerminalMode::NONE)
    {
        terminal_brake_elapsed_sec_ += dt;
    }

    bool accepted_changed = false;

    if (terminal_mode_ == TerminalMode::NONE && session_elapsed_sec_ >= max_session_sec_)
    {
        beginTerminalBrake(TerminalMode::SUCCEED_AFTER_BRAKE, TrackerError::SESSION_TIMEOUT);
    }

    double new_target = 0.0;
    std::int64_t receipt_ns = 0;
    std::uint64_t sequence = 0;
    if (terminal_mode_ == TerminalMode::NONE &&
        readLatestTarget(&new_target, &receipt_ns, &sequence) &&
        sequence != consumed_target_sequence_)
    {
        consumed_target_sequence_ = sequence;
        if (std::abs(new_target) <= active_line_.half_length + 1e-9)
        {
            accepted_target_s_ = clamp(new_target, -active_line_.half_length, active_line_.half_length);
            target_age_sec_ = 0.0;
            stale_braking_ = false;
            stale_hold_active_ = false;
            accepted_changed = true;
        }
    }

    if (terminal_mode_ == TerminalMode::NONE && target_age_sec_ > target_timeout_sec_)
    {
        switch (stale_target_behavior_)
        {
            case StaleTargetBehavior::HOLD_LAST:
                stale_hold_active_ = true;
                break;
            case StaleTargetBehavior::BRAKE:
                if (!stale_braking_)
                {
                    beginRecoverableStaleBrake();
                }
                break;
            case StaleTargetBehavior::ABORT:
                beginTerminalBrake(TerminalMode::ABORT_AFTER_BRAKE, TrackerError::STALE_TARGET_ABORT);
                break;
            default:
                break;
        }
    }

    updateReference(dt);
    const Eigen::Vector3d desired_pos = active_line_.center + active_line_.axis * s_ref_;
    const Eigen::Vector3d desired_vel = active_line_.axis * sdot_ref_;

    if (terminal_mode_ != TerminalMode::NONE)
    {
        phase_ = TrackerPhase::TERMINAL_BRAKE;
    }
    else if (stale_braking_)
    {
        phase_ = TrackerPhase::BRAKING_STALE;
    }
    else if (stale_hold_active_)
    {
        phase_ = referenceIsHolding() ? TrackerPhase::HOLDING_LAST_STALE : TrackerPhase::TRACKING_LAST_STALE;
    }
    else
    {
        phase_ = referenceIsHolding() ? TrackerPhase::HOLDING : TrackerPhase::TRACKING;
    }

    const bool moving = std::abs(sdot_ref_) > holding_velocity_mps_;
    if (!computeCartesianCommand(desired_pos, desired_vel, dt, moving))
    {
        writeTelemetrySnapshot(accepted_changed);
        return ComputeResult::ABORTED;
    }

    if (terminal_mode_ != TerminalMode::NONE)
    {
        const bool stopped = referenceIsHolding();
        const bool brake_timed_out = terminal_brake_elapsed_sec_ > terminal_brake_timeout_sec_;
        if (brake_timed_out && !stopped)
        {
            storeRtError(TrackerError::BRAKE_TIMEOUT);
            writeTelemetrySnapshot(accepted_changed);
            return ComputeResult::ABORTED;
        }
        if (stopped)
        {
            writeTelemetrySnapshot(accepted_changed);
            return terminal_mode_ == TerminalMode::SUCCEED_AFTER_BRAKE
                       ? ComputeResult::SUCCEEDED
                       : ComputeResult::ABORTED;
        }
    }

    writeTelemetrySnapshot(accepted_changed);
    return ComputeResult::RUNNING;
}

bool ContTracker::readLatestLine(LineSnapshot* line) const
{
    if (!line)
    {
        return false;
    }
    const std::uint64_t before = line_sequence_.load(std::memory_order_acquire);
    if ((before & 1U) != 0U)
    {
        return false;
    }
    const LineSnapshot copy = (((before / 2U) % 2U) == 0U) ? line_slot_a_ : line_slot_b_;
    const std::uint64_t after = line_sequence_.load(std::memory_order_acquire);
    if (before != after || (after & 1U) != 0U)
    {
        return false;
    }
    *line = copy;
    return true;
}

bool ContTracker::snapshotAndValidateLine(TrackerError* error)
{
    if (!readLatestLine(&active_line_))
    {
        if (error) { *error = TrackerError::INVALID_LINE; }
        return false;
    }
    if (!validateLineGeometry(active_line_, error))
    {
        return false;
    }
    active_line_.axis.normalize();
    return true;
}

bool ContTracker::validateLineGeometry(const LineSnapshot& line, TrackerError* error) const
{
    auto fail = [&](TrackerError err)
    {
        if (error) { *error = err; }
        return false;
    };

    if (!line.valid)
    {
        return fail(TrackerError::INVALID_LINE);
    }
    if (!finite3(line.center) || !finite3(line.axis) || line.axis.norm() < 1e-9 ||
        !std::isfinite(line.half_length))
    {
        return fail(TrackerError::INVALID_LINE);
    }
    if (line.half_length < min_line_half_length_ || line.half_length > max_line_half_length_)
    {
        return fail(TrackerError::INVALID_LINE);
    }

    const Eigen::Vector3d axis = line.axis.normalized();
    const Eigen::Vector3d start = line.center - line.half_length * axis;
    const Eigen::Vector3d end = line.center + line.half_length * axis;
    const auto inside_workspace = [&](const Eigen::Vector3d& point)
    {
        return point.x() >= workspace_min_.x() && point.x() <= workspace_max_.x() &&
               point.y() >= workspace_min_.y() && point.y() <= workspace_max_.y() &&
               point.z() >= workspace_min_.z() && point.z() <= workspace_max_.z();
    };
    if (!inside_workspace(line.center) || !inside_workspace(start) || !inside_workspace(end))
    {
        return fail(TrackerError::INVALID_LINE);
    }
    if (line.center.z() < safety_min_z_ + safety_margin_z_ ||
        start.z() < safety_min_z_ + safety_margin_z_ ||
        end.z() < safety_min_z_ + safety_margin_z_)
    {
        return fail(TrackerError::INVALID_LINE);
    }
    return true;
}

bool ContTracker::initializeReference(TrackerError* error)
{
    const auto pose = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    const auto velocity = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    const Eigen::Vector3d current = pose.translation();
    const double measured_s = active_line_.axis.dot(current - active_line_.center);
    const Eigen::Vector3d projected = active_line_.center + active_line_.axis * measured_s;
    const double cross_track = (current - projected).norm();

    if (!std::isfinite(measured_s) || !std::isfinite(cross_track))
    {
        if (error) { *error = TrackerError::START_FAILED; }
        return false;
    }
    if (std::abs(measured_s) > active_line_.half_length + 1e-6)
    {
        if (error) { *error = TrackerError::START_FAILED; }
        return false;
    }
    if (cross_track > max_start_cross_track_m_)
    {
        if (error) { *error = TrackerError::START_FAILED; }
        return false;
    }

    s_ref_ = measured_s;
    sdot_ref_ = clamp(active_line_.axis.dot(velocity.head<3>()), -goal_.v_max, goal_.v_max);
    return true;
}

bool ContTracker::readLatestTarget(
    double* target_s,
    std::int64_t* receipt_ns,
    std::uint64_t* sequence) const
{
    const std::uint64_t before = target_sequence_.load(std::memory_order_acquire);
    if ((before & 1U) != 0U)
    {
        return false;
    }
    const double target = target_s_atomic_.load(std::memory_order_relaxed);
    const std::int64_t receipt = target_receipt_ns_atomic_.load(std::memory_order_relaxed);
    const std::uint64_t after = target_sequence_.load(std::memory_order_acquire);
    if (before != after || (after & 1U) != 0U)
    {
        return false;
    }
    *target_s = target;
    *receipt_ns = receipt;
    *sequence = after;
    return std::isfinite(target) && receipt > 0;
}

void ContTracker::writeInitialTarget(double target_s, std::int64_t receipt_ns)
{
    target_sequence_.fetch_add(1, std::memory_order_acq_rel);
    target_s_atomic_.store(target_s, std::memory_order_relaxed);
    target_receipt_ns_atomic_.store(receipt_ns, std::memory_order_relaxed);
    target_sequence_.fetch_add(1, std::memory_order_release);
}

void ContTracker::beginTerminalBrake(TerminalMode mode, TrackerError error)
{
    if (terminal_mode_ != TerminalMode::NONE)
    {
        return;
    }
    terminal_mode_ = mode;
    stale_braking_ = false;
    stale_hold_active_ = false;
    terminal_brake_elapsed_sec_ = 0.0;
    session_accepts_updates_.store(false, std::memory_order_release);

    const double stopping_distance =
        sign(sdot_ref_) * sdot_ref_ * sdot_ref_ / (2.0 * std::max(goal_.a_max, 1e-6));
    braking_target_s_ = clamp(
        s_ref_ + stopping_distance,
        -active_line_.half_length,
        active_line_.half_length);

    if (mode == TerminalMode::ABORT_AFTER_BRAKE)
    {
        storeRtError(error);
    }
}

void ContTracker::beginRecoverableStaleBrake()
{
    stale_braking_ = true;
    stale_hold_active_ = false;
    const double stopping_distance =
        sign(sdot_ref_) * sdot_ref_ * sdot_ref_ / (2.0 * std::max(goal_.a_max, 1e-6));
    braking_target_s_ = clamp(
        s_ref_ + stopping_distance,
        -active_line_.half_length,
        active_line_.half_length);
}

void ContTracker::updateReference(double dt)
{
    const double target =
        (terminal_mode_ != TerminalMode::NONE || stale_braking_)
            ? braking_target_s_
            : accepted_target_s_;
    const double clamped_target = clamp(target, -active_line_.half_length, active_line_.half_length);
    const double error = clamped_target - s_ref_;

    double desired_velocity = 0.0;
    if (std::abs(error) > target_deadband_m_ || std::abs(sdot_ref_) > holding_velocity_mps_)
    {
        const double stopping_speed = std::sqrt(
            std::max(0.0, 2.0 * goal_.a_max * std::max(0.0, std::abs(error) - 0.5 * target_deadband_m_)));
        desired_velocity = sign(error) * std::min(goal_.v_max, stopping_speed);
    }

    const double old_velocity = sdot_ref_;
    const double max_delta_velocity = goal_.a_max * dt;
    sdot_ref_ += clamp(desired_velocity - sdot_ref_, -max_delta_velocity, max_delta_velocity);
    sdot_ref_ = clamp(sdot_ref_, -goal_.v_max, goal_.v_max);
    s_ref_ += 0.5 * (old_velocity + sdot_ref_) * dt;

    if (s_ref_ > active_line_.half_length)
    {
        s_ref_ = active_line_.half_length;
        sdot_ref_ = std::min(0.0, sdot_ref_);
    }
    else if (s_ref_ < -active_line_.half_length)
    {
        s_ref_ = -active_line_.half_length;
        sdot_ref_ = std::max(0.0, sdot_ref_);
    }

}

bool ContTracker::referenceIsHolding() const
{
    const double target =
        (terminal_mode_ != TerminalMode::NONE || stale_braking_)
            ? braking_target_s_
            : accepted_target_s_;
    return std::abs(target - s_ref_) <= target_deadband_m_ &&
           std::abs(sdot_ref_) <= holding_velocity_mps_;
}

bool ContTracker::computeCartesianCommand(
    const Eigen::Vector3d& desired_pos,
    const Eigen::Vector3d& desired_vel,
    double dt,
    bool include_motion_stats)
{
    if (ee_task_it_ == ee_data_.end())
    {
        storeRtError(TrackerError::TASK_DATA_MISSING);
        return false;
    }

    auto& task = ee_task_it_->second;
    task.x = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    task.xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    task.xddot.setZero();

    const Eigen::Vector3d current_pos = task.x.translation();
    const Eigen::Vector3d current_vel = task.xdot.head<3>();
    updateMotionStats(current_pos, current_vel, dt, include_motion_stats);

    const Eigen::Vector3d error = current_pos - desired_pos;
    const double tracking_error_pos = error.norm();
    const double tracking_error_z = std::abs(error.z());
    last_current_pos_ = current_pos;
    last_desired_pos_ = desired_pos;
    last_tracking_error_pos_ = tracking_error_pos;
    last_tracking_error_z_ = tracking_error_z;

    if (current_pos.z() < safety_min_z_)
    {
        storeRtError(TrackerError::CURRENT_Z_LOW);
        return false;
    }
    if (desired_pos.z() < safety_min_z_ + safety_margin_z_)
    {
        storeRtError(TrackerError::DESIRED_Z_LOW);
        return false;
    }
    if (tracking_error_pos > 2.0 * max_tracking_error_pos_ ||
        tracking_error_z > 2.0 * max_tracking_error_z_)
    {
        storeRtError(TrackerError::GROSS_TRACKING_ERROR);
        return false;
    }

    target_pose_.translation() = desired_pos;
    target_pose_.linear() = active_orientation_;
    Eigen::Vector3d clamped_pos = target_pose_.translation();
    clamped_pos.x() = clamp(clamped_pos.x(), workspace_min_.x(), workspace_max_.x());
    clamped_pos.y() = clamp(clamped_pos.y(), workspace_min_.y(), workspace_max_.y());
    clamped_pos.z() = clamp(clamped_pos.z(), workspace_min_.z(), workspace_max_.z());
    target_pose_.translation() = clamped_pos;

    Eigen::Vector6d target_velocity = Eigen::Vector6d::Zero();
    if (use_velocity_feedforward_)
    {
        target_velocity.head<3>() = desired_vel;
    }
    task.x_desired = target_pose_;
    task.xdot_desired = target_velocity;

    bool solved = true;
    switch (control_mode_)
    {
        case 0:
            fr3_model_updater_.robot_controller_->CLIKStep(
                ee_data_, fr3_model_updater_.qdot_desired_total_);
            fr3_model_updater_.q_desired_total_ =
                fr3_model_updater_.q_total_ +
                fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;
            fr3_model_updater_.torque_desired_total_ =
                fr3_model_updater_.robot_controller_->moveJointTorqueStep(
                    fr3_model_updater_.q_desired_total_,
                    fr3_model_updater_.qdot_desired_total_,
                    false);
            break;

        case 1:
            fr3_model_updater_.robot_controller_->OSFStep(
                ee_data_, fr3_model_updater_.torque_desired_total_);
            break;

        case 2:
            solved = fr3_model_updater_.robot_controller_->QPIKStep(
                ee_data_, fr3_model_updater_.qdot_desired_total_, false);
            if (!solved) { fr3_model_updater_.qdot_desired_total_.setZero(); }
            fr3_model_updater_.q_desired_total_ =
                fr3_model_updater_.q_total_ +
                fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;
            fr3_model_updater_.torque_desired_total_ =
                fr3_model_updater_.robot_controller_->moveJointTorqueStep(
                    fr3_model_updater_.q_desired_total_,
                    fr3_model_updater_.qdot_desired_total_,
                    false);
            break;

        case 3:
            solved = fr3_model_updater_.robot_controller_->QPIDStep(
                ee_data_, fr3_model_updater_.torque_desired_total_, false);
            if (!solved)
            {
                fr3_model_updater_.torque_desired_total_ =
                    fr3_model_updater_.robot_data_->getGravity();
            }
            break;

        default:
            solved = false;
            break;
    }

    if (control_mode_ == 2 || control_mode_ == 3)
    {
        consecutive_qp_failures_ = solved ? 0 : consecutive_qp_failures_ + 1;
        if (!solved && abort_on_repeated_qp_failure_ &&
            consecutive_qp_failures_ >= max_consecutive_qp_failures_)
        {
            storeRtError(TrackerError::REPEATED_QP_FAILURES);
            return false;
        }
    }
    else
    {
        consecutive_qp_failures_ = 0;
    }

    if (!solved)
    {
        storeRtError(TrackerError::CONTROLLER_STEP_FAILED);
        return false;
    }
    if (tracking_error_pos > max_tracking_error_pos_)
    {
        storeRtError(TrackerError::TRACKING_ERROR_POS);
        return false;
    }
    if (tracking_error_z > max_tracking_error_z_)
    {
        storeRtError(TrackerError::TRACKING_ERROR_Z);
        return false;
    }

    first_command_written_ = true;
    fr3_model_updater_.writeCommand(
        fr3_model_updater_.torque_desired_total_ - fr3_model_updater_.g_total_);
    return true;
}

Eigen::Matrix3d ContTracker::makeBaseOrientationFromParams() const
{
    if (base_orientation_rpy_deg_.size() != 3)
    {
        return Eigen::Matrix3d::Identity();
    }
    return rpyDegToRot(
        base_orientation_rpy_deg_[0],
        base_orientation_rpy_deg_[1],
        base_orientation_rpy_deg_[2]);
}

Eigen::Matrix3d ContTracker::resolveActiveOrientation(const Eigen::Affine3d& current_pose) const
{
    if (target_orientation_mode_ == "base_rpy")
    {
        return makeBaseOrientationFromParams();
    }
    return current_pose.linear();
}

void ContTracker::publishTrackS(double target_s)
{
    std_msgs::msg::Float64 msg;
    msg.data = target_s;
    track_s_pub_->publish(msg);
}

void ContTracker::publishAcceptedTarget(const RtTelemetrySnapshot& snapshot)
{
    std_msgs::msg::Float64 scalar;
    scalar.data = snapshot.accepted_target_s;
    accepted_target_s_pub_->publish(scalar);

    geometry_msgs::msg::PointStamped stamped;
    stamped.header.stamp = node_->now();
    stamped.header.frame_id = active_line_.frame_id.data();
    stamped.point.x = snapshot.accepted_target_point.x();
    stamped.point.y = snapshot.accepted_target_point.y();
    stamped.point.z = snapshot.accepted_target_point.z();
    accepted_target_base_pub_->publish(stamped);
}

void ContTracker::onStop(StopReason reason)
{
    session_accepts_updates_.store(false, std::memory_order_release);
    active_target_bound_m_.store(0.0, std::memory_order_relaxed);
    stop_in_progress_.store(true, std::memory_order_release);

    model_updater_.haltCommands();

    LastStopReason final_reason = LastStopReason::NONE;
    if (reason == StopReason::SUCCEEDED) { final_reason = LastStopReason::SUCCEEDED; }
    else if (reason == StopReason::CANCELED) { final_reason = LastStopReason::CANCELED; }
    else { final_reason = LastStopReason::ABORTED; }

    executor_state_.store(
        reason == StopReason::ABORTED ? ExecutorState::ABORTED : ExecutorState::STOPPED,
        std::memory_order_release);
    last_stop_reason_.store(final_reason, std::memory_order_release);
    session_active_.store(false, std::memory_order_release);
    stop_in_progress_.store(false, std::memory_order_release);

    if (gate_acquired_)
    {
        const auto release_result = motion_gate_->releaseRT(MotionGate::Owner::CONT_TRACKER);
        if (release_result == MotionGate::ReleaseResult::RELEASED)
        {
            gate_acquired_ = false;
            RCLCPP_INFO(node_->get_logger(), "[%s] gate released on stop", name_.c_str());
        }
        else
        {
            gate_acquired_ = motion_gate_->owner() == MotionGate::Owner::CONT_TRACKER;
            RCLCPP_WARN(node_->get_logger(), "[%s] gate release on stop ignored; current owner=%s", name_.c_str(), MotionGate::ownerToString(motion_gate_->owner()).c_str());
        }
    }

    writeTelemetrySnapshot(false);
}

ContTracker::ResultPtr ContTracker::makeResult(StopReason reason)
{
    auto result = std::make_shared<ActionT::Result>();
    const std::string stats = formatMotionStatsSummary();
    if (reason == StopReason::SUCCEEDED)
    {
        result->success = true;
        result->message = "continuous tracking session completed | " + stats;
        result->final_state = 0;
    }
    else if (reason == StopReason::CANCELED)
    {
        result->success = false;
        result->message = "continuous tracking session canceled | " + stats;
        result->final_state = 2;
    }
    else
    {
        result->success = false;
        result->message = std::string("continuous tracking aborted: ") + errorToString(rt_error_) + " | " + stats;
        result->final_state = 1;
    }
    return result;
}

void ContTracker::resetMotionStats()
{
    motion_stats_ = MotionStats{};
}

void ContTracker::updateMotionStats(
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
    if (include_sample)
    {
        const double speed = current_vel.norm();
        const double acceleration = ((current_vel - motion_stats_.last_vel) / dt).norm();
        motion_stats_.moving_time += dt;
        motion_stats_.path_length += (current_pos - motion_stats_.last_pos).norm();
        motion_stats_.speed_integral += speed * dt;
        motion_stats_.acc_integral += acceleration * dt;
        motion_stats_.peak_speed = std::max(motion_stats_.peak_speed, speed);
        motion_stats_.peak_acc = std::max(motion_stats_.peak_acc, acceleration);
    }
    motion_stats_.last_pos = current_pos;
    motion_stats_.last_vel = current_vel;
}

std::string ContTracker::formatMotionStatsSummary() const
{
    if (motion_stats_.moving_time <= 1e-9)
    {
        return "motion_stats: no moving samples";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "motion_stats: moving_time=" << motion_stats_.moving_time << "s"
           << " path=" << motion_stats_.path_length << "m"
           << " avg_speed=" << motion_stats_.speed_integral / motion_stats_.moving_time << "m/s"
           << " peak_speed=" << motion_stats_.peak_speed << "m/s"
           << " avg_acc=" << motion_stats_.acc_integral / motion_stats_.moving_time << "m/s^2"
           << " peak_acc=" << motion_stats_.peak_acc << "m/s^2";
    return stream.str();
}

const char* ContTracker::phaseToString(TrackerPhase phase) const
{
    switch (phase)
    {
        case TrackerPhase::STOPPED: return "stopped";
        case TrackerPhase::ACCEPTED: return "accepted";
        case TrackerPhase::TRACKING: return "tracking";
        case TrackerPhase::HOLDING: return "holding";
        case TrackerPhase::TRACKING_LAST_STALE: return "tracking_last_stale";
        case TrackerPhase::HOLDING_LAST_STALE: return "holding_last_stale";
        case TrackerPhase::BRAKING_STALE: return "braking_stale";
        case TrackerPhase::BRAKING_TIMEOUT: return "braking_timeout";
        case TrackerPhase::TERMINAL_BRAKE: return "terminal_brake";
        default: return "unknown";
    }
}

const char* ContTracker::errorToString(TrackerError error) const
{
    switch (error)
    {
        case TrackerError::NONE: return "none";
        case TrackerError::START_FAILED: return "start_failed";
        case TrackerError::INVALID_DT: return "invalid_dt";
        case TrackerError::SESSION_TIMEOUT: return "session_timeout";
        case TrackerError::STALE_TARGET_ABORT: return "stale_target_abort";
        case TrackerError::BRAKE_TIMEOUT: return "brake_timeout";
        case TrackerError::TASK_DATA_MISSING: return "task_data_missing";
        case TrackerError::CURRENT_Z_LOW: return "current_z_low";
        case TrackerError::DESIRED_Z_LOW: return "desired_z_low";
        case TrackerError::GROSS_TRACKING_ERROR: return "gross_tracking_error";
        case TrackerError::TRACKING_ERROR_POS: return "tracking_error_pos";
        case TrackerError::TRACKING_ERROR_Z: return "tracking_error_z";
        case TrackerError::CONTROLLER_STEP_FAILED: return "controller_step_failed";
        case TrackerError::REPEATED_QP_FAILURES: return "repeated_qp_failures";
        case TrackerError::INVALID_LINE: return "invalid_line";
        default: return "unknown";
    }
}

const char* ContTracker::staleBehaviorToString(StaleTargetBehavior behavior) const
{
    switch (behavior)
    {
        case StaleTargetBehavior::HOLD_LAST: return "hold_last";
        case StaleTargetBehavior::BRAKE: return "brake";
        case StaleTargetBehavior::ABORT: return "abort";
        default: return "unknown";
    }
}

void ContTracker::storeRtError(TrackerError error)
{
    if (rt_error_ == TrackerError::NONE)
    {
        rt_error_ = error;
    }
}

void ContTracker::writeTelemetrySnapshot(bool accepted_target_changed)
{
    RtTelemetrySnapshot snapshot;
    snapshot.sequence = ++telemetry_snapshot_sequence_;
    snapshot.session_active = session_active_.load(std::memory_order_acquire);
    snapshot.phase = static_cast<std::uint8_t>(phase_);
    snapshot.terminal_mode = static_cast<std::uint8_t>(terminal_mode_);
    snapshot.error = static_cast<std::uint8_t>(rt_error_);
    snapshot.stale_behavior = static_cast<std::uint8_t>(stale_target_behavior_);
    snapshot.line_revision = active_line_.revision;
    snapshot.accepted_target_s = accepted_target_s_;
    snapshot.accepted_target_point = active_line_.center + active_line_.axis * accepted_target_s_;
    snapshot.s_ref = s_ref_;
    snapshot.sdot_ref = sdot_ref_;
    snapshot.target_age_sec = target_age_sec_;
    snapshot.session_elapsed_sec = session_elapsed_sec_;
    snapshot.tracking_error_pos = last_tracking_error_pos_;
    snapshot.tracking_error_z = last_tracking_error_z_;
    snapshot.peak_speed = motion_stats_.peak_speed;
    snapshot.peak_acc = motion_stats_.peak_acc;
    snapshot.moving_time = motion_stats_.moving_time;
    snapshot.accepted_target_changed = accepted_target_changed;

    const std::uint64_t before = telemetry_sequence_.load(std::memory_order_relaxed);
    telemetry_sequence_.store(before + 1, std::memory_order_release);
    if (((before / 2U) % 2U) == 0U)
    {
        telemetry_slot_b_ = snapshot;
    }
    else
    {
        telemetry_slot_a_ = snapshot;
    }
    telemetry_sequence_.store(before + 2, std::memory_order_release);
}

bool ContTracker::readTelemetrySnapshot(RtTelemetrySnapshot* snapshot) const
{
    if (!snapshot)
    {
        return false;
    }
    const std::uint64_t before = telemetry_sequence_.load(std::memory_order_acquire);
    if ((before & 1U) != 0U)
    {
        return false;
    }
    const RtTelemetrySnapshot copy = (((before / 2U) % 2U) == 0U) ? telemetry_slot_a_ : telemetry_slot_b_;
    const std::uint64_t after = telemetry_sequence_.load(std::memory_order_acquire);
    if (before != after || (after & 1U) != 0U)
    {
        return false;
    }
    *snapshot = copy;
    return true;
}

void ContTracker::telemetryTimerCallback()
{
    RtTelemetrySnapshot snapshot;
    if (!readTelemetrySnapshot(&snapshot))
    {
        return;
    }

    if (snapshot.sequence != 0 && snapshot.sequence != last_published_accepted_target_sequence_ &&
        snapshot.accepted_target_changed)
    {
        publishAcceptedTarget(snapshot);
        last_published_accepted_target_sequence_ = snapshot.sequence;
    }

    const bool active = snapshot.session_active;
    if (active)
    {
        auto feedback = std::make_shared<ActionT::Feedback>();
        feedback->phase = phaseToString(static_cast<TrackerPhase>(snapshot.phase));
        feedback->progress = std::clamp(snapshot.session_elapsed_sec / max_session_sec_, 0.0, 1.0);
        feedback->s_des = snapshot.s_ref;
        feedback->sdot_des = snapshot.sdot_ref;
        feedback->tracking_error_pos = snapshot.tracking_error_pos;
        feedback->tracking_error_z = snapshot.tracking_error_z;
        publishFeedback(feedback);
    }

    if (active && !telemetry_prev_session_active_)
    {
        last_start_time_ = node_->now();
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] started line_revision=%lu stale_policy=%s",
            name_.c_str(),
            static_cast<unsigned long>(snapshot.line_revision),
            staleBehaviorToString(stale_target_behavior_));
    }

    if (!active && telemetry_prev_session_active_)
    {
        last_stop_time_ = node_->now();
        RCLCPP_INFO(
            node_->get_logger(),
            "[%s] stopped last_stop_reason=%s rt_error=%s | %s",
            name_.c_str(),
            lastStopReasonToString(last_stop_reason_.load(std::memory_order_acquire)).c_str(),
            errorToString(static_cast<TrackerError>(snapshot.error)),
            formatMotionStatsSummary().c_str());
    }

    telemetry_prev_session_active_ = active;

}

std::string ContTracker::buildStatusMessage() const
{
    RtTelemetrySnapshot snapshot;
    if (!readTelemetrySnapshot(&snapshot))
    {
        return "status unavailable";
    }

    std::ostringstream stream;
    stream << "state=" << executorStateToString(executor_state_.load(std::memory_order_acquire))
           << " last_stop_reason=" << lastStopReasonToString(last_stop_reason_.load(std::memory_order_acquire))
           << " phase=" << phaseToString(static_cast<TrackerPhase>(snapshot.phase))
           << " s_ref=" << snapshot.s_ref
           << " sdot_ref=" << snapshot.sdot_ref
           << " accepted_target_s=" << snapshot.accepted_target_s
           << " target_age=" << snapshot.target_age_sec
           << " tracking_error_pos=" << snapshot.tracking_error_pos
           << " tracking_error_z=" << snapshot.tracking_error_z
           << " line_revision=" << snapshot.line_revision
           << " stale_policy=" << staleBehaviorToString(stale_target_behavior_)
           << " rt_error=" << errorToString(static_cast<TrackerError>(snapshot.error));
    return stream.str();
}

std::string ContTracker::executorStateToString(ExecutorState state) const
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

std::string ContTracker::lastStopReasonToString(LastStopReason reason) const
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

void ContTracker::handleGetStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    response->success = true;
    response->message = buildStatusMessage();
}

REGISTER_FR3_ACTION_SERVER(ContTracker, "cont_tracker")

}  // namespace fr3_husky_controller::servers::fr3
