#pragma once

#include <fr3_husky_controller/model/fr3_model_updater.hpp>
#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/servers/motion_gate.hpp>

#include <fr3_husky_msgs/action/line_trajectory.hpp>
#include <fr3_husky_msgs/msg/middle_line.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace fr3_husky_controller::servers::fr3
{

class ContTracker final : public ActionServerBase<fr3_husky_msgs::action::LineTrajectory>
{
public:
    using ActionT = fr3_husky_msgs::action::LineTrajectory;
    using Base = ActionServerBase<ActionT>;
    using NodePtr = typename Base::NodePtr;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason = typename Base::StopReason;
    using ResultPtr = typename Base::ResultPtr;

    ContTracker(
        const std::string& name,
        const NodePtr& node,
        ModelUpdaterBase& model_updater);

protected:
    bool acceptGoal(const ActionT::Goal& goal) override;
    void onGoalAccepted(const ActionT::Goal& goal) override;
    void onStart() override;
    ComputeResult compute(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;
    void onStop(StopReason reason) override;
    ResultPtr makeResult(StopReason reason) override;

private:
    enum class TrackerPhase : std::uint8_t
    {
        STOPPED = 0,
        ACCEPTED,
        TRACKING,
        HOLDING,
        TRACKING_LAST_STALE,
        HOLDING_LAST_STALE,
        BRAKING_STALE,
        BRAKING_TIMEOUT,
        TERMINAL_BRAKE,
    };

    enum class TrackerError : std::uint8_t
    {
        NONE = 0,
        START_FAILED,
        INVALID_DT,
        SESSION_TIMEOUT,
        STALE_TARGET_ABORT,
        BRAKE_TIMEOUT,
        TASK_DATA_MISSING,
        CURRENT_Z_LOW,
        DESIRED_Z_LOW,
        GROSS_TRACKING_ERROR,
        TRACKING_ERROR_POS,
        TRACKING_ERROR_Z,
        CONTROLLER_STEP_FAILED,
        REPEATED_QP_FAILURES,
        INVALID_LINE,
    };

    enum class StaleTargetBehavior : std::uint8_t
    {
        HOLD_LAST = 0,
        BRAKE,
        ABORT,
    };

    enum class ExecutorState
    {
        STOPPED,
        ACTIVE,
        CANCELING,
        ABORTED
    };

    enum class LastStopReason
    {
        NONE,
        SUCCEEDED,
        CANCELED,
        ABORTED
    };

    enum class TerminalMode
    {
        NONE,
        SUCCEED_AFTER_BRAKE,
        ABORT_AFTER_BRAKE
    };

    struct LineSnapshot
    {
        Eigen::Vector3d center{Eigen::Vector3d::Zero()};
        Eigen::Vector3d axis{Eigen::Vector3d::UnitX()};
        double half_length{0.0};
        std::array<char, 64> frame_id{};
        std::array<char, 64> ee_name{};
        std::uint64_t revision{0};
        bool valid{false};
    };

    struct RtTelemetrySnapshot
    {
        std::uint64_t sequence{0};
        bool session_active{false};
        std::uint8_t phase{0};
        std::uint8_t terminal_mode{0};
        std::uint8_t error{0};
        std::uint8_t stale_behavior{0};
        std::uint64_t line_revision{0};
        double accepted_target_s{0.0};
        Eigen::Vector3d accepted_target_point{Eigen::Vector3d::Zero()};
        double s_ref{0.0};
        double sdot_ref{0.0};
        double target_age_sec{0.0};
        double session_elapsed_sec{0.0};
        double tracking_error_pos{0.0};
        double tracking_error_z{0.0};
        double peak_speed{0.0};
        double peak_acc{0.0};
        double moving_time{0.0};
        bool accepted_target_changed{false};
    };

    struct MotionStats
    {
        bool initialized{false};
        Eigen::Vector3d last_pos{Eigen::Vector3d::Zero()};
        Eigen::Vector3d last_vel{Eigen::Vector3d::Zero()};
        double moving_time{0.0};
        double path_length{0.0};
        double speed_integral{0.0};
        double acc_integral{0.0};
        double peak_speed{0.0};
        double peak_acc{0.0};
    };

    void onMiddleLine(const fr3_husky_msgs::msg::MiddleLine::SharedPtr msg);
    void onTargetS(const std_msgs::msg::Float64::SharedPtr msg);

    bool snapshotAndValidateLine(TrackerError* error);
    bool readLatestLine(LineSnapshot* line) const;
    bool validateLineGeometry(const LineSnapshot& line, TrackerError* error) const;
    bool initializeReference(TrackerError* error);
    bool readLatestTarget(double* target_s, std::int64_t* receipt_ns, std::uint64_t* sequence) const;
    void writeInitialTarget(double target_s, std::int64_t receipt_ns);
    void beginTerminalBrake(TerminalMode mode, TrackerError error);
    void beginRecoverableStaleBrake();
    void updateReference(double dt);
    bool referenceIsHolding() const;
    void writeTelemetrySnapshot(bool accepted_target_changed);
    bool readTelemetrySnapshot(RtTelemetrySnapshot* snapshot) const;
    void telemetryTimerCallback();

    bool computeCartesianCommand(
        const Eigen::Vector3d& desired_pos,
        const Eigen::Vector3d& desired_vel,
        double dt,
        bool include_motion_stats);

    Eigen::Matrix3d makeBaseOrientationFromParams() const;
    Eigen::Matrix3d resolveActiveOrientation(const Eigen::Affine3d& current_pose) const;

    void publishTrackS(double target_s);
    void publishAcceptedTarget(const RtTelemetrySnapshot& snapshot);
    void resetMotionStats();
    void updateMotionStats(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_vel,
        double dt,
        bool include_sample);
    std::string formatMotionStatsSummary() const;
    const char* phaseToString(TrackerPhase phase) const;
    const char* errorToString(TrackerError error) const;
    const char* staleBehaviorToString(StaleTargetBehavior behavior) const;
    void storeRtError(TrackerError error);
    std::string buildStatusMessage() const;
    std::string executorStateToString(ExecutorState state) const;
    std::string lastStopReasonToString(LastStopReason reason) const;
    void handleGetStatus(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    FR3ModelUpdater& fr3_model_updater_;
    std::shared_ptr<MotionGate> motion_gate_;
    bool gate_acquired_{false};

    ActionT::Goal goal_{};
    std::string ee_name_default_;
    std::string control_ee_name_;

    // Lock-free line snapshot exchange for the RT path.
    std::atomic<std::uint64_t> line_sequence_{0};
    LineSnapshot line_slot_a_;
    LineSnapshot line_slot_b_;
    LineSnapshot active_line_;

    // Latest-value target hand-off. target_sequence_ is a seqlock: even values
    // are stable, odd values mean the callback is writing. The control loop
    // never blocks or spins waiting for a callback.
    std::atomic<bool> session_accepts_updates_{false};
    std::atomic<double> active_target_bound_m_{0.0};
    mutable std::atomic<std::uint64_t> target_sequence_{0};
    std::atomic<double> target_s_atomic_{0.0};
    std::atomic<std::int64_t> target_receipt_ns_atomic_{0};
    std::uint64_t consumed_target_sequence_{0};
    std::uint64_t telemetry_snapshot_sequence_{0};

    rclcpp::Subscription<fr3_husky_msgs::msg::MiddleLine>::SharedPtr middle_line_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_s_sub_;
    // Accepted target-s command stream for recording and offline tracking analysis.
    // Includes the initial action goal and every valid/clamped target_s update.
    // Publication occurs only from non-RT callbacks.
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr track_s_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accepted_target_s_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr accepted_target_base_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_srv_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;

    std::map<std::string, drc::TaskSpaceData> ee_data_;
    std::map<std::string, drc::TaskSpaceData>::iterator ee_task_it_;
    Eigen::Affine3d target_pose_{Eigen::Affine3d::Identity()};
    Eigen::Matrix3d active_orientation_{Eigen::Matrix3d::Identity()};

    double s_ref_{0.0};
    double sdot_ref_{0.0};
    double accepted_target_s_{0.0};
    double braking_target_s_{0.0};
    bool stale_braking_{false};
    bool stale_hold_active_{false};
    TerminalMode terminal_mode_{TerminalMode::NONE};
    double terminal_brake_elapsed_sec_{0.0};

    double session_elapsed_sec_{0.0};
    double target_age_sec_{0.0};

    bool start_failed_{false};
    TrackerError start_error_{TrackerError::NONE};
    TrackerError rt_error_{TrackerError::NONE};
    TrackerPhase phase_{TrackerPhase::STOPPED};
    bool first_command_written_{false};

    Eigen::Vector3d last_current_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d last_desired_pos_{Eigen::Vector3d::Zero()};
    double last_tracking_error_pos_{0.0};
    double last_tracking_error_z_{0.0};
    int consecutive_qp_failures_{0};

    std::atomic<ExecutorState> executor_state_{ExecutorState::STOPPED};
    std::atomic<LastStopReason> last_stop_reason_{LastStopReason::NONE};
    std::atomic<bool> stop_in_progress_{false};
    std::atomic<bool> session_active_{false};
    rclcpp::Time last_start_time_;
    rclcpp::Time last_stop_time_;

    std::atomic<std::uint64_t> telemetry_sequence_{0};
    RtTelemetrySnapshot telemetry_slot_a_;
    RtTelemetrySnapshot telemetry_slot_b_;
    std::uint64_t last_published_accepted_target_sequence_{0};
    bool telemetry_prev_session_active_{false};

    MotionStats motion_stats_;

    // Parameters. v_max/a_max can be overridden per action goal, within hard
    // bounds.
    double default_v_max_{0.40};
    double default_a_max_{1.00};
    double hard_v_max_{1.90};
    double hard_a_max_{5.00};
    double safety_min_z_{0.08};
    double hard_min_safety_z_{0.05};
    double safety_margin_z_{0.005};
    double max_tracking_error_pos_{0.02};
    double max_tracking_error_z_{0.005};
    double max_start_cross_track_m_{0.015};
    double target_timeout_sec_{0.25};
    double max_session_sec_{5.0};
    double terminal_brake_timeout_sec_{1.0};
    double target_deadband_m_{0.002};
    double holding_velocity_mps_{0.005};
    double max_control_period_sec_{0.02};
    double min_line_half_length_{0.005};
    double max_line_half_length_{0.40};
    double feedback_period_sec_{0.02};
    bool abort_on_stale_target_{true};
    StaleTargetBehavior stale_target_behavior_{StaleTargetBehavior::HOLD_LAST};
    bool use_velocity_feedforward_{true};
    bool allow_clamped_targets_{false};
    int control_mode_{0};
    bool abort_on_repeated_qp_failure_{false};
    int max_consecutive_qp_failures_{20};

    Eigen::Vector3d workspace_min_{0.05, -0.4, 0.05};
    Eigen::Vector3d workspace_max_{0.95, 0.4, 0.65};
    std::string target_orientation_mode_{"captured"};
    std::vector<double> base_orientation_rpy_deg_{0.0, 0.0, 0.0};
};

}  // namespace fr3_husky_controller::servers::fr3