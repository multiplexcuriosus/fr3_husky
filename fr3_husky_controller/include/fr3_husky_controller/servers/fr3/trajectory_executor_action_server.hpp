#pragma once

#include <fr3_husky_controller/model/fr3_model_updater.hpp>
#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/servers/motion_gate.hpp>

#include <fr3_husky_msgs/action/line_trajectory.hpp>
#include <fr3_husky_msgs/srv/capture_line_center.hpp>
#include <fr3_husky_msgs/srv/project_point_to_line.hpp>
#include <fr3_husky_msgs/srv/set_line_center.hpp>
#include <fr3_husky_msgs/srv/set_line_params.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace fr3_husky_controller::servers::fr3
{

class TrajectoryExecutor final : public ActionServerBase<fr3_husky_msgs::action::LineTrajectory>
{
public:
    using ActionT = fr3_husky_msgs::action::LineTrajectory;
    using Base = ActionServerBase<ActionT>;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason = typename Base::StopReason;
    using ResultPtr = typename Base::ResultPtr;

    enum class ExecutorState
    {
        STOPPED,
        ACTIVE,
        CANCELING,
        ABORTED,
    };

    enum class LastStopReason
    {
        NONE,
        SUCCEEDED,
        CANCELED,
        ABORTED,
    };

    explicit TrajectoryExecutor(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater);

    bool acceptGoal(const ActionT::Goal& goal) override;
    void onGoalAccepted(const ActionT::Goal& goal) override;
    void onStart() override;
    ComputeResult compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void onStop(StopReason reason) override;
    ResultPtr makeResult(StopReason reason) override;

    int priority() const override { return 8; }
    bool allowPreemption() const override { return false; }

private:
    struct TrajSegment
    {
        std::string phase;
        Eigen::Vector3d p0{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p1{Eigen::Vector3d::Zero()};
        Eigen::Vector3d dir{Eigen::Vector3d::UnitX()};
        double length{0.0};
        double v_max{0.0};
        double a_max{0.0};
        double t_acc{0.0};
        double t_flat{0.0};
        double t_total{0.0};
        bool hold_only{false};
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

        std::size_t speed_samples{0};
        std::size_t acc_samples{0};
    };

    bool checkCartesianExecutorSafe(std::string* reason);
    bool buildSegments(std::string* reason);
    TrajSegment makeSegment(
        const std::string& phase,
        const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1,
        double v_max,
        double a_max) const;
    TrajSegment makeHold(const std::string& phase, const Eigen::Vector3d& p, double hold_sec) const;

    Eigen::Matrix3d makeBaseOrientationFromParams() const;
    Eigen::Matrix3d resolveActiveOrientation(const Eigen::Affine3d& current_pose) const;
    Eigen::Vector3d linePoint(double s) const;
    double commandTargetS(uint8_t command) const;
    void refreshTrajectoryLimitParamsFromServer(bool log_changes);
    void resetMotionStats();
    void updateMotionStats(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_vel,
        double dt,
        bool include_sample);
    std::string formatMotionStatsSummary() const;

    bool computeCartesianCommand(
        const Eigen::Vector3d& desired_pos,
        const Eigen::Vector3d& desired_vel,
        double dt,
        bool include_motion_stats);
    void updateComputeTimingDebug(std::chrono::steady_clock::time_point start_time);

    void handleGetStatus(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleCaptureLineCenter(
        const std::shared_ptr<fr3_husky_msgs::srv::CaptureLineCenter::Request> request,
        std::shared_ptr<fr3_husky_msgs::srv::CaptureLineCenter::Response> response);

    void handleSetLineCenter(
        const std::shared_ptr<fr3_husky_msgs::srv::SetLineCenter::Request> request,
        std::shared_ptr<fr3_husky_msgs::srv::SetLineCenter::Response> response);

    void handleSetLineParams(
        const std::shared_ptr<fr3_husky_msgs::srv::SetLineParams::Request> request,
        std::shared_ptr<fr3_husky_msgs::srv::SetLineParams::Response> response);

    void handleProjectPointToLine(
        const std::shared_ptr<fr3_husky_msgs::srv::ProjectPointToLine::Request> request,
        std::shared_ptr<fr3_husky_msgs::srv::ProjectPointToLine::Response> response);

    bool geometryMutationBlocked() const;
    bool validateLineGeometry(
        const Eigen::Vector3d& center,
        const Eigen::Vector3d& axis,
        double half_length,
        std::string* reason) const;
    std::string commandToString(uint8_t command) const;
    std::string makeAbortDetail(
        const std::string& reason,
        const std::string& phase,
        const Eigen::Vector3d& current,
        const Eigen::Vector3d& desired,
        double tracking_error_pos,
        double tracking_error_z) const;

    std::string buildStatusMessage() const;
    std::string executorStateToString(ExecutorState state) const;
    std::string lastStopReasonToString(LastStopReason reason) const;

    FR3ModelUpdater& fr3_model_updater_;
    std::shared_ptr<MotionGate> motion_gate_;
    bool gate_acquired_{false};
    bool start_failed_{false};
    std::string start_failure_reason_;

    std::mutex line_mutex_;
    Eigen::Affine3d line_center_pose_{Eigen::Affine3d::Identity()};
    Eigen::Vector3d line_axis_{Eigen::Vector3d::UnitX()};
    double line_half_length_{0.20};

    Eigen::Vector3d active_center_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d active_axis_{Eigen::Vector3d::UnitX()};
    double active_half_length_{0.20};
    Eigen::Matrix3d active_orientation_{Eigen::Matrix3d::Identity()};

    bool have_captured_orientation_{false};
    bool jerk_unused_logged_{false};

    std::string ee_name_default_;
    std::string line_frame_;
    int control_mode_{0};
    std::string control_ee_name_;

    // Orientation policy:
    //   base_rpy  -> fixed orientation in base frame from base_orientation_rpy_deg
    //   captured  -> orientation stored by capture/set_line_center
    //   current   -> debug mode; preserve current measured orientation every tick
    std::string target_orientation_mode_{"base_rpy"};
    std::vector<double> base_orientation_rpy_deg_{0.0, 0.0, 0.0};
    bool use_velocity_feedforward_{false};

    double hard_v_max_{1.9};
    double hard_a_max_{5.0};
    double hard_j_max_{30.0};
    double hard_min_safety_z_{0.05};
    double min_line_half_length_{0.005};
    double max_line_half_length_{0.40};
    double safety_margin_z_{0.005};
    bool reject_line_services_while_active_{true};
    bool allow_service_update_safety_min_z_{false};

    double safety_min_z_{0.08};
    double max_tracking_error_pos_{0.02};
    double max_tracking_error_z_{0.005};
    double default_v_max_slow_{0.10};
    double default_a_max_slow_{0.30};
    double default_j_max_slow_{10.0};
    double default_v_max_fast_{0.40};
    double default_a_max_fast_{1.00};
    double default_j_max_fast_{30.0};
    bool require_cartesian_stopped_{false};
    bool require_cartesian_status_service_check_{false};

    Eigen::Vector3d workspace_min_{Eigen::Vector3d(0.20, -0.45, 0.05)};
    Eigen::Vector3d workspace_max_{Eigen::Vector3d(0.75, 0.45, 0.65)};

    bool abort_on_repeated_qp_failure_{false};
    int max_consecutive_qp_failures_{20};
    int consecutive_qp_failures_{0};

    ActionT::Goal goal_{};
    std::vector<TrajSegment> segments_;
    std::size_t segment_index_{0};
    double segment_time_{0.0};
    double overall_progress_{0.0};
    MotionStats motion_stats_;

    std::string last_error_message_;
    std::string last_phase_;
    Eigen::Vector3d last_current_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d last_desired_pos_{Eigen::Vector3d::Zero()};
    double last_tracking_error_pos_{0.0};
    double last_tracking_error_z_{0.0};
    double last_s_des_{0.0};
    double last_sdot_des_{0.0};

    std::map<std::string, drc::TaskSpaceData> ee_data_;
    Eigen::Affine3d target_pose_{Eigen::Affine3d::Identity()};
    Eigen::Matrix3d target_rotation_{Eigen::Matrix3d::Identity()};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_srv_;
    rclcpp::Service<fr3_husky_msgs::srv::CaptureLineCenter>::SharedPtr capture_center_srv_;
    rclcpp::Service<fr3_husky_msgs::srv::SetLineCenter>::SharedPtr set_center_srv_;
    rclcpp::Service<fr3_husky_msgs::srv::SetLineParams>::SharedPtr set_params_srv_;
    rclcpp::Service<fr3_husky_msgs::srv::ProjectPointToLine>::SharedPtr project_point_srv_;
    rclcpp::TimerBase::SharedPtr trajectory_limit_refresh_timer_;

    mutable std::mutex status_mutex_;
    ExecutorState executor_state_{ExecutorState::STOPPED};
    rclcpp::Time last_start_time_;
    rclcpp::Time last_stop_time_;
    LastStopReason last_stop_reason_{LastStopReason::NONE};
    bool stop_in_progress_{false};

    mutable std::mutex trajectory_limits_mutex_;

    double feedback_period_sec_{0.02};
    rclcpp::Time last_feedback_time_;
    bool first_command_written_{false};

    bool enable_compute_timing_debug_{false};
    uint64_t timing_call_count_{0};
    uint64_t timing_overrun_800us_{0};
    uint64_t timing_overrun_1000us_{0};
    double timing_max_compute_us_{0.0};
    std::chrono::steady_clock::time_point timing_last_report_time_;
};

}  // namespace fr3_husky_controller::servers::fr3