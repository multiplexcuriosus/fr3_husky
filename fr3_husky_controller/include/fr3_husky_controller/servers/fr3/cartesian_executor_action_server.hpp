#pragma once

#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/model/fr3_model_updater.hpp>
#include <fr3_husky_msgs/action/omega_haptic.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <map>
#include <mutex>
#include <string>

namespace fr3_husky_controller::servers::fr3
{

class CartesianExecutor : public ActionServerBase<fr3_husky_msgs::action::OmegaHaptic>
{
public:
    using Base = ActionServerBase<fr3_husky_msgs::action::OmegaHaptic>;
    using ActionT = fr3_husky_msgs::action::OmegaHaptic;
    using ResultPtr = typename Base::ResultPtr;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason = typename Base::StopReason;

    enum class ExecutorState
    {
        STOPPED,
        ACTIVE,
        CANCELING,
        ABORTED
    };

    CartesianExecutor(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater);

    bool acceptGoal(const ActionT::Goal& goal) override;
    void onGoalAccepted(const ActionT::Goal& goal) override;
    void onStart() override;
    ComputeResult compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void onStop(StopReason reason) override;
    ResultPtr makeResult(StopReason reason) override;

private:
    void publishStatus(bool log_status = false);
    std::string buildStatusMessage() const;
    std::string executorStateToString(ExecutorState state) const;
    int64_t getLastCommandAgeMs(const rclcpp::Time& now) const;

    void updateComputeTimingDebug(std::chrono::steady_clock::time_point start_time);

    void subTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    void handleResetTarget(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleGetStatus(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    FR3ModelUpdater& fr3_model_updater_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_target_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_srv_;

    std::mutex cmd_mutex_;
    Eigen::Vector3d latest_lin_vel_cmd_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d latest_ang_vel_cmd_{Eigen::Vector3d::Zero()};
    rclcpp::Time latest_cmd_stamp_{0, 0, RCL_ROS_TIME};
    bool have_twist_cmd_{false};

    std::map<std::string, drc::TaskSpaceData> ee_data;

    int control_mode_{0};
    std::string control_ee_name_;
    bool move_ori_{false};

    double haptic_pos_multiplier_{1.0};
    double haptic_ori_multiplier_{1.0};
    double haptic_lin_vel_multiplier_{1.0};
    double haptic_ang_vel_multiplier_{1.0};

    bool reset_target_requested_{false};

    double vel_lpf_tau_{0.03};
    double cmd_timeout_sec_{0.20};
    std::string linear_twist_frame_;
    std::string angular_twist_frame_;

    Eigen::Vector3d filtered_lin_vel_cmd_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d filtered_ang_vel_cmd_{Eigen::Vector3d::Zero()};
    Eigen::Affine3d target_pose_{Eigen::Affine3d::Identity()};
    Eigen::Matrix3d target_rotation_{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d workspace_min_;
    Eigen::Vector3d workspace_max_;

    std::string twist_topic_name_;
    std::string reset_target_service_name_;
    std::string status_topic_name_;
    std::string get_status_service_name_;

    mutable std::mutex status_mutex_;
    ExecutorState executor_state_{ExecutorState::STOPPED};
    rclcpp::Time last_start_time_;
    rclcpp::Time last_stop_time_;
    rclcpp::Time last_command_time_;
    rclcpp::Time last_status_publish_time_;
    bool stop_in_progress_{false};
    const std::chrono::milliseconds status_publish_period_{100};

    // --- Compute timing diagnostics ---
    bool enable_compute_timing_debug_{true};
    uint64_t timing_call_count_{0};
    uint64_t timing_overrun_800us_{0};
    uint64_t timing_overrun_1000us_{0};
    double   timing_max_compute_us_{0.0};
    std::chrono::steady_clock::time_point timing_last_report_time_;
};

}  // namespace fr3_husky_controller::servers::fr3