#pragma once

#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/model/fr3_model_updater.hpp>
#include <fr3_husky_msgs/action/omega_haptic.hpp>   // or your reused action type
#include <sensor_msgs/msg/joy.hpp>

#include <Eigen/Dense>
#include <map>
#include <mutex>
#include <vector>

namespace fr3_husky_controller::servers::fr3
{

class Ps4Haptic : public ActionServerBase<fr3_husky_msgs::action::OmegaHaptic>
{
public:
    using Base = ActionServerBase<fr3_husky_msgs::action::OmegaHaptic>;
    using ActionT = fr3_husky_msgs::action::OmegaHaptic;
    using ResultPtr = typename Base::ResultPtr;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason = typename Base::StopReason;

    Ps4Haptic(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater);

    bool acceptGoal(const ActionT::Goal& goal) override;
    void onGoalAccepted(const ActionT::Goal& goal) override;
    void onStart() override;
    ComputeResult compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void onStop(StopReason reason) override;
    ResultPtr makeResult(StopReason reason) override;

private:
    void subJoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    FR3ModelUpdater& fr3_model_updater_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    std::mutex joy_mutex_;
    std::vector<float> joy_axes_;
    std::vector<int> joy_buttons_;

    std::map<std::string, drc::TaskSpaceData> ee_data;

    int control_mode_{0};
    std::string control_ee_name_;
    bool move_ori_{false};

    double hapic_pos_multiplier_{1.0};
    double hapic_ori_multiplier_{1.0};
    double hapic_lin_vel_multiplier_{1.0};
    double hapic_ang_vel_multiplier_{1.0};

    bool is_mouse_mode_on_{false};
    bool prev_deadman_pressed_{false};

    int deadman_button_idx_{5};
    int left_stick_x_idx_{0};
    int left_stick_y_idx_{1};
    int right_stick_y_idx_{4};

    double deadzone_{0.08};
    double max_vx_{0.08};
    double max_vy_{0.08};
    double max_vz_{0.05};
    double vel_lpf_tau_{0.03};

    Eigen::Vector3d filtered_lin_vel_cmd_;
    Eigen::Affine3d target_pose_;
    Eigen::Matrix3d target_rotation_;
    Eigen::Vector3d workspace_min_;
    Eigen::Vector3d workspace_max_;
};

}  // namespace fr3_husky_controller::servers::fr3