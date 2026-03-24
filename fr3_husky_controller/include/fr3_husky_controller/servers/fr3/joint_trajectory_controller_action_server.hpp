#pragma once

#include <memory>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/model/fr3_model_updater.hpp>

namespace fr3_husky_controller::servers::fr3
{

class JointTrajectoryController final : public ActionServerBase<control_msgs::action::FollowJointTrajectory>
{
public:
    using ActionT = control_msgs::action::FollowJointTrajectory;
    using Base = ActionServerBase<ActionT>;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason = typename Base::StopReason;
    using ResultPtr = typename Base::ResultPtr;

    JointTrajectoryController(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater);
    ~JointTrajectoryController() override = default;

    int priority() const override { return 0; }

private:
    bool acceptGoal(const ActionT::Goal& goal) override;
    void onGoalAccepted(const ActionT::Goal& goal) override;
    void onStart() override;
    ComputeResult compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void onStop(StopReason reason) override;
    ResultPtr makeResult(StopReason reason) override;

    // Returns q_desired_total_ index for the given joint name, or -1 if unknown.
    // Expected format: {robot_names_[r]}_{arm_id_}_joint{1..FR3_DOF}
    int resolveJointIndex(const std::string& joint_name) const;

    // Returns true if any named joint in `tolerances` exceeds its position limit.
    // A tolerance.position <= 0 means "don't check".
    bool checkTolerance(const std::vector<control_msgs::msg::JointTolerance>& tolerances,
                        const std::vector<double>& position_errors) const;

private:
    FR3ModelUpdater& fr3_model_updater_;

    // ---- Cached goal data ----
    trajectory_msgs::msg::JointTrajectory trajectory_;
    std::vector<int> goal_to_cmd_index_;  // goal joint i  →  q_desired_total_ index
    std::vector<control_msgs::msg::JointTolerance> path_tolerance_;
    std::vector<control_msgs::msg::JointTolerance> goal_tolerance_;
    rclcpp::Duration goal_time_tolerance_{0, 0};

    // ---- Execution state ----
    rclcpp::Time start_time_;
    bool start_time_set_{false};
    bool trajectory_done_{false};
    rclcpp::Time trajectory_done_time_;
    int32_t result_error_code_{0};
    Eigen::VectorXd q_hold_;   // non-goal joints are held at this position (latched at onStart)
};

}  // namespace fr3_husky_controller::servers::fr3
