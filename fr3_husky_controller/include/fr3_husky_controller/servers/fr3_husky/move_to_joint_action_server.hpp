#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <fr3_husky_msgs/action/move_to_joint.hpp>

#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/model/fr3_husky_model_updater.hpp>

namespace fr3_husky_controller::servers::fr3_husky
{

/**
 * MoveToJoint (FR3-Husky variant)
 *
 * Plans a joint-space trajectory with MoveIt2 and forwards it to
 * fr3_husky_joint_trajectory_controller.  Husky wheels are held at zero
 * during planning and execution hand-off.
 */
class MoveToJoint final : public ActionServerBase<fr3_husky_msgs::action::MoveToJoint>
{
public:
    using ActionT       = fr3_husky_msgs::action::MoveToJoint;
    using Base          = ActionServerBase<ActionT>;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason    = typename Base::StopReason;
    using ResultPtr     = typename Base::ResultPtr;

    using FJT           = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FJT>;

    MoveToJoint(const std::string& name, const NodePtr& node,
                ModelUpdaterBase& model_updater);
    ~MoveToJoint() override;

    int priority() const override { return 0; }

private:
    bool          acceptGoal(const ActionT::Goal& goal) override;
    void          onGoalAccepted(const ActionT::Goal& goal) override;
    void          onStart() override;
    ComputeResult compute(const rclcpp::Time& time,
                          const rclcpp::Duration& period) override;
    void          onStop(StopReason reason) override;
    ResultPtr     makeResult(StopReason reason) override;

    static std::string inferGroup(const std::vector<std::string>& joint_names);
    void writeHoldCommands();
    void runPlanning();

    FR3HuskyModelUpdater& fr3_husky_model_updater_;

    rclcpp::Node::SharedPtr moveit_node_;

    rclcpp_action::Client<FJT>::SharedPtr jtc_client_;

    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr jtc_status_sub_;
    std::atomic<bool> jtc_busy_{false};

    enum class PlanState : uint8_t { PLANNING, DONE, FAILED };
    std::atomic<PlanState> plan_state_{PlanState::PLANNING};
    std::atomic<bool>      cancel_flag_{false};
    std::string            plan_error_msg_;
    std::mutex             msg_mutex_;
    std::thread            planning_thread_;

    std::vector<std::string> goal_joint_names_;
    std::vector<double>      goal_target_positions_;
    double                   goal_vel_scale_{0.1};
    double                   goal_acc_scale_{0.1};

    Eigen::VectorXd q_hold_;

    int32_t result_error_code_{0};
};

}  // namespace fr3_husky_controller::servers::fr3_husky
