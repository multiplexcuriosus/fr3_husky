#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

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

    int priority() const override { return 9; }
    bool allowPreemption() const override { return false; }  // if true, a new incoming goal preempts (aborts) the current one

private:
    bool          acceptGoal(const ActionT::Goal& goal) override;
    void          onGoalAccepted(const ActionT::Goal& goal) override;
    void          onStart() override;
    ComputeResult compute(const rclcpp::Time& time,
                          const rclcpp::Duration& period) override;
    void          onStop(StopReason reason) override;
    ResultPtr     makeResult(StopReason reason) override;

    void writeHoldCommands();
    void initializeMoveGroup();
    /** Background thread: plan via MoveGroupInterface, wait until this
     *  server has finished, then hand off the trajectory to
     *  fr3_husky_joint_trajectory_controller. */
    void runPlanning();

    // ---- Planning group (derived from robot_names at construction) -----------
    std::string planning_group_;
    std::set<std::string> valid_joint_prefixes_;

    FR3HuskyModelUpdater& fr3_husky_model_updater_;

    rclcpp::Node::SharedPtr moveit_node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::mutex move_group_mutex_;
    std::atomic<bool> planning_active_{false};

    rclcpp_action::Client<FJT>::SharedPtr jtc_client_;

    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr jtc_status_sub_;
    std::atomic<bool> jtc_busy_{false};

    enum class PlanState : uint8_t { PLANNING, READY, FAILED };
    std::atomic<PlanState> plan_state_{PlanState::PLANNING};
    std::atomic<bool>      cancel_flag_{false};
    std::atomic<bool>      handoff_requested_{false};
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
