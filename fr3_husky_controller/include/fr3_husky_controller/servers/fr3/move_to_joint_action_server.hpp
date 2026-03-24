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
#include <fr3_husky_controller/model/fr3_model_updater.hpp>

namespace fr3_husky_controller::servers::fr3
{

/**
 * MoveToJoint
 *
 * FR3 ros2_control action server plugin that:
 *  1. Receives a MoveToJoint goal (joint names + target positions).
 *  2. Rejects if fr3_joint_trajectory_controller is already executing
 *     (e.g. fr3_moveit.launch.py was started and MoveIt is controlling
 *      the robot via RViz).
 *  3. Plans a collision-free trajectory with MoveIt2 MoveGroupInterface
 *     in a background thread; writes PD hold commands during planning so
 *     the robot stays put.
 *  4. Forwards the planned FollowJointTrajectory goal to
 *     fr3_joint_trajectory_controller and returns SUCCEEDED, allowing
 *     JointTrajectoryController to become active_server_ and execute
 *     the trajectory.
 *
 * Registered action name: "fr3_move_to_joint"
 * Prerequisite: move_group must be running (e.g. via move_group.launch.py).
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
    // ---- ActionServerBase interface ----------------------------------------
    bool          acceptGoal(const ActionT::Goal& goal) override;
    void          onGoalAccepted(const ActionT::Goal& goal) override;
    void          onStart() override;
    ComputeResult compute(const rclcpp::Time& time,
                          const rclcpp::Duration& period) override;
    void          onStop(StopReason reason) override;
    ResultPtr     makeResult(StopReason reason) override;

    // ---- Helpers -------------------------------------------------------------
    /** Derive MoveIt planning group from joint names.
     *  e.g. "left_fr3_joint1" → "left_fr3_arm" */
    static std::string inferGroup(const std::vector<std::string>& joint_names);

    /** Write PD hold commands at q_hold_ (runs every compute() cycle
     *  while planning is in progress). */
    void writeHoldCommands();

    /** Background thread: plan via MoveGroupInterface, then send the
     *  resulting trajectory to fr3_joint_trajectory_controller. */
    void runPlanning();

    // ---- FR3 model -----------------------------------------------------------
    FR3ModelUpdater& fr3_model_updater_;

    // ---- MoveIt2 node (no executor — MoveGroupInterface spins it internally) --
    rclcpp::Node::SharedPtr moveit_node_;

    // ---- FollowJointTrajectory client → fr3_joint_trajectory_controller ------
    rclcpp_action::Client<FJT>::SharedPtr jtc_client_;

    // ---- JTC busy detection (subscribe to its action status topic) -----------
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr jtc_status_sub_;
    std::atomic<bool> jtc_busy_{false};

    // ---- Per-goal planning state (reset on every new goal) -------------------
    enum class PlanState : uint8_t { PLANNING, DONE, FAILED };
    std::atomic<PlanState> plan_state_{PlanState::PLANNING};
    std::atomic<bool>      cancel_flag_{false};
    std::string            plan_error_msg_;   ///< set by planning thread on failure
    std::mutex             msg_mutex_;        ///< guards plan_error_msg_
    std::thread            planning_thread_;

    // ---- Cached goal fields --------------------------------------------------
    std::vector<std::string> goal_joint_names_;
    std::vector<double>      goal_target_positions_;
    double                   goal_vel_scale_{0.1};
    double                   goal_acc_scale_{0.1};

    // ---- Position hold (latched at onStart) ----------------------------------
    Eigen::VectorXd q_hold_;

    // ---- Result bookkeeping --------------------------------------------------
    int32_t result_error_code_{0};
};

}  // namespace fr3_husky_controller::servers::fr3
