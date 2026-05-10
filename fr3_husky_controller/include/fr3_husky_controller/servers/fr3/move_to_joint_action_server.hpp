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
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

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

    int priority() const override { return 9; }
    bool allowPreemption() const override { return false; }  // if true, a new incoming goal preempts (aborts) the current one

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
    /** Write PD hold commands at q_hold_ (runs every compute() cycle
     *  while planning is in progress). */
    void writeHoldCommands();

    /** Background thread: plan via MoveGroupInterface, wait until this
     *  server has finished, then hand off the trajectory to
     *  fr3_joint_trajectory_controller. */
    void runPlanning();

    bool validateGoalInputs(std::string* reason = nullptr) const;
    bool isJointStateFresh(std::string* reason = nullptr) const;
    bool isMoveItCurrentStateFresh(std::string* reason = nullptr);
    bool isCartesianExecutorSafe(std::string* reason = nullptr);
    bool runPreflightSafetyChecks(std::string* reason = nullptr);
    bool runPostPlanSafetyChecks(std::string* reason = nullptr);

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ---- Planning group (derived from robot_names at construction) -----------
    /** MoveIt planning group name, e.g. "left_fr3_arm" / "dual_fr3_arm". */
    std::string planning_group_;
    /** Valid joint-name prefixes for this group, e.g. {"left_fr3_", "right_fr3_"}. */
    std::set<std::string> valid_joint_prefixes_;

    // ---- FR3 model -----------------------------------------------------------
    FR3ModelUpdater& fr3_model_updater_;

    // ---- MoveIt2 node (no executor — MoveGroupInterface spins it internally) --
    rclcpp::Node::SharedPtr moveit_node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;

    // ---- FollowJointTrajectory client → fr3_joint_trajectory_controller ------
    rclcpp_action::Client<FJT>::SharedPtr jtc_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cartesian_status_client_;

    // ---- JTC busy detection (subscribe to its action status topic) -----------
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr jtc_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::atomic<bool> jtc_busy_{false};

    // ---- Per-goal planning state (reset on every new goal) -------------------
    enum class PlanState : uint8_t { PLANNING, READY, FAILED };
    std::atomic<PlanState> plan_state_{PlanState::PLANNING};
    std::atomic<bool>      cancel_flag_{false};
    std::atomic<bool>      handoff_requested_{false};
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

    // ---- Safety inputs / parameters ------------------------------------------
    mutable std::mutex joint_state_mutex_;
    bool have_joint_state_{false};
    rclcpp::Time last_joint_state_stamp_;
    rclcpp::Time last_joint_state_receive_time_;

    bool require_fresh_joint_state_{true};
    int max_joint_state_age_ms_{150};
    bool require_fresh_moveit_state_{true};
    int max_moveit_state_age_ms_{150};
    bool require_cartesian_stopped_{true};
    std::string cartesian_get_status_service_{"/cartesian_executor/get_status"};
    int cartesian_settle_delay_ms_{1500};
    int wait_for_cartesian_status_timeout_ms_{300};
    bool reject_if_cartesian_status_unavailable_{true};
    std::string joint_state_topic_{"/right_fr3/joint_states"};
};

}  // namespace fr3_husky_controller::servers::fr3
