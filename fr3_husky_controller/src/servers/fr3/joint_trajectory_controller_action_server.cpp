#include <fr3_husky_controller/servers/fr3/joint_trajectory_controller_action_server.hpp>

#include <cmath>
#include <stdexcept>

#include <Eigen/Core>

namespace fr3_husky_controller::servers::fr3
{

namespace
{
FR3ModelUpdater& getFR3ModelUpdater(ModelUpdaterBase& model_updater, const std::string& server_name)
{
    auto* fr3_model_updater = dynamic_cast<FR3ModelUpdater*>(&model_updater);
    if (!fr3_model_updater)
    {
        throw std::runtime_error("[" + server_name + "] requires FR3ModelUpdater");
    }
    return *fr3_model_updater;
}
}  // namespace

// ============================================================
// Constructor
// ============================================================

JointTrajectoryController::JointTrajectoryController(
    const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
  fr3_model_updater_(getFR3ModelUpdater(model_updater, name))
{
    RCLCPP_INFO(node_->get_logger(), "[%s] JointTrajectoryController created", name_.c_str());
}

// ============================================================
// Helpers
// ============================================================

int JointTrajectoryController::resolveJointIndex(const std::string& joint_name) const
{
    // Expected: "{robot_names_[r]}_{arm_id_}_joint{1..FR3_DOF}"
    // e.g.  "left_fr3_joint3"  →  robot index r=0 ("left"), joint 3  →  total index 2
    for (size_t r = 0; r < model_updater_.num_robots_; ++r)
    {
        const std::string prefix =
            model_updater_.robot_names_[r] + "_" + model_updater_.arm_id_ + "_joint";

        if (joint_name.rfind(prefix, 0) != 0)  // must start with prefix
        {
            continue;
        }

        try
        {
            const int joint_num = std::stoi(joint_name.substr(prefix.size()));
            if (joint_num >= 1 && joint_num <= static_cast<int>(FR3_DOF))
            {
                return static_cast<int>(r * FR3_DOF) + joint_num - 1;
            }
        }
        catch (...) {}
    }
    return -1;
}

bool JointTrajectoryController::checkTolerance(
    const std::vector<control_msgs::msg::JointTolerance>& tolerances,
    const std::vector<double>& position_errors) const
{
    for (const auto& tol : tolerances)
    {
        if (tol.position <= 0.0)
        {
            continue;  // not checked
        }
        for (size_t i = 0; i < trajectory_.joint_names.size(); ++i)
        {
            if (trajectory_.joint_names[i] == tol.name)
            {
                if (std::abs(position_errors[i]) > tol.position)
                {
                    return true;  // violated
                }
                break;
            }
        }
    }
    return false;  // all OK
}

// ============================================================
// Goal lifecycle
// ============================================================

bool JointTrajectoryController::acceptGoal(const ActionT::Goal& goal)
{
    if (goal.trajectory.points.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject: trajectory has no points", name_.c_str());
        return false;
    }

    for (const auto& jname : goal.trajectory.joint_names)
    {
        if (resolveJointIndex(jname) < 0)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] Reject: unknown joint '%s'", name_.c_str(), jname.c_str());
            return false;
        }
    }

    return true;
}

void JointTrajectoryController::onGoalAccepted(const ActionT::Goal& goal)
{
    trajectory_         = goal.trajectory;
    path_tolerance_     = goal.path_tolerance;
    goal_tolerance_     = goal.goal_tolerance;
    goal_time_tolerance_ = rclcpp::Duration(goal.goal_time_tolerance);

    goal_to_cmd_index_.resize(trajectory_.joint_names.size());
    for (size_t i = 0; i < trajectory_.joint_names.size(); ++i)
    {
        goal_to_cmd_index_[i] = resolveJointIndex(trajectory_.joint_names[i]);
    }

    RCLCPP_INFO(node_->get_logger(),
                "[%s] Goal accepted: %zu joints, %zu waypoints",
                name_.c_str(), trajectory_.joint_names.size(), trajectory_.points.size());
}

void JointTrajectoryController::onStart()
{
    fr3_model_updater_.setInitFromCurrent();
    q_hold_           = fr3_model_updater_.q_total_;  // latch ALL joint positions
    start_time_set_   = false;
    trajectory_done_  = false;
    result_error_code_ = ActionT::Result::SUCCESSFUL;
    RCLCPP_INFO(node_->get_logger(), "[%s] started", name_.c_str());
}

// ============================================================
// Control loop
// ============================================================

JointTrajectoryController::ComputeResult JointTrajectoryController::compute(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/)
{
    // Latch start time on the first control tick.
    if (!start_time_set_)
    {
        start_time_     = time;
        start_time_set_ = true;
    }

    const auto&  points       = trajectory_.points;
    const size_t n_pts        = points.size();
    const size_t n_goal_joints = trajectory_.joint_names.size();
    const size_t total_dof    = model_updater_.manipulator_dof_;

    const double elapsed      = (time - start_time_).seconds();
    const double last_pt_time = rclcpp::Duration(points.back().time_from_start).seconds();
    const bool   at_goal      = (elapsed >= last_pt_time);

    // ---- Interpolate desired state ----
    // Non-goal joints use q_hold_ (latched at onStart) so the Kp error term is non-zero
    // and provides proper position-hold stiffness. Using q_total_ here would make
    // Kp*(q_desired - q_actual) = 0 every tick, leaving only velocity damping.
    Eigen::VectorXd q_desired    = q_hold_;
    Eigen::VectorXd qdot_desired = Eigen::VectorXd::Zero(total_dof);

    if (at_goal)
    {
        // Hold final waypoint.
        for (size_t i = 0; i < n_goal_joints; ++i)
        {
            const int idx = goal_to_cmd_index_[i];
            if (idx < 0) continue;
            q_desired(idx) = points.back().positions[i];
            if (!points.back().velocities.empty())
            {
                qdot_desired(idx) = points.back().velocities[i];
            }
        }
    }
    else
    {
        // Find seg_end: first waypoint whose time_from_start > elapsed.
        size_t seg_end = 0;
        for (size_t k = 0; k < n_pts; ++k)
        {
            if (rclcpp::Duration(points[k].time_from_start).seconds() > elapsed)
            {
                seg_end = k;
                break;
            }
        }

        const double t_end = rclcpp::Duration(points[seg_end].time_from_start).seconds();
        double t_start = 0.0;

        Eigen::VectorXd q_seg_start    = q_hold_;
        Eigen::VectorXd q_seg_end      = q_hold_;
        Eigen::VectorXd qdot_seg_start = Eigen::VectorXd::Zero(total_dof);
        Eigen::VectorXd qdot_seg_end   = Eigen::VectorXd::Zero(total_dof);

        if (seg_end == 0)
        {
            // Before first waypoint: interpolate from current robot state.
            for (size_t i = 0; i < n_goal_joints; ++i)
            {
                const int idx = goal_to_cmd_index_[i];
                if (idx < 0) continue;
                // q_seg_start already = current state
                q_seg_end(idx) = points[0].positions[i];
                if (!points[0].velocities.empty())
                {
                    qdot_seg_end(idx) = points[0].velocities[i];
                }
            }
        }
        else
        {
            t_start = rclcpp::Duration(points[seg_end - 1].time_from_start).seconds();
            for (size_t i = 0; i < n_goal_joints; ++i)
            {
                const int idx = goal_to_cmd_index_[i];
                if (idx < 0) continue;
                // seg_end == 1: first trajectory segment (points[0] → points[1]).
                // Keep q_seg_start at q_hold_ (actual robot position latched at onStart)
                // instead of overriding with points[0].positions (MoveIt's assumed start).
                // Any mismatch between the two would create a non-zero Kp error at t=0,
                // causing a torque spike → startup jolt.  For seg_end > 1 we trust the
                // trajectory waypoints directly (the robot is already tracking them).
                if (seg_end > 1)
                {
                    q_seg_start(idx) = points[seg_end - 1].positions[i];
                    if (!points[seg_end - 1].velocities.empty())
                    {
                        qdot_seg_start(idx) = points[seg_end - 1].velocities[i];
                    }
                }
                q_seg_end(idx) = points[seg_end].positions[i];
                if (!points[seg_end].velocities.empty())
                {
                    qdot_seg_end(idx) = points[seg_end].velocities[i];
                }
            }
        }

        const double duration      = t_end - t_start;
        const bool   has_velocities = !points[seg_end].velocities.empty();

        if (fr3_model_updater_.robot_controller_ && has_velocities && duration > 0.0)
        {
            // Cubic spline using RobotController helpers.
            q_desired = fr3_model_updater_.robot_controller_->moveJointPositionCubic(
                q_seg_end, qdot_seg_end, q_seg_start, qdot_seg_start,
                elapsed, t_start, duration);
            qdot_desired = fr3_model_updater_.robot_controller_->moveJointVelocityCubic(
                q_seg_end, qdot_seg_end, q_seg_start, qdot_seg_start,
                elapsed, t_start, duration);
        }
        else if (duration > 0.0)
        {
            // Linear fallback (no velocity info in waypoints).
            const double alpha = std::clamp((elapsed - t_start) / duration, 0.0, 1.0);
            q_desired    = q_seg_start + alpha * (q_seg_end - q_seg_start);
            qdot_desired = (q_seg_end - q_seg_start) / duration;
        }
        else
        {
            q_desired = q_seg_end;
        }
    }

    // ---- Write command ----
    if (model_updater_.HasEffortCommandInterface())
    {
        if (!fr3_model_updater_.robot_controller_)
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                  "[%s] robot_controller_ is null; cannot compute torque", name_.c_str());
            result_error_code_ = ActionT::Result::INVALID_GOAL;
            return ComputeResult::ABORTED;
        }
        // PD + gravity:  τ = Kp*(q_d-q) + Kv*(qdot_d-qdot) + g   (use_mass=false)
        const Eigen::VectorXd torque =
            fr3_model_updater_.robot_controller_->moveJointTorqueStep(q_desired, qdot_desired, false);
        fr3_model_updater_.torque_desired_total_ = torque - fr3_model_updater_.g_total_;
        fr3_model_updater_.writeCommand(fr3_model_updater_.torque_desired_total_);
    }
    else if (model_updater_.HasVelocityCommandInterface())
    {
        fr3_model_updater_.qdot_desired_total_ = qdot_desired;
        fr3_model_updater_.writeCommand(fr3_model_updater_.qdot_desired_total_);
    }
    else if (model_updater_.HasPositionCommandInterface())
    {
        fr3_model_updater_.q_desired_total_ = q_desired;
        fr3_model_updater_.writeCommand(fr3_model_updater_.q_desired_total_);
    }
    else
    {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                              "[%s] No suitable command interface available", name_.c_str());
        result_error_code_ = ActionT::Result::INVALID_GOAL;
        return ComputeResult::ABORTED;
    }

    // ---- Build feedback ----
    auto fb = std::make_shared<ActionT::Feedback>();
    fb->header.stamp = time;
    fb->joint_names  = trajectory_.joint_names;
    fb->desired.positions.assign(n_goal_joints, 0.0);
    fb->actual.positions.assign(n_goal_joints, 0.0);
    fb->error.positions.assign(n_goal_joints, 0.0);
    fb->desired.velocities.assign(n_goal_joints, 0.0);
    fb->actual.velocities.assign(n_goal_joints, 0.0);
    fb->error.velocities.assign(n_goal_joints, 0.0);

    for (size_t i = 0; i < n_goal_joints; ++i)
    {
        const int idx = goal_to_cmd_index_[i];
        if (idx < 0) continue;
        fb->desired.positions[i]  = q_desired(idx);
        fb->actual.positions[i]   = fr3_model_updater_.q_total_(idx);
        fb->error.positions[i]    = q_desired(idx) - fr3_model_updater_.q_total_(idx);
        fb->desired.velocities[i] = qdot_desired(idx);
        fb->actual.velocities[i]  = fr3_model_updater_.qdot_total_(idx);
        fb->error.velocities[i]   = qdot_desired(idx) - fr3_model_updater_.qdot_total_(idx);
    }
    publishFeedback(fb);

    // ---- Path tolerance check (while tracking, not during goal hold) ----
    if (!at_goal && checkTolerance(path_tolerance_, fb->error.positions))
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Path tolerance violated", name_.c_str());
        result_error_code_ = ActionT::Result::PATH_TOLERANCE_VIOLATED;
        return ComputeResult::ABORTED;
    }

    // ---- Goal completion check ----
    if (at_goal)
    {
        if (!trajectory_done_)
        {
            trajectory_done_      = true;
            trajectory_done_time_ = time;
        }

        // All tolerances satisfied (or none specified) → success.
        if (!checkTolerance(goal_tolerance_, fb->error.positions))
        {
            return ComputeResult::SUCCEEDED;
        }

        // Tolerance violated; wait within goal_time_tolerance if > 0.
        const double goal_time_tol       = goal_time_tolerance_.seconds();
        const double time_since_done     = (time - trajectory_done_time_).seconds();

        if (goal_time_tol > 0.0 && time_since_done > goal_time_tol)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] Goal tolerance not met within %.3f s", name_.c_str(), goal_time_tol);
            result_error_code_ = ActionT::Result::GOAL_TOLERANCE_VIOLATED;
            return ComputeResult::ABORTED;
        }

        // Still within the grace window (or no time limit); keep holding.
        return ComputeResult::RUNNING;
    }

    return ComputeResult::RUNNING;
}

// ============================================================
// Stop / Result
// ============================================================

void JointTrajectoryController::onStop(StopReason reason)
{
    model_updater_.haltCommands();

    const char* reason_str = "none";
    if (reason == StopReason::CANCELED)       reason_str = "canceled";
    else if (reason == StopReason::SUCCEEDED) reason_str = "succeeded";
    else if (reason == StopReason::ABORTED)   reason_str = "aborted";

    RCLCPP_INFO(node_->get_logger(), "[%s] stopped (%s)", name_.c_str(), reason_str);
}

JointTrajectoryController::ResultPtr JointTrajectoryController::makeResult(StopReason reason)
{
    auto result = std::make_shared<ActionT::Result>();
    if (reason == StopReason::SUCCEEDED || reason == StopReason::CANCELED)
    {
        result->error_code = ActionT::Result::SUCCESSFUL;
    }
    else
    {
        result->error_code = result_error_code_;
    }
    return result;
}

// Register this server into global registry (executed when this TU is linked)
REGISTER_FR3_ACTION_SERVER(JointTrajectoryController, "fr3_joint_trajectory_controller")

}  // namespace fr3_husky_controller::servers::fr3
