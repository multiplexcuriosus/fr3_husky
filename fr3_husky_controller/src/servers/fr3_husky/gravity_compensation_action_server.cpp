#include <fr3_husky_controller/servers/fr3_husky/gravity_compensation_action_server.hpp>

#include <map>
#include <stdexcept>

#include <Eigen/Core>

namespace fr3_husky_controller::servers::fr3_husky
{

namespace
{
FR3HuskyModelUpdater& getFR3HuskyModelUpdater(ModelUpdaterBase& model_updater, const std::string& server_name)
{
    auto* fr3_husky_model_updater = dynamic_cast<FR3HuskyModelUpdater*>(&model_updater);
    if (!fr3_husky_model_updater)
    {
        throw std::runtime_error("[" + server_name + "] requires FR3HuskyModelUpdater");
    }
    return *fr3_husky_model_updater;
}

}  // namespace

GravityCompensation::GravityCompensation(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
  fr3_husky_model_updater_(getFR3HuskyModelUpdater(model_updater, name))
{
    RCLCPP_INFO(node_->get_logger(), "[%s] GravityCompensation created", name_.c_str());
}

bool GravityCompensation::acceptGoal(const ActionT::Goal& goal)
{
    if (!model_updater_.HasEffortCommandInterface())
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "[%s] Reject action: effort command interface is required",
            name_.c_str());
        return false;
    }

    return true;
}

void GravityCompensation::onGoalAccepted(const Goal& goal)
{
    use_qp_ = goal.use_qp;
    RCLCPP_INFO(node_->get_logger(), "[%s] use_qp: %s", name_.c_str(), use_qp_ ?  "true" : "false");
}

void GravityCompensation::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[%s] started", name_.c_str());
}

GravityCompensation::ComputeResult GravityCompensation::compute(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if(use_qp_)
    {
        // For gravity compensation using QPID, set desired acceleration and tracking gains as zero.
        std::map<std::string, drc::TaskSpaceData> zero_data;
        std::map<std::string, Eigen::Vector6d> zero_qpid_tracking;
        for (const auto& ee_name : model_updater_.ee_names_)
        {
            zero_data[ee_name] = drc::TaskSpaceData::Zero();
            zero_qpid_tracking[ee_name] = Eigen::Vector6d::Zero();
        }
        fr3_husky_model_updater_.robot_controller_->setQPIDTrackingGain(zero_qpid_tracking);
    
        Eigen::VectorXd opt_torque, opt_wheel_qddot;
        opt_torque.setZero(model_updater_.manipulator_dof_);
        opt_wheel_qddot.setZero(2);
    
        std::string time_verbose;
        const bool qp_ok = fr3_husky_model_updater_.robot_controller_->QPID(zero_data, opt_wheel_qddot, opt_torque, time_verbose);
    
        if (qp_ok)
        {
            fr3_husky_model_updater_.torque_desired_total_ = opt_torque - fr3_husky_model_updater_.g_total_;
            fr3_husky_model_updater_.wheel_vel_desired_ = fr3_husky_model_updater_.wheel_vel_ + opt_wheel_qddot * fr3_husky_model_updater_.dt_;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "[%s] QPID solve failed", name_.c_str());
            fr3_husky_model_updater_.torque_desired_total_.setZero();
            fr3_husky_model_updater_.wheel_vel_desired_ = fr3_husky_model_updater_.wheel_vel_;
        }
    
        fr3_husky_model_updater_.writeCommand(fr3_husky_model_updater_.torque_desired_total_,
                                              fr3_husky_model_updater_.wheel_vel_desired_);

        auto fb = std::make_shared<ActionT::Feedback>();
        fb->is_qp_solved = qp_ok;
        fb->time_verbose = time_verbose;
        publishFeedback(fb);

    }
    else
    {
        fr3_husky_model_updater_.torque_desired_total_.setZero();
        fr3_husky_model_updater_.wheel_vel_desired_.setZero();
        fr3_husky_model_updater_.writeCommand(fr3_husky_model_updater_.torque_desired_total_,
                                              fr3_husky_model_updater_.wheel_vel_desired_);

        auto fb = std::make_shared<ActionT::Feedback>();
        fb->is_qp_solved = true;
        fb->time_verbose = "";
        publishFeedback(fb);
    }

    // Keep running until explicit cancel request arrives.
    return ComputeResult::RUNNING;
}

void GravityCompensation::onStop(StopReason reason)
{
    model_updater_.haltCommands();

    const char* reason_str = "none";
    if (reason == StopReason::CANCELED)
    {
        reason_str = "canceled";
    }
    else if (reason == StopReason::SUCCEEDED)
    {
        reason_str = "succeeded";
    }
    else if (reason == StopReason::ABORTED)
    {
        reason_str = "aborted";
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] stopped (%s)", name_.c_str(), reason_str);
}

GravityCompensation::ResultPtr GravityCompensation::makeResult(StopReason reason)
{
    auto result = std::make_shared<ActionT::Result>();
    result->is_completed = (reason == StopReason::SUCCEEDED || reason == StopReason::CANCELED);
    return result;
}

// Register this server into global registry (executed when this TU is linked)
REGISTER_FR3_HUSKY_ACTION_SERVER(GravityCompensation, "fr3_husky_gravity_compensation")

}  // namespace fr3_husky_controller::servers::fr3_husky
/*
# send goal
ros2 action send_goal /fr3_husky_gravity_compensation fr3_husky_msgs/action/GravityCompensation \
 "{use_qp: true}"
*/