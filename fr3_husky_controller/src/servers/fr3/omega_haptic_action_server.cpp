#include <fr3_husky_controller/servers/fr3/omega_haptic_action_server.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

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

OmegaHaptic::OmegaHaptic(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
  fr3_model_updater_(getFR3ModelUpdater(model_updater, name))
{
    pose_sub_         = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/haptic/pose", 1, std::bind(&OmegaHaptic::subPoseCallback, this, std::placeholders::_1));
    ori_encoder_sub_  = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/haptic/encoder_orientation", 1, std::bind(&OmegaHaptic::subOriEncoderCallback, this, std::placeholders::_1));
    twist_sub_        = node_->create_subscription<geometry_msgs::msg::Twist>("/haptic/twist", 1, std::bind(&OmegaHaptic::subTwistCallback, this, std::placeholders::_1));
    button_state_sub_ = node_->create_subscription<std_msgs::msg::Int8MultiArray>("/haptic/button_state", 1, std::bind(&OmegaHaptic::subButtonCallback, this, std::placeholders::_1));
    
    wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>("/haptic/wrench_feedback", 1);
    force_pub_  = node_->create_publisher<geometry_msgs::msg::Vector3>("/haptic/force_feedback", 1);

    haptic_pose_.setIdentity();
    haptic_vel_.setZero();
    haptic_ori_encoder_.setZero();
    haptic_pose_init_.setIdentity();
    button0_state_ = false;

    haptic_base2robot_base_.setIdentity();
    haptic_base2robot_base_ = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    ee_data.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] OmegaHaptic created", name_.c_str());
}

bool OmegaHaptic::acceptGoal(const ActionT::Goal& goal)
{
    if (!model_updater_.HasEffortCommandInterface())
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject action: effort command interface is required",
                                         name_.c_str());
        return false;
    }

    if (goal.mode < 0 || goal.mode > 3)
    {
        RCLCPP_WARN(node_->get_logger(),
                                         "[%s] Reject action: mode must be 0 to 3 (0: CLIK, 1: OSF, 2: QPIK, 3: QPID). The mode from action goal is %d.",
                                         name_.c_str(),
                                         static_cast<int>(goal.mode));
        return false;
    }

    if(!fr3_model_updater_.robot_data_->hasLinkFrame(goal.ee_name))
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject action: ee_name from the goal [%s] is not includede in URDF.",
                                         name_.c_str(), goal.ee_name.c_str());
        return false;
    }

    return true;
}

void OmegaHaptic::onGoalAccepted(const ActionT::Goal& goal)
{
    control_mode_ = goal.mode;
    control_ee_name_ = goal.ee_name;
    move_ori_ = goal.move_orientation;
    hapic_pos_multiplier_ = static_cast<double>(goal.hapic_pos_multiplier);
    hapic_ori_multiplier_ = static_cast<double>(goal.hapic_ori_multiplier);
    hapic_lin_vel_multiplier_ = static_cast<double>(goal.hapic_lin_vel_multiplier);
    hapic_ang_vel_multiplier_ = static_cast<double>(goal.hapic_ang_vel_multiplier);
}

void OmegaHaptic::onStart()
{
    {
        std::lock_guard<std::mutex> lock(haptic_pose_mutex_);
        haptic_pose_.setIdentity();
    }
    {
        std::lock_guard<std::mutex> lock(haptic_vel_mutex_);
        haptic_vel_.setZero();
    }
    {
        std::lock_guard<std::mutex> lock(haptic_ori_encoder_mutex_);
        haptic_ori_encoder_.setZero();
    }
    haptic_pose_init_.setIdentity();
    {
        std::lock_guard<std::mutex> lock(button0_state_mutex_);
        button0_state_ = false;
    }
    
    is_mouse_mode_on_ = false;

    ee_data.clear();
    ee_data[control_ee_name_] = drc::TaskSpaceData::Zero();
    ee_data[control_ee_name_].x = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    ee_data[control_ee_name_].xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    ee_data[control_ee_name_].xddot.setZero();
    ee_data[control_ee_name_].setInit();
    ee_data[control_ee_name_].setDesired();

    RCLCPP_INFO(node_->get_logger(), "[%s] started", name_.c_str());
}

OmegaHaptic::ComputeResult OmegaHaptic::compute(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    ee_data[control_ee_name_].x = fr3_model_updater_.robot_data_->getPose(control_ee_name_);
    ee_data[control_ee_name_].xdot = fr3_model_updater_.robot_data_->getVelocity(control_ee_name_);
    ee_data[control_ee_name_].xddot.setZero();

    Eigen::Affine3d haptic_pose_local;
    Eigen::Vector6d haptic_vel_local;
    Eigen::Vector3d haptic_ori_encoder_local;
    bool button0_state_local = false;

    {
        std::lock_guard<std::mutex> lock(haptic_pose_mutex_);
        haptic_pose_local = haptic_pose_;
    }
    {
        std::lock_guard<std::mutex> lock(haptic_vel_mutex_);
        haptic_vel_local = haptic_vel_;
    }
    {
        std::lock_guard<std::mutex> lock(haptic_ori_encoder_mutex_);
        haptic_ori_encoder_local = haptic_ori_encoder_;
    }
    {
        std::lock_guard<std::mutex> lock(button0_state_mutex_);
        button0_state_local = button0_state_;
    }
    (void)haptic_ori_encoder_local;


    if(!is_mouse_mode_on_ && button0_state_local) // activate mouse mode
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] Mouse Mode activated!", name_.c_str());
        is_mouse_mode_on_ = true;

        haptic_pose_init_ = haptic_pose_local;
        ee_data[control_ee_name_].setInit();
    }
    else if(is_mouse_mode_on_ && !button0_state_local) // deactivate mouse mode
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] Mouse Mode deactivated!", name_.c_str());
        is_mouse_mode_on_ = false;

        haptic_pose_init_ = haptic_pose_local;
        ee_data[control_ee_name_].setInit();
    }

    Eigen::Affine3d target_pose_diff;
    Eigen::Vector6d target_vel;
    target_pose_diff.setIdentity();
    target_vel.setZero();
    if(is_mouse_mode_on_)
    {
        // haptic diff in haptic_init frame: D = H_init^{-1} * H_cur
        Eigen::Affine3d haptic_pose_diff = haptic_pose_init_.inverse() * haptic_pose_local;

        const Eigen::Matrix3d& R_h2r     = haptic_base2robot_base_;
        const Eigen::Matrix3d& R_init    = haptic_pose_init_.linear();
        const Eigen::Matrix3d& R_ee_init = ee_data[control_ee_name_].x_init.linear();

        // Position: haptic_init frame → haptic_base → robot_base → EE_init body frame
        // delta_pos (haptic_init) = D.translation()
        // delta_pos (haptic_base) = R_init * D.translation()
        // delta_pos (robot_base)  = R_h2r * R_init * D.translation()
        // delta_pos (EE body)     = R_ee_init^T * delta_pos (robot_base)
        const Eigen::Vector3d delta_pos = hapic_pos_multiplier_
            * R_ee_init.transpose() * R_h2r * R_init * haptic_pose_diff.translation();

        // Orientation: scale rotation angle by hapic_ori_multiplier_ (via AngleAxis),
        //              then convert haptic_init frame → robot_base → EE_init body frame
        Eigen::Matrix3d rot_diff_ee_body = Eigen::Matrix3d::Identity();
        if(move_ori_)
        {
            const Eigen::AngleAxisd aa(haptic_pose_diff.linear());
            const Eigen::Matrix3d R_diff_scaled = (std::abs(aa.angle()) > 1e-10)
                ? Eigen::AngleAxisd(hapic_ori_multiplier_ * aa.angle(), aa.axis()).toRotationMatrix()
                : Eigen::Matrix3d::Identity();
            // R_diff (haptic_init) → R_h2r * R_diff * R_h2r^T (robot_base) → R_ee^T * (...) * R_ee (EE body)
            rot_diff_ee_body = R_ee_init.transpose() * R_h2r * R_diff_scaled * R_h2r.transpose() * R_ee_init;
        }

        target_pose_diff.translation() = delta_pos;
        target_pose_diff.linear()      = rot_diff_ee_body;
    }
    
    ee_data[control_ee_name_].x_desired = ee_data[control_ee_name_].x_init * target_pose_diff;
    ee_data[control_ee_name_].xdot_desired  = target_vel;

    Eigen::VectorXd torque_desired;
    torque_desired.setZero(fr3_model_updater_.manipulator_dof_);
    bool is_qp_solved = true;
    std::string time_verbose = "";
    switch (control_mode_)
    {
        case 0: // CLIK
            fr3_model_updater_.robot_controller_->CLIKStep(ee_data, fr3_model_updater_.qdot_desired_total_);
            // fr3_model_updater_.qdot_desired_total_ = dyros_math::lowPassFilter(fr3_model_updater_.qdot_desired_total_,
            //                                                                                         fr3_model_updater_.qdot_total_,
            //                                                                                         fr3_model_updater_.dt_,
            //                                                                                         0.01);
            fr3_model_updater_.q_desired_total_ = fr3_model_updater_.q_total_ +
                                                                       fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;
            
            fr3_model_updater_.torque_desired_total_ = fr3_model_updater_.robot_controller_->moveJointTorqueStep(fr3_model_updater_.q_desired_total_,
                                                                                                                   fr3_model_updater_.qdot_desired_total_,
                                                                                                                   false);
            break;
        case 1: // OSF
            fr3_model_updater_.robot_controller_->OSFStep(ee_data, fr3_model_updater_.torque_desired_total_);
            break;
        case 2: // QPIK
            is_qp_solved = fr3_model_updater_.robot_controller_->QPIKStep(ee_data, fr3_model_updater_.qdot_desired_total_, time_verbose);
            if(!is_qp_solved) fr3_model_updater_.qdot_desired_total_.setZero();
            // fr3_model_updater_.qdot_desired_total_ = dyros_math::lowPassFilter(fr3_model_updater_.qdot_desired_total_,
            //                                                                                         fr3_model_updater_.qdot_total_,
            //                                                                                         fr3_model_updater_.dt_,
            //                                                                                         0.01);
            fr3_model_updater_.q_desired_total_ = fr3_model_updater_.q_total_ +
                                                                       fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;
            fr3_model_updater_.torque_desired_total_ = fr3_model_updater_.robot_controller_->moveJointTorqueStep(fr3_model_updater_.q_desired_total_,
                                                                                                                   fr3_model_updater_.qdot_desired_total_,
                                                                                                                   false);
            break;
        case 3: // QPID
            is_qp_solved = fr3_model_updater_.robot_controller_->QPIDStep(ee_data, fr3_model_updater_.torque_desired_total_, time_verbose);
            if(!is_qp_solved) fr3_model_updater_.torque_desired_total_ = fr3_model_updater_.robot_data_->getGravity();
            break;
        default:
            break;
    }

    fr3_model_updater_.writeCommand(fr3_model_updater_.torque_desired_total_ - fr3_model_updater_.g_total_); // robot_controller automatically add gravity force
    // fr3_model_updater_.haltCommands();


    auto fb = std::make_shared<ActionT::Feedback>();
    fb->is_qp_solved = is_qp_solved;
    fb->time_verbose = time_verbose;
    publishFeedback(fb);

    return ComputeResult::RUNNING;
}

void OmegaHaptic::onStop(StopReason reason)
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

OmegaHaptic::ResultPtr OmegaHaptic::makeResult(StopReason reason)
{
    auto result = std::make_shared<ActionT::Result>();
    result->is_completed = true;
    return result;
}

void OmegaHaptic::subPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    position = dyros_math::lowPassFilter(position, haptic_pose_.translation(), 0.001, 0.002);
    Eigen::Quaterniond quaternion(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    quaternion.normalize();
    Eigen::Matrix3d orientation = quaternion.toRotationMatrix();
    {
        std::lock_guard<std::mutex> lock(haptic_pose_mutex_);
        haptic_pose_.translation() = position;
        haptic_pose_.linear() = orientation;
    }
}

void OmegaHaptic::subOriEncoderCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    Eigen::Vector3d ori_encoder(msg->data[0], msg->data[1], msg->data[2]);
    {
        std::lock_guard<std::mutex> lock(haptic_ori_encoder_mutex_);
        haptic_ori_encoder_ = ori_encoder;
    }
}

void OmegaHaptic::subTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    Eigen::Vector6d vel(msg->linear.x, msg->linear.y, msg->linear.z,
                        msg->angular.x, msg->angular.y, msg->angular.z);
    vel = dyros_math::lowPassFilter(vel, haptic_vel_, 0.001, 0.002);
    
    {
        std::lock_guard<std::mutex> lock(haptic_vel_mutex_);
        haptic_vel_ = vel;
    }
}

void OmegaHaptic::subButtonCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg)
{
    // bool button0_state = static_cast<bool>(msg->data[0]);
    bool button0_state = (static_cast<int>(msg->data[0]) == 0) ? false : true;
    {
        std::lock_guard<std::mutex> lock(button0_state_mutex_);
        button0_state_ = button0_state;
    }
}

// Register this server into global registry (executed when this TU is linked)
REGISTER_FR3_ACTION_SERVER(OmegaHaptic, "omega_haptic")

}  // namespace fr3_husky_controller::servers::fr3

/* check action name
ros2 action list -t | grep omega_haptic

# send goal 
ros2 action send_goal /omega_haptic fr3_husky_msgs/action/OmegaHaptic \
"{mode: 1, ee_name: 'left_fr3_hand_tcp', move_orientation: false, hapic_pos_multiplier: 1.0, hapic_ori_multiplier: 1.0, hapic_lin_vel_multiplier: 1.0, hapic_ang_vel_multiplier: 1.0}" \
--feedback
*/
