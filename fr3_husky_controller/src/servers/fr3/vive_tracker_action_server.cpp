#include <fr3_husky_controller/servers/fr3/vive_tracker_action_server.hpp>

#include <stdexcept>

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

ViveTracker::ViveTracker(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
  fr3_model_updater_(getFR3ModelUpdater(model_updater, name))
{
    pose_sub_         = node_->create_subscription<geometry_msgs::msg::PoseArray>("tracker_pose", 1, std::bind(&ViveTracker::subPoseCallback, this, std::placeholders::_1));
    l_button_state_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>("lhand_button", 1, std::bind(&ViveTracker::subLButtonCallback, this, std::placeholders::_1));
    r_button_state_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>("rhand_button", 1, std::bind(&ViveTracker::subRButtonCallback, this, std::placeholders::_1));
    
    tracker_poses_.assign(3, Eigen::Affine3d::Identity());
    tracker_poses_init_.assign(3, Eigen::Affine3d::Identity());
    button_states_.assign(2, std::vector<bool>(4, false));
    is_mouse_mode_on_.assign(2, false);

    tracker_base2robot_base_.assign(2, Eigen::Matrix3d::Identity());
    // tracker_base2robot_base_[0] = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix(); 
                                //   Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // tracker_base2robot_base_[1] = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix() * 
    //                               Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    ee_data_.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] ViveTracker created", name_.c_str());
}

bool ViveTracker::acceptGoal(const ActionT::Goal& goal)
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

    if(!goal.left_ee_name.empty() && !fr3_model_updater_.robot_data_->hasLinkFrame(goal.left_ee_name))
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject action: left_ee_name from the goal [%s] is not includede in URDF.",
                                         name_.c_str(), goal.left_ee_name.c_str());
        return false;
    }

    if(!goal.right_ee_name.empty() && !fr3_model_updater_.robot_data_->hasLinkFrame(goal.right_ee_name))
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Reject action: right_ee_name from the goal [%s] is not includede in URDF.",
                                         name_.c_str(), goal.right_ee_name.c_str());
        return false;
    }

    return true;
}

void ViveTracker::onGoalAccepted(const ActionT::Goal& goal)
{
    control_mode_ = goal.mode;
    control_left_ee_name_ = goal.left_ee_name;
    control_right_ee_name_ = goal.right_ee_name;
    move_ori_ = goal.move_orientation;
    tracker_pos_multiplier_ = static_cast<double>(goal.tracker_pos_multiplier);
    tracker_ori_multiplier_ = static_cast<double>(goal.tracker_ori_multiplier);
}

void ViveTracker::onStart()
{
    {
        std::lock_guard<std::mutex> lock(tracker_pose_mutex_);
        for(auto& tracker_pose : tracker_poses_) tracker_pose.setIdentity();
    }
    for(auto& tracker_pose_init : tracker_poses_init_) tracker_pose_init.setIdentity();
    {
        std::lock_guard<std::mutex> lock(button_state_mutex_);
        for(auto& button_state : button_states_) button_state = std::vector<bool>(4, false);
    }

    is_mouse_mode_on_.assign(2, false);
    ee_data_.clear();

    if(!control_left_ee_name_.empty())
    {
        ee_data_[control_left_ee_name_] = drc::TaskSpaceData::Zero();
        ee_data_[control_left_ee_name_].x = fr3_model_updater_.robot_data_->getPose(control_left_ee_name_);
        ee_data_[control_left_ee_name_].xdot = fr3_model_updater_.robot_data_->getVelocity(control_left_ee_name_);
        ee_data_[control_left_ee_name_].xddot.setZero();
        ee_data_[control_left_ee_name_].setInit();
        ee_data_[control_left_ee_name_].setDesired();
    }
    if(!control_right_ee_name_.empty())
    {
        ee_data_[control_right_ee_name_] = drc::TaskSpaceData::Zero();
        ee_data_[control_right_ee_name_].x = fr3_model_updater_.robot_data_->getPose(control_right_ee_name_);
        ee_data_[control_right_ee_name_].xdot = fr3_model_updater_.robot_data_->getVelocity(control_right_ee_name_);
        ee_data_[control_right_ee_name_].xddot.setZero();
        ee_data_[control_right_ee_name_].setInit();
        ee_data_[control_right_ee_name_].setDesired();
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] started", name_.c_str());
}

ViveTracker::ComputeResult ViveTracker::compute(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    for(auto& [ee_name, ee_data] : ee_data_)
    {
        ee_data.x = fr3_model_updater_.robot_data_->getPose(ee_name);
        ee_data.xdot = fr3_model_updater_.robot_data_->getVelocity(ee_name);
        ee_data.xddot.setZero();
    }

    std::vector<Eigen::Affine3d> tracker_poses_local;      // left, right, head
    std::vector<std::vector<bool>> button_states_local;    // [left, right][trigger, grip, a, b]
    {
        std::lock_guard<std::mutex> lock(tracker_pose_mutex_);
        tracker_poses_local = tracker_poses_;
    }
    {
        std::lock_guard<std::mutex> lock(button_state_mutex_);
        button_states_local = button_states_;
    }

    for(size_t i = 0; i < 2; ++i)
    {
        if(!is_mouse_mode_on_[i] && button_states_local[i][1]) // activate mouse mode when grip button pressed
        {
            RCLCPP_INFO(node_->get_logger(), "[%s] %s Mouse Mode activated!", name_.c_str(), (i==0)?"Left":"Right");
            is_mouse_mode_on_[i] = true;

            tracker_poses_init_[i] = tracker_poses_local[i];
            if(i == 0 && !control_left_ee_name_.empty())       ee_data_[control_left_ee_name_].setInit();
            else if(i == 1 && !control_right_ee_name_.empty()) ee_data_[control_right_ee_name_].setInit();
        }
        else if(is_mouse_mode_on_[i] && !button_states_local[i][1]) // deactivate mouse mode when grip button released
        {
            RCLCPP_INFO(node_->get_logger(), "[%s] %s Mouse Mode deactivated!", name_.c_str(), (i==0)?"Left":"Right");
            is_mouse_mode_on_[i] = false;

            tracker_poses_init_[i] = tracker_poses_local[i];
            if(i == 0 && !control_left_ee_name_.empty())       ee_data_[control_left_ee_name_].setInit();
            else if(i == 1 && !control_right_ee_name_.empty()) ee_data_[control_right_ee_name_].setInit();
        }
    }

    if(!control_left_ee_name_.empty()) // left vive controller
    {
        Eigen::Affine3d target_pose_diff;
        Eigen::Vector6d target_vel;
        target_pose_diff.setIdentity();
        target_vel.setZero();
        if(is_mouse_mode_on_[0])
        {
            Eigen::Affine3d tracker_pose_diff = tracker_poses_init_[0].inverse() * tracker_poses_local[0];

            const Eigen::Matrix3d& R_h2r     = tracker_base2robot_base_[0];
            const Eigen::Matrix3d& R_init    = tracker_poses_init_[0].linear();
            const Eigen::Matrix3d& R_ee_init = ee_data_[control_left_ee_name_].x_init.linear();

            const Eigen::Vector3d delta_pos = tracker_pos_multiplier_
                * R_ee_init.transpose() * R_h2r * R_init * tracker_pose_diff.translation();

            Eigen::Matrix3d rot_diff_ee_body = Eigen::Matrix3d::Identity();
            if(move_ori_)
            {
                const Eigen::AngleAxisd aa(tracker_pose_diff.linear());
                const Eigen::Matrix3d R_diff_scaled = (std::abs(aa.angle()) > 1e-10)
                    ? Eigen::AngleAxisd(tracker_ori_multiplier_ * aa.angle(), aa.axis()).toRotationMatrix()
                    : Eigen::Matrix3d::Identity();
                rot_diff_ee_body = R_ee_init.transpose() * R_h2r * R_diff_scaled * R_h2r.transpose() * R_ee_init;
            }

            target_pose_diff.translation() = delta_pos;
            target_pose_diff.linear()      = rot_diff_ee_body;
        }
        
        ee_data_[control_left_ee_name_].x_desired = ee_data_[control_left_ee_name_].x_init * target_pose_diff;
        ee_data_[control_left_ee_name_].xdot_desired  = target_vel;
    }

    if(!control_right_ee_name_.empty()) // right vive controller
    {
        Eigen::Affine3d target_pose_diff;
        Eigen::Vector6d target_vel;
        target_pose_diff.setIdentity();
        target_vel.setZero();
        if(is_mouse_mode_on_[1])
        {
            Eigen::Affine3d tracker_pose_diff = tracker_poses_init_[1].inverse() * tracker_poses_local[1];

            const Eigen::Matrix3d& R_h2r     = tracker_base2robot_base_[1];
            const Eigen::Matrix3d& R_init    = tracker_poses_init_[1].linear();
            const Eigen::Matrix3d& R_ee_init = ee_data_[control_right_ee_name_].x_init.linear();

            const Eigen::Vector3d delta_pos = tracker_pos_multiplier_
                * R_ee_init.transpose() * R_h2r * R_init * tracker_pose_diff.translation();

            Eigen::Matrix3d rot_diff_ee_body = Eigen::Matrix3d::Identity();
            if(move_ori_)
            {
                const Eigen::AngleAxisd aa(tracker_pose_diff.linear());
                const Eigen::Matrix3d R_diff_scaled = (std::abs(aa.angle()) > 1e-10)
                    ? Eigen::AngleAxisd(tracker_ori_multiplier_ * aa.angle(), aa.axis()).toRotationMatrix()
                    : Eigen::Matrix3d::Identity();
                rot_diff_ee_body = R_ee_init.transpose() * R_h2r * R_diff_scaled * R_h2r.transpose() * R_ee_init;
            }

            target_pose_diff.translation() = delta_pos;
            target_pose_diff.linear()      = rot_diff_ee_body;
        }
        
        ee_data_[control_right_ee_name_].x_desired = ee_data_[control_right_ee_name_].x_init * target_pose_diff;
        ee_data_[control_right_ee_name_].xdot_desired  = target_vel;
    }

    bool is_qp_solved = true;
    std::string time_verbose = "";
    switch (control_mode_)
    {
        case 0: // CLIK
            fr3_model_updater_.robot_controller_->CLIKStep(ee_data_, fr3_model_updater_.qdot_desired_total_);
            fr3_model_updater_.q_desired_total_ = fr3_model_updater_.q_total_ +
                                                  fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;
            
            fr3_model_updater_.torque_desired_total_ = fr3_model_updater_.robot_controller_->moveJointTorqueStep(fr3_model_updater_.q_desired_total_,
                                                                                                                 fr3_model_updater_.qdot_desired_total_,
                                                                                                                 false);
            break;
        case 1: // OSF
            fr3_model_updater_.robot_controller_->OSFStep(ee_data_, fr3_model_updater_.torque_desired_total_);
            break;
        case 2: // QPIK
            is_qp_solved = fr3_model_updater_.robot_controller_->QPIKStep(ee_data_, fr3_model_updater_.qdot_desired_total_, time_verbose);
            if(!is_qp_solved) fr3_model_updater_.qdot_desired_total_.setZero();
            fr3_model_updater_.q_desired_total_ = fr3_model_updater_.q_total_ +
                                                  fr3_model_updater_.dt_ * fr3_model_updater_.qdot_desired_total_;
            fr3_model_updater_.torque_desired_total_ = fr3_model_updater_.robot_controller_->moveJointTorqueStep(fr3_model_updater_.q_desired_total_,
                                                                                                                 fr3_model_updater_.qdot_desired_total_,
                                                                                                                 false);
            break;
        case 3: // QPID
            is_qp_solved = fr3_model_updater_.robot_controller_->QPIDStep(ee_data_, fr3_model_updater_.torque_desired_total_, time_verbose);
            if(!is_qp_solved) fr3_model_updater_.torque_desired_total_ = fr3_model_updater_.robot_data_->getGravity();
            break;
        default:
            break;
    }

    fr3_model_updater_.writeCommand(fr3_model_updater_.torque_desired_total_ - fr3_model_updater_.g_total_); // robot_controller automatically add gravity force

    auto fb = std::make_shared<ActionT::Feedback>();
    fb->is_qp_solved = is_qp_solved;
    fb->time_verbose = time_verbose;
    publishFeedback(fb);

    return ComputeResult::RUNNING;
}

void ViveTracker::onStop(StopReason reason)
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

ViveTracker::ResultPtr ViveTracker::makeResult(StopReason reason)
{
    auto result = std::make_shared<ActionT::Result>();
    result->is_completed = true;
    return result;
}

void ViveTracker::subPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if(msg->poses.size() != 3)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Size of PoseArray for tracker_pose (%ld) does not equal to 3.", name_.c_str(), msg->poses.size());
    }
    else
    {
        for(size_t i = 0; i < msg->poses.size(); ++i)
        {
            Eigen::Vector3d position(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);
            position = dyros_math::lowPassFilter(position, tracker_poses_[i].translation(), 0.001, 0.002);
            Eigen::Quaterniond quaternion(msg->poses[i].orientation.w, msg->poses[i].orientation.x, msg->poses[i].orientation.y, msg->poses[i].orientation.z);
            quaternion.normalize();
            Eigen::Matrix3d orientation = quaternion.toRotationMatrix();
            {
                std::lock_guard<std::mutex> lock(tracker_pose_mutex_);
                tracker_poses_[i].translation() = position;
                tracker_poses_[i].linear() = orientation;
            }
        }
    }
}

void ViveTracker::subLButtonCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if(msg->data.size() != 4)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Size of Int32MultiArray for lhand_button (%ld) does not equal to 4.", name_.c_str(), msg->data.size());
    }
    else
    {
        for(size_t i = 0; i < msg->data.size(); ++i)
        {
            std::lock_guard<std::mutex> lock(button_state_mutex_);
            button_states_[0][i] = (static_cast<int>(msg->data[i]) == 0) ? false : true;
        }

    }
}

void ViveTracker::subRButtonCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if(msg->data.size() != 4)
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Size of Int32MultiArray for rhand_button (%ld) does not equal to 4.", name_.c_str(), msg->data.size());
    }
    else
    {
        for(size_t i = 0; i < msg->data.size(); ++i)
        {
            std::lock_guard<std::mutex> lock(button_state_mutex_);
            button_states_[1][i] = (static_cast<int>(msg->data[i]) == 0) ? false : true;
        }

    }
}


// Register this server into global registry (executed when this TU is linked)
REGISTER_FR3_ACTION_SERVER(ViveTracker, "fr3_vive_tracker")

}  // namespace fr3_husky_controller::servers::fr3
/*
# send goal 
ros2 action send_goal /fr3_vive_tracker fr3_husky_msgs/action/ViveTracker \
"{mode: 1, left_ee_name: 'left_fr3_hand_tcp', right_ee_name: 'right_fr3_hand_tcp', move_orientation: false, tracker_pos_multiplier: 1.0, tracker_ori_multiplier: 1.0}" \
--feedback
*/