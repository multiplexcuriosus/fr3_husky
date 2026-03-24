#include "fr3_husky_controller/model/fr3_model_updater.hpp"

namespace fr3_husky_controller
{

bool FR3ModelUpdater::initialize(size_t num_robots,
                                 size_t manipulator_dof,
                                 double dt,
                                 const std::vector<std::string>& robot_names,
                                 const std::vector<std::string>& ee_names)
{
    if (!ModelUpdaterBase::initialize(num_robots, manipulator_dof, dt, robot_names, ee_names))
    {
        return false;
    }

    // Allocate manipulator state buffers
    for (const auto& robot_name : robot_names_)
    {
        q_init_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        qdot_init_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        qddot_init_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        q_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        qdot_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        qddot_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        torque_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        q_desired_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        qdot_desired_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        qddot_desired_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        torque_desired_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        M_[robot_name] = Eigen::Matrix<double, FR3_DOF, FR3_DOF>::Zero();
        M_inv_[robot_name] = Eigen::Matrix<double, FR3_DOF, FR3_DOF>::Zero();
        c_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
        g_[robot_name] = Eigen::Matrix<double, FR3_DOF, 1>::Zero();
    }

    q_total_init_.setZero(manipulator_dof_);
    qdot_total_init_.setZero(manipulator_dof_);
    qddot_total_init_.setZero(manipulator_dof_);
    q_total_.setZero(manipulator_dof_);
    qdot_total_.setZero(manipulator_dof_);
    qddot_total_.setZero(manipulator_dof_);
    torque_total_.setZero(manipulator_dof_);
    q_desired_total_.setZero(manipulator_dof_);
    qdot_desired_total_.setZero(manipulator_dof_);
    qddot_desired_total_.setZero(manipulator_dof_);
    torque_desired_total_.setZero(manipulator_dof_);
    M_total_.setZero(manipulator_dof_, manipulator_dof_);
    M_inv_total_.setZero(manipulator_dof_, manipulator_dof_);
    c_total_.setZero(manipulator_dof_);
    g_total_.setZero(manipulator_dof_);

    for (const auto& ee_name : ee_names_)
    {
        x_init_[ee_name]     = Eigen::Affine3d::Identity();
        xdot_init_[ee_name]  = Eigen::Vector6d::Zero();
        x_[ee_name]          = Eigen::Affine3d::Identity();
        xdot_[ee_name]       = Eigen::Vector6d::Zero();
        J_[ee_name]          = Eigen::Matrix<double, 6, FR3_DOF>::Zero();
        x_desired_[ee_name]  = Eigen::Affine3d::Identity();
        xdot_desired_[ee_name] = Eigen::Vector6d::Zero();
    }

    return true;
}

void FR3ModelUpdater::updateJointStates()
{
    if (!is_configured_ || !getHandlesReady()) return;

    const Eigen::VectorXd last_qdot_total = qdot_total_;
    for (size_t i = 0; i < manipulator_dof_; ++i)
    {
        if (has_position_state_interface_) q_total_(i)      = robot_handle_.mani_joints[i].state[kPositionIndex].get().get_value();
        if (has_velocity_state_interface_) qdot_total_(i)   = robot_handle_.mani_joints[i].state[kVelocityIndex].get().get_value();
        if (has_effort_state_interface_)   torque_total_(i) = robot_handle_.mani_joints[i].state[kEffortIndex].get().get_value();
    }
    qddot_total_ = (qdot_total_ - last_qdot_total) / dt_;

    for (size_t r = 0; r < num_robots_; ++r)
    {
        const std::string& robot_name = robot_names_[r];
        q_[robot_name]      = q_total_.segment(FR3_DOF * r, FR3_DOF);
        qdot_[robot_name]   = qdot_total_.segment(FR3_DOF * r, FR3_DOF);
        qddot_[robot_name]  = qddot_total_.segment(FR3_DOF * r, FR3_DOF);
        torque_[robot_name] = torque_total_.segment(FR3_DOF * r, FR3_DOF);
    }
}

void FR3ModelUpdater::updateRobotData()
{
    if (!is_configured_)
    {
        return;
    }

    if (!robot_data_)
    {
        if (node_) RCLCPP_WARN(node_->get_logger(), "DRC robot data pointer is null; skipping model update.");
        return;
    }

    robot_data_->updateState(q_total_, qdot_total_);

    if (franka_robot_model_)
    {
        M_total_.setZero(manipulator_dof_, manipulator_dof_);
        for (size_t i = 0; i < num_robots_; ++i)
        {
            const std::string& robot_name = robot_names_[i];
            std::array<double, FR3_DOF * FR3_DOF>  mass     = (*franka_robot_model_)[i]->getMassMatrix();
            std::array<double, FR3_DOF>            coriolis = (*franka_robot_model_)[i]->getCoriolisForceVector();
            std::array<double, FR3_DOF>            gravity  = (*franka_robot_model_)[i]->getGravityForceVector();
            {
                std::lock_guard<std::mutex> lock(robot_data_mutex_);
                M_[robot_name]     = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, FR3_DOF, Eigen::RowMajor>>(mass.data());
                g_[robot_name]     = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, 1>>(gravity.data());
                c_[robot_name]     = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, 1>>(coriolis.data());
                M_inv_[robot_name] = M_[robot_name].inverse();
                M_total_.block(FR3_DOF * i, FR3_DOF * i, FR3_DOF, FR3_DOF) = M_[robot_name];
                M_inv_total_.block(FR3_DOF * i, FR3_DOF * i, FR3_DOF, FR3_DOF) = M_inv_[robot_name];
                g_total_.segment(FR3_DOF * i, FR3_DOF) = g_[robot_name];
                c_total_.segment(FR3_DOF * i, FR3_DOF) = c_[robot_name];
            }
        }

        for (const auto& ee_name : ee_names_)
        {
            for (size_t i = 0; i < num_robots_; ++i)
            {
                if (ee_name.find(robot_names_[i]) != std::string::npos)
                {
                    std::array<double, 16> pose = (*franka_robot_model_)[i]->getPoseMatrix(franka::Frame::kEndEffector);
                    std::array<double, 42> jac  = (*franka_robot_model_)[i]->getZeroJacobian(franka::Frame::kEndEffector);
                    x_[ee_name].matrix() = Eigen::Map<const Eigen::Matrix4d>(pose.data());
                    J_[ee_name] = Eigen::Map<const Eigen::Matrix<double, 6, FR3_DOF, Eigen::ColMajor>>(jac.data());
                    xdot_[ee_name] = J_[ee_name] * qdot_[robot_names_[i]];
                    break;
                }
            }
        }
    }
    else
    {
        M_total_ = robot_data_->getMassMatrix();;
        M_inv_total_ = M_total_.inverse();
        g_total_ = robot_data_->getGravity();
        c_total_ = robot_data_->getCoriolis();

        for (size_t i = 0; i < num_robots_; ++i)
        {
            const std::string& robot_name = robot_names_[i];
            M_[robot_name]     = M_total_.block(FR3_DOF * i, FR3_DOF * i, FR3_DOF, FR3_DOF);
            M_inv_[robot_name] = M_inv_total_.block(FR3_DOF * i, FR3_DOF * i, FR3_DOF, FR3_DOF);
            g_[robot_name]     = g_total_.segment(FR3_DOF * i, FR3_DOF);
            c_[robot_name]     = c_total_.segment(FR3_DOF * i, FR3_DOF);
        }

        for (const auto& ee_name : ee_names_)
        {
            if(!robot_data_->hasLinkFrame(ee_name)) continue;
            x_[ee_name] = robot_data_->getPose(ee_name);
            const Eigen::MatrixXd J_total = robot_data_->getJacobian(ee_name);
            for (size_t i = 0; i < num_robots_; ++i)
            {
                if (ee_name.find(robot_names_[i]) != std::string::npos)
                {
                    J_[ee_name] = J_total.block(0, i * FR3_DOF, 6, FR3_DOF);
                    break;
                }
            }
            xdot_[ee_name] = robot_data_->getVelocity(ee_name);
        }
    }
}

void FR3ModelUpdater::setInitFromCurrent()
{
    if (!is_configured_) return;

    q_init_ = q_;
    qdot_init_ = qdot_;
    qddot_init_ = qddot_;
    q_total_init_ = q_total_;
    qdot_total_init_ = qdot_total_;
    qddot_total_init_ = qddot_total_;
    x_init_    = x_;
    xdot_init_ = xdot_;
}

void FR3ModelUpdater::writeCommand(const Eigen::VectorXd& command)
{
    if (!is_configured_ || !getHandlesReady())
    {
        return;
    }

    if (static_cast<size_t>(command.size()) == manipulator_dof_)
    {
        halt_initialized_ = false;  // leave halt mode; next halt will re-capture pose
        for (size_t i = 0; i < robot_handle_.mani_joints.size(); ++i)
        {
            const auto& h = robot_handle_.mani_joints[i];
            const int joint_idx = jointNameToIndex(h.command.get().get_name());
            Eigen::Index cmd_idx = static_cast<Eigen::Index>(i);
            if (joint_idx >= 0)
            {
                if (num_robots_ > 1)
                {
                    // Keep arm block offset in dual-arm mode (left/right jointN must not alias).
                    const Eigen::Index arm_block = static_cast<Eigen::Index>(i / FR3_DOF);
                    cmd_idx = arm_block * static_cast<Eigen::Index>(FR3_DOF) + static_cast<Eigen::Index>(joint_idx);
                }
                else
                {
                    cmd_idx = static_cast<Eigen::Index>(joint_idx);
                }
            }
            if (cmd_idx < 0 || cmd_idx >= command.size())
            {
                if (node_) RCLCPP_ERROR(node_->get_logger(),
                                        "Computed invalid command index (%ld) for joint '%s' with command size %zu. Halting.",
                                        static_cast<long>(cmd_idx),
                                        h.command.get().get_name().c_str(),
                                        static_cast<size_t>(command.size()));
                haltCommands();
                return;
            }
            robot_handle_.mani_joints[i].command.get().set_value(command(cmd_idx));
        }
    }
    else
    {
        if (node_) RCLCPP_WARN(node_->get_logger(),
                               "Manipulator cmd size mismatch (expected %zu, got %zu). Holding/zeroing.",
                               manipulator_dof_, static_cast<size_t>(command.size()));
        haltCommands();
    }
}

void FR3ModelUpdater::haltCommands()
{
    if (!getHandlesReady())
    {
        return;
    }

    if (!halt_initialized_)
    {
        halt_position_.clear();
        for (size_t i = 0; i < manipulator_dof_; ++i)
        {
            const std::string jname = robot_handle_.mani_joints[i].command.get().get_name();
            if(jname.find(arm_id_) != std::string::npos && jname.find("joint") != std::string::npos)
            {
                halt_position_[jname] = robot_handle_.mani_joints[i].state[kPositionIndex].get().get_value();
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Joint named [%s] does not enclude [%s_joint]. Halt command for [%s] is omitted!",
                            jname.c_str(), arm_id_.c_str(), jname.c_str());
            }
        }
        halt_initialized_ = true;
    }

    if (has_position_command_interface_)
    {
        for (size_t i = 0; i < robot_handle_.mani_joints.size(); ++i)
        {
            auto it = halt_position_.find(robot_handle_.mani_joints[i].command.get().get_name());
            if (it != halt_position_.end())
            {
                robot_handle_.mani_joints[i].command.get().set_value(it->second);
            }
        }
    }
    else if (has_velocity_command_interface_)
    {
        for (auto& h : robot_handle_.mani_joints) h.command.get().set_value(0.0);
    }
    else
    {
        const Eigen::Vector<double, FR3_DOF> kp{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
        const Eigen::Vector<double, FR3_DOF> kv{30.0,   30.0,  30.0,  30.0,  10.0,  10.0,  5.0};

        for (size_t i = 0; i < robot_handle_.mani_joints.size(); ++i)
        {
            auto it = halt_position_.find(robot_handle_.mani_joints[i].command.get().get_name());
            if (it != halt_position_.end())
            {
                const std::string jname = it->first;
                const double q_halted = it->second;
                
                int arm_idx = -1;
                for (int joint_idx = 1; joint_idx <= FR3_DOF; ++joint_idx)
                {
                    if (jname.find(std::to_string(joint_idx)) != std::string::npos)
                    {
                        arm_idx = joint_idx - 1;  // 0-based index
                    }
                }
                if(arm_idx < 0)
                {
                    RCLCPP_WARN(node_->get_logger(), "Joint named [%s] exceed %s joint index [0 to %zu]. Halt command for [%s] set as 0!",
                                jname.c_str(), arm_id_.c_str(), static_cast<size_t>(FR3_DOF - 1), jname.c_str());
                    robot_handle_.mani_joints[i].command.get().set_value(0.0);
                    continue;
                }

                const double q_curr = robot_handle_.mani_joints[i].state[kPositionIndex].get().get_value();
                const double qdot_curr = (has_velocity_state_interface_) ? robot_handle_.mani_joints[i].state[kVelocityIndex].get().get_value() : 0.0;

                robot_handle_.mani_joints[i].command.get().set_value(kp[arm_idx] * (q_halted - q_curr) - kv[arm_idx] * (qdot_curr));
            }
        }
    }
}

}  // namespace fr3_husky_controller
