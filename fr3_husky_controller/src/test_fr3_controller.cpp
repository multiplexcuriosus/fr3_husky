#include "fr3_husky_controller/test_fr3_controller.hpp"

#include <unordered_set>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>

namespace fr3_husky_controller
{
controller_interface::InterfaceConfiguration TestFr3Controller::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint_name : params_.manipulator_joints)
    {
        for (const auto& interface_type : params_.manipulator_state_interfaces)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }

    if (!use_pin_)
    {
        for (size_t i = 0; i < num_robots_; ++i)
        {
            for (const auto& name : franka_robot_model_[i]->get_state_interface_names())
            {
                conf.names.push_back(name);
            }
            conf.names.push_back(params_.robot_name[i] + "_" + arm_id_ + "/robot_time");
        }
    }

    return conf;
}

controller_interface::InterfaceConfiguration TestFr3Controller::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint_name : params_.manipulator_joints)
    {
        conf.names.push_back(joint_name + "/" + params_.manipulator_command_interface);
    }

    return conf;
}

CallbackReturn TestFr3Controller::on_init()
{
    try
    {
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s\n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn TestFr3Controller::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (param_listener_->is_old(params_))
    {
        params_ = param_listener_->get_params();
        LOGI(get_node(), "Parameters were updated");
    }

    num_robots_ = params_.robot_name.size();
    if (num_robots_ < 1 || num_robots_ > 2)
    {
        LOGE(get_node(), "FR3 controller expects one or two FR3 robots, but got %zu.", num_robots_);
        return CallbackReturn::FAILURE;
    }

    const std::unordered_set<std::string> allowed = {"left", "right"};
    std::unordered_set<std::string> seen;
    for (const auto& name : params_.robot_name)
    {
        if (allowed.find(name) == allowed.end())
        {
            LOGE(get_node(), "Invalid robot_name '%s'. Allowed values are 'left' or 'right'.", name.c_str());
            return CallbackReturn::FAILURE;
        }
        if (!seen.insert(name).second)
        {
            LOGE(get_node(), "Duplicate robot_name '%s' detected.", name.c_str());
            return CallbackReturn::FAILURE;
        }
    }
    robot_names_ = params_.robot_name;

    manipulator_dof_ = params_.manipulator_joints.size();
    if (manipulator_dof_ != FR3_DOF * num_robots_)
    {
        LOGE(get_node(), "FR3 controller expects %zu manipulator DoF, but got %zu.",
             FR3_DOF * num_robots_, manipulator_dof_);
        return CallbackReturn::FAILURE;
    }

    has_position_command_interface_ = (params_.manipulator_command_interface == allowed_interface_types_[0]);
    has_velocity_command_interface_ = (params_.manipulator_command_interface == allowed_interface_types_[1]);
    has_effort_command_interface_   = (params_.manipulator_command_interface == allowed_interface_types_[2]);

    has_position_state_interface_ = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[0]);
    has_velocity_state_interface_ = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[1]);
    has_effort_state_interface_   = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[2]);

    if (has_velocity_command_interface_ && (!has_velocity_state_interface_ || !has_position_state_interface_))
    {
        LOGE(get_node(), "'velocity' command interface requires 'velocity' and 'position' state interfaces.");
        return CallbackReturn::FAILURE;
    }
    if (has_effort_command_interface_ && (!has_velocity_state_interface_ || !has_position_state_interface_))
    {
        LOGE(get_node(), "'effort' command interface requires 'velocity' and 'position' state interfaces.");
        return CallbackReturn::FAILURE;
    }

    if (get_update_rate() == 0)
    {
        throw std::runtime_error("Controller's update rate is set to 0.");
    }
    dt_ = 1.0 / static_cast<double>(get_update_rate());

    auto tmp_node = rclcpp::Node::make_shared("_tmp_urdf_client_" + std::string(get_node()->get_name()));
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(tmp_node, "controller_manager");
    param_client->wait_for_service();
    auto cm_params = param_client->get_parameters({"robot_description"});
    std::string urdf_xml = cm_params[0].value_to_string();

    // When using MuJoCo, do not use franka sementic segment
    use_pin_ = (urdf_xml.find("mujoco_ros_hardware/MujocoHardwareInterface") != std::string::npos);
    LOGW(get_node(), "Franka model: %s", !use_pin_ ? "available" : "unavailable (pinocchio fallback)");

    franka_robot_model_.clear();
    if (!use_pin_)
    {
        for (const auto& name : params_.robot_name)
        {
            franka_robot_model_.push_back(
                std::make_unique<franka_semantic_components::FrankaRobotModel>(
                    name + "_" + arm_id_ + "/robot_model",
                    name + "_" + arm_id_ + "/robot_state"));
        }
    }

    pinocchio::urdf::buildModelFromXML(urdf_xml, pin_model_);
    pin_data_ = pinocchio::Data(pin_model_);

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

    ee_names_.clear();
    for (const auto& name : params_.robot_name)
    {
        const std::string hand_tcp = name + "_" + arm_id_ + "_hand_tcp";
        const std::string link8    = name + "_" + arm_id_ + "_link8";
        const bool has_hand = pin_model_.getFrameId(hand_tcp) < static_cast<pinocchio::FrameIndex>(pin_model_.nframes);
        ee_names_.push_back(has_hand ? hand_tcp : link8);
    }

    q_pin_    = Eigen::VectorXd::Zero(pin_model_.nq);
    qdot_pin_ = Eigen::VectorXd::Zero(pin_model_.nv);
    J_full_   = Eigen::MatrixXd::Zero(6, pin_model_.nv);
    for (size_t i = 0; i < robot_names_.size(); ++i)
    {
        const std::string& rn = robot_names_[i];
        for (int j = 0; j < FR3_DOF; ++j)
        {
            const std::string jname = rn + "_" + arm_id_ + "_joint" + std::to_string(j + 1);
            const pinocchio::JointIndex jid = pin_model_.getJointId(jname);
            pin_q_idx_[rn][j] = pin_model_.joints[jid].idx_q();
            pin_v_idx_[rn][j] = pin_model_.joints[jid].idx_v();
        }
        pin_link0_id_[rn] = pin_model_.getFrameId(rn + "_" + arm_id_ + "_link0");
    }
    for (size_t i = 0; i < ee_names_.size(); ++i)
    {
        pin_ee_id_[ee_names_[i]]        = pin_model_.getFrameId(ee_names_[i]);
        pin_ee_robot_idx_[ee_names_[i]] = i;
    }

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

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestFr3Controller::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    std::vector<std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>> state_by_type(
        allowed_interface_types_.size());
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_by_type;

    for (const auto& interface : params_.manipulator_state_interfaces)
    {
        auto it = std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
        if (it == allowed_interface_types_.end()) continue;
        size_t idx = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
        if (!controller_interface::get_ordered_interfaces(
                state_interfaces_, params_.manipulator_joints, interface, state_by_type[idx]))
        {
            LOGE(get_node(), "Expected %zu '%s' state interfaces, got %zu.",
                 manipulator_dof_, interface.c_str(), state_by_type[idx].size());
            return CallbackReturn::ERROR;
        }
    }

    const auto& cmd_interface = params_.manipulator_command_interface;
    auto it = std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), cmd_interface);
    if (it == allowed_interface_types_.end())
    {
        LOGE(get_node(), "Invalid command_interface '%s'.", cmd_interface.c_str());
        return CallbackReturn::ERROR;
    }
    if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, params_.manipulator_joints, cmd_interface, cmd_by_type))
    {
        LOGE(get_node(), "Expected %zu '%s' command interfaces, got %zu.",
             manipulator_dof_, cmd_interface.c_str(), cmd_by_type.size());
        return CallbackReturn::ERROR;
    }

    registered_manipulator_joint_handles_.clear();
    for (size_t i = 0; i < manipulator_dof_; ++i)
    {
        registered_manipulator_joint_handles_.emplace_back(cmd_by_type[i]);
        auto& handle = registered_manipulator_joint_handles_.back();
        handle.state.clear();

        auto default_state_ref = state_by_type[kPositionIndex][i];
        handle.state.assign(allowed_interface_types_.size(), default_state_ref);

        if (has_position_state_interface_) handle.state[kPositionIndex] = state_by_type[kPositionIndex][i];
        if (has_velocity_state_interface_) handle.state[kVelocityIndex] = state_by_type[kVelocityIndex][i];
        if (has_effort_state_interface_ && !state_by_type[kEffortIndex].empty())
            handle.state[kEffortIndex] = state_by_type[kEffortIndex][i];
    }

    if (!use_pin_)
    {
        try
        {
            for (const auto& model : franka_robot_model_)
            {
                model->assign_loaned_state_interfaces(state_interfaces_);
            }
        }
        catch (const std::exception& e)
        {
            LOGW(get_node(), "Franka semantic component state interfaces not available (%s). Falling back to pinocchio.", e.what());
            use_pin_ = true;
        }
    }

    is_halted_ = false;
    updateJointStates();
    updateRobotData();
    setInitfromCurrent();

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TestFr3Controller::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (!is_halted_)
    {
        haltCommands();
        is_halted_ = true;
    }
    halt_initialized_ = false;
    registered_manipulator_joint_handles_.clear();
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TestFr3Controller::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        if (!is_halted_)
        {
            haltCommands();
            is_halted_ = true;
        }
        return controller_interface::return_type::OK;
    }

    updateJointStates();
    updateRobotData();

    // -------------------------------------------------------------------------
    // TODO: implement your own controller algorithm
    // -------------------------------------------------------------------------

    if (has_position_command_interface_)
    {
        for (size_t i = 0; i < num_robots_; ++i)
        {
            q_desired_[robot_names_[i]] = q_init_[robot_names_[i]];
            q_desired_total_.segment(i * FR3_DOF, FR3_DOF) = q_desired_[robot_names_[i]];
        }
        writeCommandInterfaces(q_desired_total_);
    }
    else if (has_velocity_command_interface_)
    {
        for (size_t i = 0; i < num_robots_; ++i)
        {
            qdot_desired_[robot_names_[i]].setZero();
            qdot_desired_total_.segment(i * FR3_DOF, FR3_DOF) = qdot_desired_[robot_names_[i]];
        }
        writeCommandInterfaces(qdot_desired_total_);
    }
    else
    {
        for (size_t i = 0; i < num_robots_; ++i)
        {
            torque_desired_[robot_names_[i]].setZero();
            torque_desired_total_.segment(i * FR3_DOF, FR3_DOF) = torque_desired_[robot_names_[i]];
        }
        writeCommandInterfaces(torque_desired_total_);
    }

    // -------------------------------------------------------------------------

    return controller_interface::return_type::OK;
}

void TestFr3Controller::haltCommands()
{
    if (registered_manipulator_joint_handles_.empty()) return;

    if (!halt_initialized_)
    {
        halt_position_.clear();
        for (size_t i = 0; i < registered_manipulator_joint_handles_.size(); ++i)
        {
            const std::string jname = registered_manipulator_joint_handles_[i].command.get().get_name();
            if (jname.find(arm_id_) != std::string::npos && jname.find("joint") != std::string::npos)
            {
                halt_position_[jname] = registered_manipulator_joint_handles_[i].state[kPositionIndex].get().get_value();
            }
        }
        halt_initialized_ = true;
    }

    if (has_position_command_interface_)
    {
        for (size_t i = 0; i < registered_manipulator_joint_handles_.size(); ++i)
        {
            const std::string jname = registered_manipulator_joint_handles_[i].command.get().get_name();
            auto it = halt_position_.find(jname);
            if (it != halt_position_.end())
                registered_manipulator_joint_handles_[i].command.get().set_value(it->second);
        }
    }
    else if (has_velocity_command_interface_)
    {
        for (auto& h : registered_manipulator_joint_handles_) h.command.get().set_value(0.0);
    }
    else  // effort
    {
        const Eigen::Vector<double, FR3_DOF> kp{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
        const Eigen::Vector<double, FR3_DOF> kv{ 30.0,  30.0,  30.0,  30.0,  10.0,  10.0,  5.0};

        for (size_t i = 0; i < registered_manipulator_joint_handles_.size(); ++i)
        {
            const std::string jname = registered_manipulator_joint_handles_[i].command.get().get_name();
            auto it = halt_position_.find(jname);
            if (it == halt_position_.end()) continue;

            int joint_idx = -1;
            for (int j = 1; j <= static_cast<int>(FR3_DOF); ++j)
            {
                if (jname.find(std::to_string(j)) != std::string::npos)
                    joint_idx = j - 1;
            }
            if (joint_idx < 0)
            {
                registered_manipulator_joint_handles_[i].command.get().set_value(0.0);
                continue;
            }
            const double q_halted  = it->second;
            const double q_curr    = registered_manipulator_joint_handles_[i].state[kPositionIndex].get().get_value();
            const double qdot_curr = has_velocity_state_interface_
                ? registered_manipulator_joint_handles_[i].state[kVelocityIndex].get().get_value() : 0.0;
            registered_manipulator_joint_handles_[i].command.get().set_value(
                kp[joint_idx] * (q_halted - q_curr) - kv[joint_idx] * qdot_curr);
        }
    }
}

void TestFr3Controller::writeCommandInterfaces(const Eigen::VectorXd& command)
{
    if (static_cast<size_t>(command.size()) == manipulator_dof_)
    {
        for (size_t i = 0; i < registered_manipulator_joint_handles_.size(); ++i)
            registered_manipulator_joint_handles_[i].command.get().set_value(command(i));
    }
    else
    {
        LOGW(get_node(), "Command size mismatch (expected %zu, got %zu). Holding.", manipulator_dof_, command.size());
        if (has_position_command_interface_)
        {
            for (size_t i = 0; i < registered_manipulator_joint_handles_.size(); ++i)
            {
                const double cur = registered_manipulator_joint_handles_[i].state[kPositionIndex].get().get_value();
                registered_manipulator_joint_handles_[i].command.get().set_value(cur);
            }
        }
        else
        {
            for (auto& h : registered_manipulator_joint_handles_) h.command.get().set_value(0.0);
        }
    }
}

void TestFr3Controller::updateJointStates()
{
    const Eigen::VectorXd last_qdot_total = qdot_total_;
    for (size_t i = 0; i < manipulator_dof_; ++i)
    {
        if (has_position_state_interface_) q_total_(i)      = registered_manipulator_joint_handles_[i].state[kPositionIndex].get().get_value();
        if (has_velocity_state_interface_) qdot_total_(i)   = registered_manipulator_joint_handles_[i].state[kVelocityIndex].get().get_value();
        if (has_effort_state_interface_)   torque_total_(i) = registered_manipulator_joint_handles_[i].state[kEffortIndex].get().get_value();
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

void TestFr3Controller::updateRobotData()
{
    play_time_ = get_node()->now().seconds();

    if (use_pin_)
    {
        q_pin_.setZero();
        qdot_pin_.setZero();
        for (size_t i = 0; i < num_robots_; ++i)
        {
            const std::string& rn    = robot_names_[i];
            const auto& q_idx = pin_q_idx_.at(rn);
            const auto& v_idx = pin_v_idx_.at(rn);
            for (int j = 0; j < FR3_DOF; ++j)
            {
                q_pin_(q_idx[j])    = q_[rn](j);
                qdot_pin_(v_idx[j]) = qdot_[rn](j);
            }
        }

        pinocchio::forwardKinematics(pin_model_, pin_data_, q_pin_, qdot_pin_);
        pinocchio::updateFramePlacements(pin_model_, pin_data_);
        pinocchio::computeJointJacobians(pin_model_, pin_data_, q_pin_);
        pinocchio::crba(pin_model_, pin_data_, q_pin_);
        pin_data_.M.triangularView<Eigen::StrictlyLower>() =
            pin_data_.M.transpose().triangularView<Eigen::StrictlyLower>();
        pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, q_pin_);
        pinocchio::nonLinearEffects(pin_model_, pin_data_, q_pin_, qdot_pin_);

        for (size_t i = 0; i < num_robots_; ++i)
        {
            const std::string& robot_name = robot_names_[i];
            const auto& v_idx = pin_v_idx_.at(robot_name);

            Eigen::Matrix<double, FR3_DOF, FR3_DOF> M_i;
            Eigen::Matrix<double, FR3_DOF, 1> g_i, c_i;
            for (int a = 0; a < FR3_DOF; ++a)
            {
                g_i(a) = pin_data_.g(v_idx[a]);
                c_i(a) = pin_data_.nle(v_idx[a]) - pin_data_.g(v_idx[a]);
                for (int b = 0; b < FR3_DOF; ++b)
                    M_i(a, b) = pin_data_.M(v_idx[a], v_idx[b]);
            }
            {
                std::lock_guard<std::mutex> lock(robot_data_mutex_);
                M_[robot_name]     = M_i;
                M_inv_[robot_name] = M_i.inverse();
                g_[robot_name]     = g_i;
                c_[robot_name]     = c_i;
                M_total_.block(FR3_DOF * i, FR3_DOF * i, FR3_DOF, FR3_DOF)     = M_i;
                M_inv_total_.block(FR3_DOF * i, FR3_DOF * i, FR3_DOF, FR3_DOF) = M_i.inverse();
                g_total_.segment(FR3_DOF * i, FR3_DOF) = g_i;
                c_total_.segment(FR3_DOF * i, FR3_DOF) = c_i;
            }
        }

        for (const auto& ee_name : ee_names_)
        {
            const size_t       i          = pin_ee_robot_idx_.at(ee_name);
            const std::string& robot_name = robot_names_[i];
            const auto&        v_idx      = pin_v_idx_.at(robot_name);
            const pinocchio::FrameIndex ee_id    = pin_ee_id_.at(ee_name);
            const pinocchio::FrameIndex link0_id = pin_link0_id_.at(robot_name);

            const pinocchio::SE3& T_w_ee    = pin_data_.oMf[ee_id];
            const pinocchio::SE3& T_w_link0 = pin_data_.oMf[link0_id];
            const pinocchio::SE3  T_link0_ee = T_w_link0.actInv(T_w_ee);

            J_full_.setZero();
            pinocchio::getFrameJacobian(pin_model_, pin_data_, ee_id,
                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full_);

            const Eigen::Matrix3d R = T_w_link0.rotation().transpose();
            Eigen::Matrix<double, 6, FR3_DOF> J_link0;
            for (int j = 0; j < FR3_DOF; ++j)
                J_link0.col(j) = J_full_.col(v_idx[j]);
            J_link0.topRows(3)    = R * J_link0.topRows(3);
            J_link0.bottomRows(3) = R * J_link0.bottomRows(3);

            {
                std::lock_guard<std::mutex> lock(robot_data_mutex_);
                x_[ee_name].matrix() = T_link0_ee.toHomogeneousMatrix();
                J_[ee_name]            = J_link0;
                xdot_[ee_name]       = J_link0 * qdot_[robot_name];
            }
        }

    }
    else
    {
        M_total_.setZero(manipulator_dof_, manipulator_dof_);
        for (size_t i = 0; i < num_robots_; ++i)
        {
            const std::string& robot_name = robot_names_[i];
            std::array<double, FR3_DOF * FR3_DOF> mass     = franka_robot_model_[i]->getMassMatrix();
            std::array<double, FR3_DOF>            coriolis = franka_robot_model_[i]->getCoriolisForceVector();
            std::array<double, FR3_DOF>            gravity  = franka_robot_model_[i]->getGravityForceVector();
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
                    std::array<double, 16> pose = franka_robot_model_[i]->getPoseMatrix(franka::Frame::kEndEffector);
                    std::array<double, 42> jac  = franka_robot_model_[i]->getZeroJacobian(franka::Frame::kEndEffector);
                    x_[ee_name].matrix() = Eigen::Map<const Eigen::Matrix4d>(pose.data());
                    J_[ee_name] = Eigen::Map<const Eigen::Matrix<double, 6, FR3_DOF, Eigen::ColMajor>>(jac.data());
                    xdot_[ee_name] = J_[ee_name] * qdot_[robot_names_[i]];
                    break;
                }
            }
        }
    }
}

void TestFr3Controller::setInitfromCurrent()
{
    q_init_    = q_;
    qdot_init_ = qdot_;
    qddot_init_ = qdot_;
    q_total_init_    = q_total_;
    qdot_total_init_ = qdot_total_;
    qddot_total_init_ = qdot_total_;
    x_init_    = x_;
    xdot_init_ = xdot_;
    control_start_time_ = play_time_;
}

}  // namespace fr3_husky_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_husky_controller::TestFr3Controller, controller_interface::ControllerInterface)
