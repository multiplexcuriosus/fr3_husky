#include "fr3_husky_controller/test_fr3_husky_controller.hpp"

#include <unordered_set>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>

namespace fr3_husky_controller
{
controller_interface::InterfaceConfiguration TestFr3HuskyController::state_interface_configuration() const
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

    for (const auto& joint_name : params_.left_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[0]);
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[1]);
    }
    for (const auto& joint_name : params_.right_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[0]);
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[1]);
    }

    return conf;
}

controller_interface::InterfaceConfiguration TestFr3HuskyController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint_name : params_.manipulator_joints)
    {
        conf.names.push_back(joint_name + "/" + params_.manipulator_command_interface);
    }

    for (const auto& joint_name : params_.left_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[1]);
    }
    for (const auto& joint_name : params_.right_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[1]);
    }

    return conf;
}

CallbackReturn TestFr3HuskyController::on_init()
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

CallbackReturn TestFr3HuskyController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (param_listener_->is_old(params_))
    {
        params_ = param_listener_->get_params();
        LOGI(get_node(), "Parameters were updated");
    }

    num_robots_ = params_.robot_name.size();
    if (num_robots_ < 1 || num_robots_ > 2)
    {
        LOGE(get_node(), "FR3 Husky controller expects one or two FR3 robots, but got %zu.", num_robots_);
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
        LOGE(get_node(), "FR3 Husky controller expects %zu manipulator DoF, but got %zu.",
             FR3_DOF * num_robots_, manipulator_dof_);
        return CallbackReturn::FAILURE;
    }

    if (params_.left_wheel_names.size() != params_.right_wheel_names.size())
    {
        LOGE(get_node(), "Number of left wheels (%zu) differs from right wheels (%zu).",
             params_.left_wheel_names.size(), params_.right_wheel_names.size());
        return CallbackReturn::FAILURE;
    }
    if (params_.left_wheel_names.size() != WHEEL_PER_SIDE)
    {
        LOGW(get_node(), "Expected %d wheels per side; got %zu.", WHEEL_PER_SIDE, params_.left_wheel_names.size());
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

    wheel_pos_.setZero();
    wheel_vel_.setZero();
    wheel_vel_desired_.setZero();

    ee_names_.clear();
    for (const auto& name : params_.robot_name)
    {
        const std::string hand_tcp = name + "_" + arm_id_ + "_hand_tcp";
        const std::string link8    = name + "_" + arm_id_ + "_link8";
        const bool has_hand = pin_model_.getFrameId(hand_tcp) < static_cast<pinocchio::FrameIndex>(pin_model_.nframes);
        ee_names_.push_back(has_hand ? hand_tcp : link8);
    }

    // Compute fixed T_footprint_link0 transforms
    {
        pinocchio::forwardKinematics(pin_model_, pin_data_, Eigen::VectorXd::Zero(pin_model_.nq));
        pinocchio::updateFramePlacements(pin_model_, pin_data_);

        pinocchio::FrameIndex footprint_id = static_cast<pinocchio::FrameIndex>(-1);
        for (pinocchio::FrameIndex fi = 0; fi < static_cast<pinocchio::FrameIndex>(pin_model_.nframes); ++fi)
        {
            if (pin_model_.frames[fi].name.find("footprint") != std::string::npos)
            {
                footprint_id = fi;
                break;
            }
        }
        if (footprint_id == static_cast<pinocchio::FrameIndex>(-1))
        {
            LOGE(get_node(), "No 'footprint' frame found in pinocchio model. T_base_link0 will be identity.");
        }
        const pinocchio::SE3 T_world_footprint = (footprint_id != static_cast<pinocchio::FrameIndex>(-1))
            ? pin_data_.oMf[footprint_id] : pinocchio::SE3::Identity();

        for (const auto& robot_name : robot_names_)
        {
            const std::string link0_name = robot_name + "_" + arm_id_ + "_link0";
            const pinocchio::FrameIndex link0_id = pin_model_.getFrameId(link0_name);
            const pinocchio::SE3 T_footprint_link0 = T_world_footprint.actInv(pin_data_.oMf[link0_id]);
            T_base_link0_[robot_name] = Eigen::Affine3d(T_footprint_link0.toHomogeneousMatrix());
        }
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

    base_pose_w_init_.setIdentity();
    base_vel_w_init_.setZero();
    base_vel_b_init_.setZero();
    base_pose_w_.setIdentity();
    base_vel_w_.setZero();
    base_vel_b_.setZero();
    base_pose_w_desired_.setIdentity();
    base_vel_w_desired_.setZero();
    base_vel_b_desired_.setZero();

    for (const auto& ee_name : ee_names_)
    {
        x_m_init_[ee_name]    = Eigen::Affine3d::Identity();
        xdot_m_init_[ee_name] = Eigen::Vector6d::Zero();
        x_w_init_[ee_name]    = Eigen::Affine3d::Identity();
        xdot_w_init_[ee_name] = Eigen::Vector6d::Zero();
        x_m_[ee_name]         = Eigen::Affine3d::Identity();
        xdot_m_[ee_name]      = Eigen::Vector6d::Zero();
        x_w_[ee_name]         = Eigen::Affine3d::Identity();
        xdot_w_[ee_name]      = Eigen::Vector6d::Zero();
        J_[ee_name]           = Eigen::Matrix<double, 6, FR3_DOF>::Zero();
        x_m_desired_[ee_name]    = Eigen::Affine3d::Identity();
        xdot_m_desired_[ee_name] = Eigen::Vector6d::Zero();
        x_w_desired_[ee_name]    = Eigen::Affine3d::Identity();
        xdot_w_desired_[ee_name] = Eigen::Vector6d::Zero();
    }

    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        "~/odom", rclcpp::SystemDefaultsQoS());
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::SystemDefaultsQoS());
    publish_rate_ = params_.publish_rate;
    mobi_state_pub_buf_.writeFromNonRT(std::make_pair(base_pose_w_, base_vel_b_));

    joy_msg_received_.store(false, std::memory_order_release);
    estop_button_pressed_.store(false, std::memory_order_release);
    estop_is_active_ = false;
    estop_button_index_warned_ = false;
    joy_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SystemDefaultsQoS(),
        std::bind(&TestFr3HuskyController::onJoyMessage, this, std::placeholders::_1));

    odom_timer_ = get_node()->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_),
        [this]() { this->publishFromMobileStateBuffer(); });

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestFr3HuskyController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
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

    auto configure_side = [this](const std::vector<std::string>& wheel_names,
                                  std::vector<WheelHandle>& registered_handles) -> bool
    {
        registered_handles.clear();
        registered_handles.reserve(wheel_names.size());
        for (const auto& wheel_name : wheel_names)
        {
            const auto pos_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                [&wheel_name, this](const auto& iface) {
                    return iface.get_prefix_name() == wheel_name &&
                           iface.get_interface_name() == allowed_interface_types_[0];
                });
            if (pos_handle == state_interfaces_.cend())
            {
                LOGE(get_node(), "Unable to obtain position state handle for %s", wheel_name.c_str());
                return false;
            }
            const auto vel_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                [&wheel_name, this](const auto& iface) {
                    return iface.get_prefix_name() == wheel_name &&
                           iface.get_interface_name() == allowed_interface_types_[1];
                });
            if (vel_handle == state_interfaces_.cend())
            {
                LOGE(get_node(), "Unable to obtain velocity state handle for %s", wheel_name.c_str());
                return false;
            }
            const auto cmd_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&wheel_name, this](const auto& iface) {
                    return iface.get_prefix_name() == wheel_name &&
                           iface.get_interface_name() == allowed_interface_types_[1];
                });
            if (cmd_handle == command_interfaces_.end())
            {
                LOGE(get_node(), "Unable to obtain command handle for %s", wheel_name.c_str());
                return false;
            }
            std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> sh;
            sh.reserve(2);
            sh.emplace_back(std::ref(*pos_handle));
            sh.emplace_back(std::ref(*vel_handle));
            registered_handles.emplace_back(WheelHandle{std::move(sh), std::ref(*cmd_handle)});
        }
        return true;
    };

    if (!configure_side(params_.left_wheel_names,  registered_left_wheel_handles_) ||
        !configure_side(params_.right_wheel_names, registered_right_wheel_handles_))
    {
        return CallbackReturn::ERROR;
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

controller_interface::CallbackReturn TestFr3HuskyController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (!is_halted_)
    {
        haltCommands();
        is_halted_ = true;
    }
    registered_left_wheel_handles_.clear();
    registered_right_wheel_handles_.clear();
    registered_manipulator_joint_handles_.clear();

    odom_timer_.reset();
    estop_is_active_ = false;
    estop_button_pressed_.store(false, std::memory_order_release);
    joy_msg_received_.store(false, std::memory_order_release);

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TestFr3HuskyController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
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

    mobi_state_pub_buf_.writeFromNonRT(std::make_pair(base_pose_w_, base_vel_b_));

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
        wheel_vel_desired_.setZero();
        writeCommandInterfaces(q_desired_total_, wheel_vel_desired_);
    }
    else if (has_velocity_command_interface_)
    {
        for (size_t i = 0; i < num_robots_; ++i)
        {
            qdot_desired_[robot_names_[i]].setZero();
            qdot_desired_total_.segment(i * FR3_DOF, FR3_DOF) = qdot_desired_[robot_names_[i]];
        }
        wheel_vel_desired_.setZero();
        writeCommandInterfaces(qdot_desired_total_, wheel_vel_desired_);
    }
    else
    {
        for (size_t i = 0; i < num_robots_; ++i)
        {
            torque_desired_[robot_names_[i]].setZero();
            torque_desired_total_.segment(i * FR3_DOF, FR3_DOF) = torque_desired_[robot_names_[i]];
        }
        wheel_vel_desired_.setZero();
        writeCommandInterfaces(torque_desired_total_, wheel_vel_desired_);
    }

    // -------------------------------------------------------------------------

    const bool estop_pressed = params_.use_estop &&
                               joy_msg_received_.load(std::memory_order_acquire) &&
                               estop_button_pressed_.load(std::memory_order_acquire);
    if (estop_pressed && !estop_is_active_)
    {
        LOGW(get_node(), "E-STOP for mobile base activated!");
    }
    else if (!estop_pressed && estop_is_active_)
    {
        LOGI(get_node(), "E-STOP for mobile base released!");
    }
    estop_is_active_ = estop_pressed;

    if (estop_is_active_)
    {
        for (const auto& h : registered_left_wheel_handles_)  h.command.get().set_value(0.0);
        for (const auto& h : registered_right_wheel_handles_) h.command.get().set_value(0.0);
    }

    return controller_interface::return_type::OK;
}

void TestFr3HuskyController::haltCommands()
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

    for (const auto& h : registered_left_wheel_handles_)  h.command.get().set_value(0.0);
    for (const auto& h : registered_right_wheel_handles_) h.command.get().set_value(0.0);
}

void TestFr3HuskyController::writeCommandInterfaces(const Eigen::VectorXd& command_mani, const Eigen::VectorXd& command_mobi)
{
    if (static_cast<size_t>(command_mani.size()) == manipulator_dof_)
    {
        for (size_t i = 0; i < registered_manipulator_joint_handles_.size(); ++i)
            registered_manipulator_joint_handles_[i].command.get().set_value(command_mani(i));
    }
    else
    {
        LOGW(get_node(), "Manipulator cmd size mismatch (expected %zu, got %zu). Holding.", manipulator_dof_, command_mani.size());
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

    if (static_cast<size_t>(command_mobi.size()) == mobile_dof_)
    {
        for (size_t i = 0; i < registered_left_wheel_handles_.size(); ++i)
            registered_left_wheel_handles_[i].command.get().set_value(command_mobi(0));
        for (size_t i = 0; i < registered_right_wheel_handles_.size(); ++i)
            registered_right_wheel_handles_[i].command.get().set_value(command_mobi(1));
    }
    else
    {
        LOGW(get_node(), "Mobile cmd size mismatch (expected %zu, got %zu). Zeroing.", mobile_dof_, command_mobi.size());
        for (auto& h : registered_left_wheel_handles_)  h.command.get().set_value(0.0);
        for (auto& h : registered_right_wheel_handles_) h.command.get().set_value(0.0);
    }
}

void TestFr3HuskyController::updateJointStates()
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

    double left_pos = 0.0, right_pos = 0.0, left_vel = 0.0, right_vel = 0.0;
    for (size_t i = 0; i < registered_left_wheel_handles_.size(); ++i)
    {
        left_pos  += registered_left_wheel_handles_[i].state[kFeedbackPositionIndex].get().get_value();
        right_pos += registered_right_wheel_handles_[i].state[kFeedbackPositionIndex].get().get_value();
        left_vel  += registered_left_wheel_handles_[i].state[kFeedbackVelocityIndex].get().get_value();
        right_vel += registered_right_wheel_handles_[i].state[kFeedbackVelocityIndex].get().get_value();
    }
    if (!registered_left_wheel_handles_.empty())
    {
        const double n = static_cast<double>(registered_left_wheel_handles_.size());
        left_pos /= n;  right_pos /= n;
        left_vel /= n;  right_vel /= n;
    }
    wheel_pos_ = Eigen::Vector2d(left_pos, right_pos);
    wheel_vel_ = Eigen::Vector2d(left_vel, right_vel);
}

void TestFr3HuskyController::updateRobotData()
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
                x_m_[ee_name].matrix() = T_link0_ee.toHomogeneousMatrix();
                J_[ee_name]            = J_link0;
                xdot_m_[ee_name]       = J_link0 * qdot_[robot_name];
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
                M_total_.block(FR3_DOF * i, FR3_DOF * i, FR3_DOF, FR3_DOF)     = M_[robot_name];
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
                    x_m_[ee_name].matrix() = Eigen::Map<const Eigen::Matrix4d>(pose.data());
                    J_[ee_name] = Eigen::Map<const Eigen::Matrix<double, 6, FR3_DOF, Eigen::ColMajor>>(jac.data());
                    xdot_m_[ee_name] = J_[ee_name] * qdot_[robot_names_[i]];
                    break;
                }
            }
        }
    }

    // Compute base velocity from wheel odometry
    const double wheel_radius     = params_.wheel_radius * params_.wheel_radius_multiplier;
    const double wheel_separation = params_.wheel_separation * params_.wheel_separation_multiplier;
    Eigen::Matrix<double, 3, 2> J_mobi;
    J_mobi << wheel_radius / 2.,                   wheel_radius / 2.,
              0.,                                  0.,
             -wheel_radius / wheel_separation,     wheel_radius / wheel_separation;
    base_vel_b_ = J_mobi * wheel_vel_;

    auto computeBasePose = [](const Eigen::Affine2d& cur, const Eigen::Vector3d& vel_b, double dt)
    {
        const double vx = vel_b(0), vy = vel_b(1), w = vel_b(2);
        const double x  = cur.translation()(0);
        const double y  = cur.translation()(1);
        const double th = std::atan2(cur.linear()(1, 0), cur.linear()(0, 0));
        const double dth = w * dt;
        double dx_w, dy_w;
        if (std::fabs(w) < 1e-6)
        {
            const double tm = th + 0.5 * dth;
            dx_w = (vx * std::cos(tm) - vy * std::sin(tm)) * dt;
            dy_w = (vx * std::sin(tm) + vy * std::cos(tm)) * dt;
        }
        else
        {
            const double s = std::sin(dth), c = std::cos(dth);
            const double A = s / w, B = (1.0 - c) / w;
            const double dx_b = A * vx - B * vy, dy_b = B * vx + A * vy;
            dx_w = std::cos(th) * dx_b - std::sin(th) * dy_b;
            dy_w = std::sin(th) * dx_b + std::cos(th) * dy_b;
        }
        Eigen::Affine2d out;
        out.setIdentity();
        out.translation() << x + dx_w, y + dy_w;
        out.linear() = Eigen::Rotation2Dd(th + dth).toRotationMatrix();
        return out;
    };
    base_pose_w_ = computeBasePose(base_pose_w_, base_vel_b_, dt_);

    base_vel_w_.head(2) = base_pose_w_.linear() * base_vel_b_.head(2);
    base_vel_w_(2) = base_vel_b_(2);

    const Eigen::Vector3d p_base_w{base_pose_w_.translation()(0), base_pose_w_.translation()(1), 0.0};
    const Eigen::Vector3d v_base_w{base_vel_w_(0), base_vel_w_(1), 0.0};
    const Eigen::Vector3d omega_w{0.0, 0.0, base_vel_w_(2)};

    Eigen::Affine3d T_world_base3D = Eigen::Affine3d::Identity();
    T_world_base3D.translation() = p_base_w;
    T_world_base3D.linear() = Eigen::AngleAxisd(
        Eigen::Rotation2Dd(base_pose_w_.linear()).angle(), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    for (const auto& ee_name : ee_names_)
    {
        for (size_t i = 0; i < num_robots_; ++i)
        {
            if (ee_name.find(robot_names_[i]) == std::string::npos) continue;
            const std::string& robot_name = robot_names_[i];
            const Eigen::Affine3d T_world_link0 = T_world_base3D * T_base_link0_.at(robot_name);
            const Eigen::Matrix3d R_wl0 = T_world_link0.linear();
            x_w_[ee_name] = T_world_link0 * x_m_[ee_name];
            const Eigen::Vector3d r_EE = x_w_[ee_name].translation() - p_base_w;
            xdot_w_[ee_name].head(3) = v_base_w + omega_w.cross(r_EE) + R_wl0 * xdot_m_[ee_name].head(3);
            xdot_w_[ee_name].tail(3) = omega_w + R_wl0 * xdot_m_[ee_name].tail(3);
            break;
        }
    }
}

void TestFr3HuskyController::setInitfromCurrent()
{
    q_init_    = q_;
    qdot_init_ = qdot_;
    qddot_init_ = qdot_;
    q_total_init_    = q_total_;
    qdot_total_init_ = qdot_total_;
    qddot_total_init_ = qdot_total_;
    base_pose_w_init_ = base_pose_w_;
    base_vel_w_init_  = base_vel_w_;
    base_vel_b_init_  = base_vel_b_;
    x_m_init_    = x_m_;
    xdot_m_init_ = xdot_m_;
    x_w_init_    = x_w_;
    xdot_w_init_ = xdot_w_;
    control_start_time_ = play_time_;
}

void TestFr3HuskyController::publishFromMobileStateBuffer()
{
    const auto s = *mobi_state_pub_buf_.readFromRT();
    const Eigen::Affine2d&  pose_w = s.first;
    const Eigen::Vector3d&  vel_b  = s.second;
    const double yaw = Eigen::Rotation2Dd(pose_w.linear()).angle();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    if (odometry_publisher_)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = get_node()->now();
        msg.header.frame_id = params_.odom_frame_id;
        msg.child_frame_id  = params_.base_frame_id;
        msg.pose.covariance.fill(0.0);
        msg.twist.covariance.fill(0.0);
        for (size_t i = 0; i < 6; ++i)
        {
            msg.pose.covariance[6 * i + i]  = params_.pose_covariance_diagonal[i];
            msg.twist.covariance[6 * i + i] = params_.twist_covariance_diagonal[i];
        }
        msg.pose.pose.position.x    = pose_w.translation()(0);
        msg.pose.pose.position.y    = pose_w.translation()(1);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        msg.twist.twist.linear.x    = vel_b(0);
        msg.twist.twist.angular.z   = vel_b(2);
        odometry_publisher_->publish(msg);
    }

    if (odometry_transform_publisher_)
    {
        tf2_msgs::msg::TFMessage tfm;
        tfm.transforms.resize(1);
        auto& t = tfm.transforms[0];
        t.header.stamp    = get_node()->now();
        t.header.frame_id = params_.odom_frame_id;
        t.child_frame_id  = params_.base_frame_id;
        t.transform.translation.x = pose_w.translation()(0);
        t.transform.translation.y = pose_w.translation()(1);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        odometry_transform_publisher_->publish(tfm);
    }
}

void TestFr3HuskyController::onJoyMessage(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    joy_msg_received_.store(true, std::memory_order_release);
    if (!msg)
    {
        estop_button_pressed_.store(false, std::memory_order_release);
        return;
    }
    const int64_t idx = params_.estop_button_index;
    if (idx < 0 || static_cast<size_t>(idx) >= msg->buttons.size())
    {
        if (!estop_button_index_warned_)
        {
            LOGW(get_node(), "Invalid estop_button_index=%ld (buttons size=%zu). Treating as released.",
                 static_cast<long>(idx), msg->buttons.size());
            estop_button_index_warned_ = true;
        }
        estop_button_pressed_.store(false, std::memory_order_release);
        return;
    }
    estop_button_pressed_.store(msg->buttons[static_cast<size_t>(idx)] != 0, std::memory_order_release);
}

bool TestFr3HuskyController::isJoyConnected() const
{
    return joy_subscriber_ && joy_subscriber_->get_publisher_count() > 0;
}

}  // namespace fr3_husky_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_husky_controller::TestFr3HuskyController, controller_interface::ControllerInterface)
