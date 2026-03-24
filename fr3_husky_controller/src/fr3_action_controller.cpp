#include "fr3_husky_controller/fr3_action_controller.hpp"

namespace fr3_husky_controller 
{
controller_interface::InterfaceConfiguration FR3ActionController::state_interface_configuration() const 
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Manipulator joint state interfaces
    for (const auto & joint_name : params_.manipulator_joints)
    {
        for (const auto & interface_type : params_.manipulator_state_interfaces)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }

    // Franka semantic components (model/state) — only when HW provides them
    if (use_franka_model_)
    {
        for (size_t i = 0; i < num_robots_; ++i)
        {
            for (const auto & name : franka_robot_model_[i]->get_state_interface_names())
            {
                conf.names.push_back(name);
            }
            conf.names.push_back(params_.robot_name[i] + "_" + arm_id_ + "/robot_time");
        }
    }
    
    return conf;
}

controller_interface::InterfaceConfiguration FR3ActionController::command_interface_configuration() const 
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Manipulator command interfaces
    for (const auto & joint_name : params_.manipulator_joints)
    {
        conf.names.push_back(joint_name + "/" + params_.manipulator_command_interface);
    }

    return conf;
}

CallbackReturn FR3ActionController::on_init() 
{
    try
    {
        // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn FR3ActionController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) 
{
    // update parameters if they have changed
    if (param_listener_->is_old(params_))
    {
        params_ = param_listener_->get_params();
        LOGI(get_node(), "Parameters were updated");
    }

    // number of FR3 robot used
    num_robots_ = params_.robot_name.size();
    if (num_robots_ < 1 || num_robots_ > 2)
    {
        LOGE(get_node(), "FR3 Husky controller expects one or two FR3 robots, but got %zu.", num_robots_);
        return CallbackReturn::FAILURE;
    }

    // check names in allowed name list & uniqueness 
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
            LOGE(get_node(), "Duplicate robot_name '%s' detected. robot_name entries must be unique.", name.c_str());
            return CallbackReturn::FAILURE;
        }
    }

    // get manipulator degrees of freedom
    manipulator_dof_ = params_.manipulator_joints.size();
    if (manipulator_dof_ != FR3_DOF * num_robots_)
    {
        LOGE(get_node(), "FR3 Husky controller expects %zu manipulator DoF, but got %zu.", FR3_DOF * num_robots_, manipulator_dof_);
        return CallbackReturn::FAILURE;
    }

    // command flags (exactly one will be true)
    has_position_command_interface_ = (params_.manipulator_command_interface == allowed_interface_types_[0]);
    has_velocity_command_interface_ = (params_.manipulator_command_interface == allowed_interface_types_[1]);
    has_effort_command_interface_   = (params_.manipulator_command_interface == allowed_interface_types_[2]);

    has_position_state_interface_ = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[0]);
    has_velocity_state_interface_ = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[1]);
    has_effort_state_interface_   = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[2]);

    // Validation of combinations of state and velocity together have to be done
    // here because the parameter validators only deal with each parameter
    // separately.
    if (has_velocity_command_interface_ && (!has_velocity_state_interface_ || !has_position_state_interface_))
    {
        LOGE(get_node(),
            "'velocity' command interface can only be used alone if 'velocity' and "
            "'position' state interfaces are present");
        return CallbackReturn::FAILURE;
    }

    // effort interfaces require position and velocity state
    if (has_effort_command_interface_ && (!has_velocity_state_interface_ || !has_position_state_interface_))
    {
        LOGE(get_node(),
                "'effort' command interface can only be used alone or with 'position' command interface "
                "if 'velocity' and 'position' state interfaces are present");
        return CallbackReturn::FAILURE;
    }

    // get sampling time dt_
    if (get_update_rate() == 0)
    {
        throw std::runtime_error("Controller's update rate is set to 0. This should not happen!");
    }
    dt_ = 1.0 / static_cast<double>(get_update_rate());

    // for finding whether hand/mobile base exist
    auto tmp_node = rclcpp::Node::make_shared("_tmp_urdf_client_" + std::string(get_node()->get_name()));
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(tmp_node, "controller_manager");
    param_client->wait_for_service();
    auto cm_params = param_client->get_parameters({"robot_description"});
    std::string tmp_urdf_xml = cm_params[0].value_to_string();

    // When using MuJoCo, do not use franka sementic segment
    use_franka_model_ = (tmp_urdf_xml.find("mujoco_ros_hardware/MujocoHardwareInterface") == std::string::npos);
    LOGW(get_node(), "Franka model: %s", use_franka_model_ ? "available" : "unavailable (pinocchio fallback)");

    // Initialize Franka semantic components (only if HW exports robot_model)
    franka_robot_model_.clear();
    if (use_franka_model_)
    {
        for (const auto & name : params_.robot_name)
        {
            franka_robot_model_.push_back(std::make_unique<franka_semantic_components::FrankaRobotModel>(name + "_" + arm_id_ + "/robot_model",
                                                                                                         name + "_" + arm_id_ + "/robot_state"));
        }
    }

    pinocchio::Model pin_model;
    pinocchio::urdf::buildModelFromXML(tmp_urdf_xml, pin_model);
    pinocchio::Data pin_data = pinocchio::Data(pin_model);

    bool has_hand = false;
    bool has_mobile = false;

    for (const auto& frame : pin_model.frames)
    {
        if (frame.name.find("hand") != std::string::npos) has_hand = true;
        if (frame.name.find("wheel") != std::string::npos) has_mobile = true;
        if (has_hand && has_mobile) break;
    }

    // initialize dyros_robot_data & controller
    const std::string description_pkg = ament_index_cpp::get_package_share_directory("fr3_husky_description");
    std::string xacro_path = description_pkg + "/robots/";
    std::ostringstream segmentation_args;
    segmentation_args << " with_sc:=true"
                      << " hand:=" << (has_hand ? "true" : "false")
                      << " mobile:=" << (has_mobile ? "true" : "false");
    std::string robot_segmentation_description_param = segmentation_args.str();
    std::string robot_description_param = robot_segmentation_description_param + " fix_finger:=true"
                                                                               + " ros2_control:=false"
                                                                               + " use_fake_hardware:=false"
                                                                               + " fake_sensor_commands:=false";

    if(num_robots_ == 1)
    {
        robot_segmentation_description_param += " side:=" + params_.robot_name[0];
        robot_description_param += " side:=" + params_.robot_name[0];
        xacro_path += "single_fr3";
    }
    else if(num_robots_ == 2)
    {
        xacro_path += "dual_fr3";
    }
    
    const std::string urdf_xml = robot_utils::execAndCaptureStdout("xacro " + xacro_path + ".urdf.xacro" + robot_description_param);
    const std::string srdf_xml = robot_utils::execAndCaptureStdout("xacro " + xacro_path + ".srdf.xacro" + robot_segmentation_description_param);

    std::shared_ptr<drc::Manipulator::RobotData> robot_data;
    try
    {
        robot_data = std::make_shared<drc::Manipulator::RobotData>(dt_, urdf_xml, srdf_xml, description_pkg, true);
        
        LOGI(get_node(), robot_data->getVerbose().c_str());
    }
    catch (const std::exception& e)
    {
        LOGE(get_node(), "Failed to initialize RobotData: %s", e.what());
        return CallbackReturn::ERROR;
    }

    std::shared_ptr<drc::Manipulator::RobotController> robot_controller = std::make_shared<drc::Manipulator::RobotController>(robot_data);

    if(!loadDRCGains(robot_controller)) return CallbackReturn::ERROR;

    ee_names_.clear();
    for (const auto & name : params_.robot_name)
    {
        std::string ee_name = name + "_" + arm_id_ + "_";
        if (has_hand) ee_name = ee_name + "hand_tcp";
        else          ee_name = ee_name + "link8";
        ee_names_.push_back(ee_name);
    }

    if (!model_updater_)
    {
        model_updater_ = std::make_unique<FR3ModelUpdater>();
    }

    model_updater_->setNode(get_node());
    model_updater_->setInterfaceFlags(has_position_state_interface_, has_velocity_state_interface_, has_effort_state_interface_,
                                      has_position_command_interface_, has_velocity_command_interface_, has_effort_command_interface_);
    model_updater_->initialize(num_robots_, manipulator_dof_, dt_, params_.robot_name, ee_names_);
    model_updater_->setDRCRobotData(std::move(robot_data));
    model_updater_->setDRCRobotController(std::move(robot_controller));

    action_servers_ = servers::ActionServerManager::createAllFR3(get_node(), *model_updater_);
    active_server_.reset();
    
    idle_control_ = std::make_unique<servers::IdleControl>("fr3_idle", get_node(), *model_updater_);


    return CallbackReturn::SUCCESS;
}

CallbackReturn FR3ActionController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
    if (!model_updater_)
    {
        LOGE(get_node(), "Model updater is not configured.");
        return CallbackReturn::ERROR;
    }

    // Register manipulator interfaces into JointHandle
    std::vector<std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>> state_by_type(allowed_interface_types_.size()); // [interface_type][joint_idx]
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_by_type; // [joint_idx]

    for (const auto & interface : params_.manipulator_state_interfaces)
    {
        auto it = std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
        if (it == allowed_interface_types_.end()) continue;
        size_t idx = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
        if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.manipulator_joints, interface, state_by_type[idx]))
        {
            LOGE(get_node(), "Expected %zu '%s' state interfaces, got %zu.", manipulator_dof_, interface.c_str(), state_by_type[idx].size());
            return CallbackReturn::ERROR;
        }
        if (state_by_type[idx].size() != manipulator_dof_)
        {
            LOGE(get_node(), "State interface '%s' not available for every manipulator joint (%zu of %zu).", interface.c_str(), state_by_type[idx].size(), manipulator_dof_);
            return CallbackReturn::ERROR;
        }
    }

    const auto & cmd_interface = params_.manipulator_command_interface;
    auto it = std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), cmd_interface);
    if (it == allowed_interface_types_.end())
    {
        LOGE(get_node(), "Invalid command_interface '%s'.", cmd_interface.c_str());
        return CallbackReturn::ERROR;
    }
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.manipulator_joints, cmd_interface, cmd_by_type))
    {
        LOGE(get_node(), "Expected %zu '%s' command interfaces, got %zu.",
            manipulator_dof_, cmd_interface.c_str(), cmd_by_type.size());
        return CallbackReturn::ERROR;
    }
    if (cmd_by_type.size() != manipulator_dof_)
    {
        LOGE(get_node(), "Command interface '%s' not available for every manipulator joint (%zu of %zu).",
            cmd_interface.c_str(), cmd_by_type.size(), manipulator_dof_);
        return CallbackReturn::ERROR;
    }

    RobotHandle robot_handle;
    robot_handle.mani_joints.reserve(manipulator_dof_);
    for (size_t i = 0; i < manipulator_dof_; ++i)
    {
        robot_handle.mani_joints.emplace_back(cmd_by_type[i]);
        auto& handle = robot_handle.mani_joints.back();
        handle.state.clear();

        auto default_state_ref = state_by_type[kPositionIndex][i]; // position state is mandatory
        handle.state.assign(allowed_interface_types_.size(), default_state_ref);

        if (has_position_state_interface_)   handle.state[kPositionIndex] = state_by_type[kPositionIndex][i];
        if (has_velocity_state_interface_)   handle.state[kVelocityIndex] = state_by_type[kVelocityIndex][i];
        if (has_effort_state_interface_ && state_by_type.size() > kEffortIndex && !state_by_type[kEffortIndex].empty())
            handle.state[kEffortIndex] = state_by_type[kEffortIndex][i];
    }
    model_updater_->setRobotHandles(std::move(robot_handle));

    // Assign Franka semantic component state interfaces
    if (use_franka_model_)
    {
        try
        {
            for (const auto & model : franka_robot_model_)
            {
                model->assign_loaned_state_interfaces(state_interfaces_);
            }
            model_updater_->setFrankaModel(&franka_robot_model_);
        }
        catch (const std::exception & e)
        {
            LOGW(get_node(), "Franka semantic component state interfaces not available (%s).", e.what());
        }
    }

    is_halted_ = false;

    play_time_ = get_node()->now().seconds();
    model_updater_->updateJointStates();
    model_updater_->updateRobotData();
    model_updater_->setInitFromCurrent();
    control_start_time_ = play_time_;

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FR3ActionController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
    if (!is_halted_)
    {
        if (model_updater_) model_updater_->haltCommands();
        is_halted_ = true;
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type FR3ActionController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    if (!model_updater_)
    {
        LOGE(get_node(), "Model updater is not available during update loop.");
        return controller_interface::return_type::ERROR;
    }

    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        if (!is_halted_)
        {
            if (model_updater_) model_updater_->haltCommands();
            is_halted_ = true;
        }
        return controller_interface::return_type::OK;
    }

    play_time_ = get_node()->now().seconds();
    model_updater_->updateJointStates();
    model_updater_->updateRobotData();

    if (!active_server_)
    {
        std::shared_ptr<fr3_husky_controller::servers::ActionServerManager> best;
        int best_p = std::numeric_limits<int>::min();

        for (auto& s : action_servers_)
        {
            if (s->consumeActivateRequest())
            {
            const int p = s->priority();
            if (!best || p > best_p) { best = s; best_p = p; }
            }
        }

        if (best)
        {
            active_server_ = best;
            active_server_->onActivated();
        }
    }

    if (active_server_)
    {
        const bool cancel = active_server_->consumeCancelRequest();
        const bool still_active = active_server_->isActive();

        if (cancel || !still_active)
        {
            active_server_->onDeactivated();
            active_server_.reset();
        }
    }

    if (active_server_)
    {
        if (idle_control_) idle_control_->onDeactivated();
        active_server_->update(time, period);
    }
    else
    {
        if (idle_control_) idle_control_->compute(time, period);
    }

    return controller_interface::return_type::OK;
}

bool FR3ActionController::loadDRCGains(std::shared_ptr<drc::Manipulator::RobotController> robot_controller)
{
    constexpr size_t task_dof = 6;

    const auto & joint = params_.dyros_robot_controller.manipulator_joint_gains;
    const auto & task  = params_.dyros_robot_controller.task_gains;
    const auto & qpik  = params_.dyros_robot_controller.QPIK_weight;
    const auto & qpid  = params_.dyros_robot_controller.QPID_weight;

    auto check_vector_size = [this](const std::string & name, size_t expected, size_t got) -> bool
    {
        if (got != expected)
        {
            LOGE(get_node(), "Parameter '%s' expected %zu values, got %zu.", name.c_str(), expected, got);
            return false;
        }
        return true;
    };

    // manipulator joint gains
    if (!check_vector_size("dyros_robot_controller.manipulator_joint_gains.kp", manipulator_dof_, joint.kp.size())) return false;
    if (!check_vector_size("dyros_robot_controller.manipulator_joint_gains.kv", manipulator_dof_, joint.kv.size())) return false;

    // task-space gains
    if (!check_vector_size("dyros_robot_controller.task_gains.ik.kp", task_dof, task.ik.kp.size())) return false;
    if (!check_vector_size("dyros_robot_controller.task_gains.id.kp", task_dof, task.id.kp.size())) return false;
    if (!check_vector_size("dyros_robot_controller.task_gains.id.kv", task_dof, task.id.kv.size())) return false;

    // QPIK weights
    if (!check_vector_size("dyros_robot_controller.QPIK_weight.tracking.weights", task_dof, qpik.tracking.weights.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPIK_weight.joint.velocity.manipulator", manipulator_dof_, qpik.joint.velocity.manipulator.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPIK_weight.joint.acceleration.manipulator", manipulator_dof_, qpik.joint.acceleration.manipulator.size())) return false;

    // QPID weights
    if (!check_vector_size("dyros_robot_controller.QPID_weight.tracking.weights", task_dof, qpid.tracking.weights.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.joint.velocity.manipulator", manipulator_dof_, qpid.joint.velocity.manipulator.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.joint.acceleration.manipulator", manipulator_dof_, qpid.joint.acceleration.manipulator.size())) return false;

    const Eigen::VectorXd mani_joint_kp = Eigen::Map<const Eigen::VectorXd>(joint.kp.data(), joint.kp.size());
    const Eigen::VectorXd mani_joint_kv = Eigen::Map<const Eigen::VectorXd>(joint.kv.data(), joint.kv.size());

    const Eigen::VectorXd task_ik_kp = Eigen::Map<const Eigen::VectorXd>(task.ik.kp.data(), task.ik.kp.size());
    const Eigen::VectorXd task_id_kp = Eigen::Map<const Eigen::VectorXd>(task.id.kp.data(), task.id.kp.size());
    const Eigen::VectorXd task_id_kv = Eigen::Map<const Eigen::VectorXd>(task.id.kv.data(), task.id.kv.size());

    const Eigen::VectorXd qpik_tracking = Eigen::Map<const Eigen::VectorXd>(qpik.tracking.weights.data(), qpik.tracking.weights.size());
    const Eigen::VectorXd qpik_mani_damping = Eigen::Map<const Eigen::VectorXd>(qpik.joint.velocity.manipulator.data(), qpik.joint.velocity.manipulator.size());
    const Eigen::VectorXd qpik_mani_acc_damping = Eigen::Map<const Eigen::VectorXd>(qpik.joint.acceleration.manipulator.data(), qpik.joint.acceleration.manipulator.size());
    
    const Eigen::VectorXd qpid_tracking = Eigen::Map<const Eigen::VectorXd>(qpid.tracking.weights.data(), qpid.tracking.weights.size());
    const Eigen::VectorXd qpid_mani_vel_damping = Eigen::Map<const Eigen::VectorXd>(qpid.joint.velocity.manipulator.data(), qpid.joint.velocity.manipulator.size());
    const Eigen::VectorXd qpid_mani_acc_damping = Eigen::Map<const Eigen::VectorXd>(qpid.joint.acceleration.manipulator.data(), qpid.joint.acceleration.manipulator.size());

    std::ostringstream oss;
    oss << "dyros robot controller gains" << "\n"
        << "\tmanipulator_joint_gains" << "\n"
        << "\t\tKp: " << mani_joint_kp.transpose() << "\n"
        << "\t\tKv: " << mani_joint_kv.transpose() << "\n"
        << "\ttask_gains" << "\n"
        << "\t\tik.Kp: " << task_ik_kp.transpose() << "\n"
        << "\t\tid.Kp: " << task_id_kp.transpose() << "\n"
        << "\t\tid.Kv: " << task_id_kv.transpose() << "\n"
        << "\tQPIK_weight" << "\n"
        << "\t\ttracking.weights: " << qpik_tracking.transpose() << "\n"
        << "\t\tjoint.velocity.manipulator: " << qpik_mani_damping.transpose() << "\n"
        << "\t\tjoint.acceleration.manipulator: " << qpik_mani_acc_damping.transpose() << "\n"
        << "\tQPID_weight" << "\n"
        << "\t\ttracking.weights: " << qpid_tracking.transpose() << "\n"
        << "\t\tjoint.velocity.manipulator: " << qpid_mani_vel_damping.transpose() << "\n"
        << "\t\tjoint.acceleration.manipulator: " << qpid_mani_acc_damping.transpose();
    LOGI(get_node(), "%s", oss.str().c_str());

    robot_controller->setJointGain(mani_joint_kp, mani_joint_kv);
    robot_controller->setIKGain(task_ik_kp);
    robot_controller->setIDGain(task_id_kp, task_id_kv);
    robot_controller->setQPIKGain(qpik_tracking, qpik_mani_damping, qpik_mani_acc_damping);
    robot_controller->setQPIDGain(qpid_tracking, qpid_mani_vel_damping, qpid_mani_acc_damping);
    return true;
}

}  // namespace fr3_husky_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_husky_controller::FR3ActionController, controller_interface::ControllerInterface)

        
