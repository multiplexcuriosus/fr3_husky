#include "fr3_husky_controller/model/model_updater_base.hpp"

namespace fr3_husky_controller
{

ModelUpdaterBase::~ModelUpdaterBase() = default;

int ModelUpdaterBase::jointNameToIndex(const std::string& iface_name)
{
    for (int joint_idx = 1; joint_idx <= FR3_DOF; ++joint_idx)
    {
        if (iface_name.find("joint" + std::to_string(joint_idx)) != std::string::npos)
        {
            return joint_idx - 1;  // 0-based index
        }
    }
    return -1;
}

void ModelUpdaterBase::setInterfaceFlags(bool has_position_state_interface,
                                         bool has_velocity_state_interface,
                                         bool has_effort_state_interface,
                                         bool has_position_command_interface,
                                         bool has_velocity_command_interface,
                                         bool has_effort_command_interface)
{
    has_position_state_interface_ = has_position_state_interface;
    has_velocity_state_interface_ = has_velocity_state_interface;
    has_effort_state_interface_ = has_effort_state_interface;

    has_position_command_interface_ = has_position_command_interface;
    has_velocity_command_interface_ = has_velocity_command_interface;
    has_effort_command_interface_ = has_effort_command_interface;
}


bool ModelUpdaterBase::initialize(size_t num_robots,
                                  size_t manipulator_dof,
                                  double dt,
                                  const std::vector<std::string>& robot_names,
                                  const std::vector<std::string>& ee_names)
{
    num_robots_ = num_robots;
    manipulator_dof_ = manipulator_dof;
    dt_ = dt;
    robot_names_ = robot_names;
    ee_names_ = ee_names;

    halt_initialized_ = false;
    halt_position_.clear();

    is_configured_ = true;

    return true;
}

}  // namespace fr3_husky_controller
