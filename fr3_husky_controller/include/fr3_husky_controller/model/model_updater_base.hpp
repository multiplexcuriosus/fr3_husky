#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <array>
#include <vector>

#include <Eigen/Eigen>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>

#ifndef FR3_DOF
#define FR3_DOF 7
#endif

#ifndef WHEEL_PER_SIDE
#define WHEEL_PER_SIDE 2 // front, rear
#endif

namespace fr3_husky_controller
{
struct JointHandle
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command;
    
    explicit JointHandle(std::reference_wrapper<hardware_interface::LoanedCommandInterface> cmd)
    : command(cmd)
    {
    }
};

struct WheelHandle
{
    std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> state; // position, velocity
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command; // only velocity
};

struct RobotHandle
{
  std::vector<JointHandle> mani_joints;

  std::vector<WheelHandle> left_wheels;
  std::vector<WheelHandle> right_wheels;

};

class ModelUpdaterBase
{
    public:
        virtual ~ModelUpdaterBase();

        void setNode(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) { node_ = node; }
        void setInterfaceFlags(bool has_position_state_interface,
                               bool has_velocity_state_interface,
                               bool has_effort_state_interface,
                               bool has_position_command_interface,
                               bool has_velocity_command_interface,
                               bool has_effort_command_interface);
        virtual bool initialize(size_t num_robots,
                                size_t manipulator_dof,
                                double dt, 
                                const std::vector<std::string>& robot_names,
                                const std::vector<std::string>& ee_names);
        void setFrankaModel(std::vector<std::unique_ptr<franka_semantic_components::FrankaRobotModel>>* franka_robot_model) { franka_robot_model_ = franka_robot_model; }
        void setRobotHandles(RobotHandle&& robot_handle) { robot_handle_ = std::move(robot_handle); }
        virtual void updateJointStates() = 0;
        virtual void updateRobotData() = 0;
        virtual void haltCommands() = 0;
        virtual bool getHandlesReady() const = 0;

        const bool HasPositionStateInterface()   { return has_position_state_interface_; }
        const bool HasVelocityStateInterface()   { return has_velocity_state_interface_; }
        const bool HasEffortStateInterface()     { return has_effort_state_interface_; }
        const bool HasPositionCommandInterface() { return has_position_command_interface_; }
        const bool HasVelocityCommandInterface() { return has_velocity_command_interface_; }
        const bool HasEffortCommandInterface()   { return has_effort_command_interface_; }

        const double getDT() { return dt_; }

        std::vector<std::unique_ptr<franka_semantic_components::FrankaRobotModel>>*getFrankaRobotModel() { return franka_robot_model_; }

    public:
        std::vector<std::string> robot_names_;
        std::vector<std::string> ee_names_;
        size_t num_robots_{0}; // number of FR3 arms
        size_t manipulator_dof_{0};
        const size_t mobile_dof_{2};
        const size_t virtual_dof_{3};
        double dt_{0.0};
        const std::string arm_id_{"fr3"};


    protected:
        int jointNameToIndex(const std::string& iface_name);
        
    protected:
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;



        std::mutex robot_data_mutex_;
        
        bool is_configured_{false};
        bool halt_initialized_{false};
        std::map<std::string, double> halt_position_; // [joint_name][initial pos], for manipulator

        std::vector<std::unique_ptr<franka_semantic_components::FrankaRobotModel>>* franka_robot_model_{nullptr};
        RobotHandle robot_handle_;

        static constexpr size_t kPositionIndex         = 0;
        static constexpr size_t kVelocityIndex         = 1;
        static constexpr size_t kEffortIndex           = 2;
        static constexpr size_t kFeedbackPositionIndex = 0;
        static constexpr size_t kFeedbackVelocityIndex = 1;
        static constexpr size_t kFeedbackEffortIndex   = 2;

        bool has_position_state_interface_{false};
        bool has_velocity_state_interface_{false};
        bool has_effort_state_interface_{false};
        bool has_position_command_interface_{false};
        bool has_velocity_command_interface_{false};
        bool has_effort_command_interface_{false};

};

}  // namespace fr3_husky_controller
