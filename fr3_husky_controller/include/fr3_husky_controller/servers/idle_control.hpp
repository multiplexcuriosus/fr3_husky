#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <fr3_husky_controller/model/model_updater_base.hpp>

namespace fr3_husky_controller::servers
{

class IdleControl
{
    public:
        using NodePtr = rclcpp_lifecycle::LifecycleNode::SharedPtr;

        IdleControl(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater);
        ~IdleControl() = default;

        bool compute(const rclcpp::Time& time, const rclcpp::Duration& period);

        void onActivated();
        void onDeactivated();

    private:
        std::string name_;
        NodePtr node_;
        ModelUpdaterBase& model_updater_;

        bool was_idle_{false};
};

}  // namespace fr3_husky_controller::servers
