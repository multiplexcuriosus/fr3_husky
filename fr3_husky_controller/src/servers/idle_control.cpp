#include <fr3_husky_controller/servers/idle_control.hpp>

namespace fr3_husky_controller::servers
{

IdleControl::IdleControl(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater)
: name_(name),
  node_(node),
  model_updater_(model_updater)
{
    RCLCPP_INFO(node_->get_logger(), "[%s] IdleControl created", name_.c_str());
}

bool IdleControl::compute(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!was_idle_)
    {
        onActivated();
        was_idle_ = true;
    }

    model_updater_.haltCommands();

    return true;
}

void IdleControl::onActivated()
{
    RCLCPP_INFO(node_->get_logger(), "[%s] entered idle", name_.c_str());
}

void IdleControl::onDeactivated()
{
    if (was_idle_)
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] exit idle", name_.c_str());
        was_idle_ = false;
    }
}

}  // namespace fr3_husky_controller::servers
