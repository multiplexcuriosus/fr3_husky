#include <fr3_husky_controller/servers/action_server_base.hpp>

namespace fr3_husky_controller::servers
{

std::vector<std::shared_ptr<ActionServerManager>> ActionServerManager::createAllFR3(
    const NodePtr& node,
    ModelUpdaterBase& model_updater)
{
    return createAllFromRegistry(node, model_updater, fr3Registry(), "fr3");
}

std::vector<std::shared_ptr<ActionServerManager>> ActionServerManager::createAllFR3Husky(
    const NodePtr& node,
    ModelUpdaterBase& model_updater)
{
    return createAllFromRegistry(node, model_updater, fr3HuskyRegistry(), "fr3_husky");
}

ActionServerManager::ActionServerManager(
    std::string name,
    const NodePtr& node,
    ModelUpdaterBase& model_updater)
: name_(std::move(name)),
  node_(node),
  model_updater_(model_updater)
{
}

bool ActionServerManager::consumeActivateRequest()
{
    return activate_requested_.exchange(false, std::memory_order_acq_rel);
}

bool ActionServerManager::consumeCancelRequest()
{
    return cancel_requested_.exchange(false, std::memory_order_acq_rel);
}

void ActionServerManager::requestActivate()
{
    activate_requested_.store(true, std::memory_order_release);
}

void ActionServerManager::requestCancel()
{
    cancel_requested_.store(true, std::memory_order_release);
}

}  // namespace fr3_husky_controller::servers
