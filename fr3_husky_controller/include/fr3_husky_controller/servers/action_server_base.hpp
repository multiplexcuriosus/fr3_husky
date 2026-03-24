#pragma once

#include <atomic>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <fr3_husky_controller/model/model_updater_base.hpp>

namespace fr3_husky_controller::servers
{

class ActionServerManager
{
public:
    using NodePtr = rclcpp_lifecycle::LifecycleNode::SharedPtr;
    using Factory = std::function<std::shared_ptr<ActionServerManager>(
        const std::string& name,
        const NodePtr& node,
        ModelUpdaterBase& model_updater)>;

    static bool registerFR3Server(const std::string& name, Factory factory)
    {
        fr3Registry()[name] = std::move(factory);
        return true;
    }
    static bool registerFR3HuskyServer(const std::string& name, Factory factory)
    {
        fr3HuskyRegistry()[name] = std::move(factory);
        return true;
    }

    static std::vector<std::shared_ptr<ActionServerManager>> createAllFR3(
        const NodePtr& node,
        ModelUpdaterBase& model_updater);
    static std::vector<std::shared_ptr<ActionServerManager>> createAllFR3Husky(
        const NodePtr& node,
        ModelUpdaterBase& model_updater);

    ActionServerManager(std::string name, const NodePtr& node, ModelUpdaterBase& model_updater);
    virtual ~ActionServerManager() = default;

    // Controller side interface
    virtual bool update(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
    virtual bool isActive() const = 0;
    virtual int priority() const { return 0; }

    bool consumeActivateRequest();
    bool consumeCancelRequest();

    virtual void onActivated() {};
    virtual void onDeactivated() {};

    const std::string& getName() const { return name_; }

protected:
    void requestActivate();
    void requestCancel();

    std::string name_;
    NodePtr node_;
    ModelUpdaterBase& model_updater_;

private:
    static std::map<std::string, Factory>& fr3Registry()
    {
        static std::map<std::string, Factory> reg_fr3;
        return reg_fr3;
    }
    static std::map<std::string, Factory>& fr3HuskyRegistry()
    {
        static std::map<std::string, Factory> reg_fr3_husky;
        return reg_fr3_husky;
    }

    static std::vector<std::shared_ptr<ActionServerManager>> createAllFromRegistry(
        const NodePtr& node,
        ModelUpdaterBase& model_updater,
        const std::map<std::string, Factory>& registry,
        const char* group_name)
    {
        std::vector<std::shared_ptr<ActionServerManager>> out;
        out.reserve(registry.size());
        for (const auto& [name, fac] : registry)
        {
            out.push_back(fac(name, node, model_updater));
            RCLCPP_INFO(node->get_logger(), "[ActionServerRegistry:%s] Created: %s", group_name, name.c_str());
        }
        return out;
    }

    std::atomic<bool> activate_requested_{false};
    std::atomic<bool> cancel_requested_{false};
};

template <typename ActionT>
class ActionServerBase : public ActionServerManager
{
public:
    using NodePtr = typename ActionServerManager::NodePtr;

    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    using GoalResponse = rclcpp_action::GoalResponse;
    using CancelResponse = rclcpp_action::CancelResponse;
    using FeedbackPtr = std::shared_ptr<Feedback>;
    using ResultPtr = std::shared_ptr<Result>;

    enum class ComputeResult
    {
        RUNNING,
        SUCCEEDED,
        ABORTED,
    };

    enum class StopReason
    {
        NONE,
        CANCELED,
        SUCCEEDED,
        ABORTED,
    };

    ActionServerBase(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater)
    : ActionServerManager(name, node, model_updater)
    {
        action_server_ = rclcpp_action::create_server<ActionT>(
            node_,
            name_,
            std::bind(&ActionServerBase::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServerBase::handleCancel, this, std::placeholders::_1),
            std::bind(&ActionServerBase::handleAccepted, this, std::placeholders::_1));
    }

    ~ActionServerBase() override = default;

    bool update(const rclcpp::Time& time, const rclcpp::Duration& period) override
    {
        std::shared_ptr<GoalHandle> goal_handle;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_ || !active_goal_)
            {
                return true;
            }
            goal_handle = active_goal_;
        }

        if (goal_handle->is_canceling())
        {
            finalizeGoal(StopReason::CANCELED);
            return true;
        }

        ComputeResult state = ComputeResult::RUNNING;
        try
        {
            state = compute(time, period);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] compute exception: %s", name_.c_str(), e.what());
            state = ComputeResult::ABORTED;
        }
        catch (...)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] compute unknown exception", name_.c_str());
            state = ComputeResult::ABORTED;
        }

        if (state == ComputeResult::RUNNING)
        {
            return true;
        }
        if (state == ComputeResult::SUCCEEDED)
        {
            finalizeGoal(StopReason::SUCCEEDED);
            return true;
        }
        finalizeGoal(StopReason::ABORTED);
        return true;
    }

    bool isActive() const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_;
    }

    void onActivated() override
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_ || !active_goal_)
            {
                return;
            }
        }

        try
        {
            // Refresh model state right before onStart so action servers can use current robot data.
            model_updater_.updateJointStates();
            model_updater_.updateRobotData();
            onStart();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] onStart exception: %s", name_.c_str(), e.what());
            finalizeGoal(StopReason::ABORTED);
        }
        catch (...)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] onStart unknown exception", name_.c_str());
            finalizeGoal(StopReason::ABORTED);
        }
    }

    void onDeactivated() override
    {
        std::shared_ptr<GoalHandle> goal_handle;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            goal_handle = active_goal_;
        }

        if (!goal_handle)
        {
            return;
        }

        if (goal_handle->is_canceling())
        {
            finalizeGoal(StopReason::CANCELED);
            return;
        }

        finalizeGoal(StopReason::ABORTED);
    }

protected:
    virtual bool acceptGoal(const Goal& goal) = 0;
    virtual void onGoalAccepted(const Goal& goal) {(void)goal;}
    virtual void onStart() {}
    virtual ComputeResult compute(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
    virtual void onStop(StopReason reason) {(void)reason;}
    virtual ResultPtr makeResult(StopReason reason)
    {
        (void)reason;
        return std::make_shared<Result>();
    }

    void publishFeedback(const FeedbackPtr& feedback) const
    {
        std::shared_ptr<GoalHandle> goal_handle;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            goal_handle = active_goal_;
        }

        if (goal_handle && feedback)
        {
            goal_handle->publish_feedback(feedback);
        }
    }

private:
    GoalResponse handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const Goal> goal)
    {
        if (!goal)
        {
            return GoalResponse::REJECT;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_ || active_goal_)
            {
                RCLCPP_WARN(node_->get_logger(), "[%s] Reject goal while another goal is active", name_.c_str());
                return GoalResponse::REJECT;
            }
        }

        if (!acceptGoal(*goal))
        {
            return GoalResponse::REJECT;
        }
        return GoalResponse::ACCEPT_AND_DEFER;
    }

    CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || !active_goal_ || active_goal_.get() != goal_handle.get())
        {
            return CancelResponse::REJECT;
        }

        requestCancel();
        return CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        if (!goal_handle)
        {
            return;
        }

        Goal goal_copy = *(goal_handle->get_goal());
        try
        {
            onGoalAccepted(goal_copy);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] onGoalAccepted exception: %s", name_.c_str(), e.what());
            auto result = safeMakeResult(StopReason::ABORTED);
            goal_handle->abort(result);
            return;
        }
        catch (...)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] onGoalAccepted unknown exception", name_.c_str());
            auto result = safeMakeResult(StopReason::ABORTED);
            goal_handle->abort(result);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            active_goal_ = goal_handle;
            active_goal_msg_ = std::move(goal_copy);
            active_ = true;
        }

        // Transition from ACCEPTED → EXECUTING so that succeed()/abort()/canceled()
        // are valid state transitions when the goal finishes.
        goal_handle->execute();

        requestActivate();
    }

    void finalizeGoal(StopReason reason)
    {
        std::shared_ptr<GoalHandle> goal_handle;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_goal_)
            {
                active_ = false;
                return;
            }
            goal_handle = active_goal_;
            active_goal_.reset();
            active_ = false;
        }

        try
        {
            onStop(reason);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] onStop exception: %s", name_.c_str(), e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] onStop unknown exception", name_.c_str());
        }

        auto result = safeMakeResult(reason);
        if (reason == StopReason::CANCELED)
        {
            goal_handle->canceled(result);
            return;
        }
        if (reason == StopReason::SUCCEEDED)
        {
            goal_handle->succeed(result);
            return;
        }
        goal_handle->abort(result);
    }

    ResultPtr safeMakeResult(StopReason reason)
    {
        ResultPtr result;
        try
        {
            result = makeResult(reason);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] makeResult exception: %s", name_.c_str(), e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] makeResult unknown exception", name_.c_str());
        }

        if (!result)
        {
            result = std::make_shared<Result>();
        }
        return result;
    }

    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    std::shared_ptr<GoalHandle> active_goal_;
    Goal active_goal_msg_{};
    bool active_{false};
    mutable std::mutex mutex_;
};

#define REGISTER_FR3_ACTION_SERVER(ServerClass, server_name)                               \
    static bool _registered_fr3_##ServerClass =                                           \
        ::fr3_husky_controller::servers::ActionServerManager::registerFR3Server(          \
            server_name,                                                                  \
            [](const std::string& name,                                                   \
               const ::fr3_husky_controller::servers::ActionServerManager::NodePtr& node, \
               ::fr3_husky_controller::ModelUpdaterBase& model_updater)                   \
               -> std::shared_ptr<                                                        \
                   ::fr3_husky_controller::servers::ActionServerManager> {                \
              return std::make_shared<ServerClass>(name, node, model_updater);            \
            });

#define REGISTER_FR3_HUSKY_ACTION_SERVER(ServerClass, server_name)                         \
    static bool _registered_fr3_husky_##ServerClass =                                     \
        ::fr3_husky_controller::servers::ActionServerManager::registerFR3HuskyServer(     \
            server_name,                                                                  \
            [](const std::string& name,                                                   \
               const ::fr3_husky_controller::servers::ActionServerManager::NodePtr& node, \
               ::fr3_husky_controller::ModelUpdaterBase& model_updater)                   \
               -> std::shared_ptr<                                                        \
                   ::fr3_husky_controller::servers::ActionServerManager> {                \
              return std::make_shared<ServerClass>(name, node, model_updater);            \
            });

}  // namespace fr3_husky_controller::servers
