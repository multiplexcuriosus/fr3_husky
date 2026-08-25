#pragma once

#include <atomic>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
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

    bool hasActivateRequest() const;
    bool consumeActivateRequest();
    void clearActivateRequest();
    bool hasCancelRequest() const;
    bool consumeCancelRequest();
    virtual bool handleCancelRequest() { return consumeCancelRequest(); }

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

    bool handleCancelRequest() override
    {
        if (pending_cancel_finalize_requested_.exchange(false, std::memory_order_acq_rel))
        {
            consumeCancelRequest();
            finalizeGoal(StopReason::CANCELED);
            return true;
        }
        return false;
    }

    bool update(const rclcpp::Time& time, const rclcpp::Duration& period) override
    {
        if (consumeCancelRequest())
        {
            finalizeGoal(StopReason::CANCELED);
            return true;
        }

        // Handle same-server preemption: a new goal arrived while this server was active.
        if (preempt_pending_.load(std::memory_order_acquire))
        {
            finalizeGoal(StopReason::ABORTED);  // abort current goal; CANCELED requires CANCELING state (client-initiated only)

            {
                std::lock_guard<std::mutex> lock(mutex_);
                active_goal_     = preempt_goal_handle_;
                active_goal_msg_ = std::move(preempt_goal_msg_);
                active_          = true;
                preempt_goal_handle_.reset();
                preempt_pending_.store(false, std::memory_order_release);
            }

            onActivated();  // calls updateJointStates + updateRobotData + onStart
            return true;
        }

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

    bool activatePendingGoalIfReady()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_ || !pending_goal_handle_)
        {
            return false;
        }

        active_goal_     = pending_goal_handle_;
        active_goal_msg_ = std::move(pending_goal_msg_);
        pending_goal_handle_.reset();
        pending_goal_msg_ = Goal{};
        active_          = true;
        RCLCPP_INFO(node_->get_logger(), "[%s] activation started for pending goal", name_.c_str());
        return true;
    }

    void onActivated() override
    {
        if (!activatePendingGoalIfReady())
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_ || !active_goal_)
            {
                return;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "[%s] activation selected", name_.c_str());

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
    // Override to return true to allow a new incoming goal to preempt (cancel) the current one.
    virtual bool allowPreemption() const { return false; }
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
            if (active_ || active_goal_ || pending_goal_handle_ || preempt_pending_.load(std::memory_order_relaxed))
            {
                if (!allowPreemption())
                {
                    RCLCPP_WARN(node_->get_logger(), "[%s] Reject goal while another goal is active or pending", name_.c_str());
                    return GoalResponse::REJECT;
                }
                if (preempt_pending_.load(std::memory_order_relaxed))
                {
                    // Don't stack preemptions
                    RCLCPP_WARN(node_->get_logger(), "[%s] Reject goal: preemption already pending", name_.c_str());
                    return GoalResponse::REJECT;
                }
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
        bool pending_cancel_requested = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_goal_ && active_goal_.get() == goal_handle.get())
            {
                requestCancel();
                RCLCPP_WARN(node_->get_logger(), "[%s] cancel requested for active goal", name_.c_str());
                return CancelResponse::ACCEPT;
            }
            if (pending_goal_handle_ && pending_goal_handle_.get() == goal_handle.get())
            {
                clearActivateRequest();
                pending_cancel_finalize_requested_.store(true, std::memory_order_release);
                requestCancel();
                pending_cancel_requested = true;
                RCLCPP_WARN(node_->get_logger(), "[%s] goal canceled while pending", name_.c_str());
            }
        }
        return pending_cancel_requested ? CancelResponse::ACCEPT : CancelResponse::REJECT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        if (!goal_handle)
        {
            return;
        }

        Goal goal_copy = *(goal_handle->get_goal());

        // Move ACCEPTED -> EXECUTING before calling user hooks so any early
        // abort/cancel/success path uses valid rcl_action transitions.
        goal_handle->execute();

        RCLCPP_INFO(node_->get_logger(), "[%s] goal accepted", name_.c_str());
        try
        {
            onGoalAccepted(goal_copy);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] onGoalAccepted exception: %s", name_.c_str(), e.what());
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (active_goal_ && active_goal_.get() == goal_handle.get())
                {
                    active_goal_.reset();
                    active_ = false;
                }
                else if (pending_goal_handle_ && pending_goal_handle_.get() == goal_handle.get())
                {
                    pending_goal_handle_.reset();
                    pending_goal_msg_ = Goal{};
                }
                else if (preempt_goal_handle_ && preempt_goal_handle_.get() == goal_handle.get())
                {
                    preempt_goal_handle_.reset();
                    preempt_goal_msg_ = Goal{};
                    preempt_pending_.store(false, std::memory_order_release);
                }
                clearActivateRequest();
            }
            auto result = safeMakeResult(StopReason::ABORTED);
            goal_handle->abort(result);
            return;
        }
        catch (...)
        {
            RCLCPP_ERROR(node_->get_logger(), "[%s] onGoalAccepted unknown exception", name_.c_str());
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (active_goal_ && active_goal_.get() == goal_handle.get())
                {
                    active_goal_.reset();
                    active_ = false;
                }
                else if (pending_goal_handle_ && pending_goal_handle_.get() == goal_handle.get())
                {
                    pending_goal_handle_.reset();
                    pending_goal_msg_ = Goal{};
                }
                else if (preempt_goal_handle_ && preempt_goal_handle_.get() == goal_handle.get())
                {
                    preempt_goal_handle_.reset();
                    preempt_goal_msg_ = Goal{};
                    preempt_pending_.store(false, std::memory_order_release);
                }
                clearActivateRequest();
            }
            auto result = safeMakeResult(StopReason::ABORTED);
            goal_handle->abort(result);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_ || active_goal_)
            {
                // Preemption: store new goal; update() will cancel current and activate this one.
                preempt_goal_handle_ = goal_handle;
                preempt_goal_msg_    = std::move(goal_copy);
                preempt_pending_.store(true, std::memory_order_release);
                RCLCPP_INFO(node_->get_logger(), "[%s] activation pending while higher-priority server is active", name_.c_str());
                return;
            }
            pending_goal_handle_ = goal_handle;
            pending_goal_msg_    = std::move(goal_copy);
        }

        RCLCPP_INFO(node_->get_logger(), "[%s] activation pending", name_.c_str());
        requestActivate();
    }

protected:
    void finalizeGoal(StopReason reason)
    {
        std::shared_ptr<GoalHandle> goal_handle;
        bool finalized_active_goal = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_goal_)
            {
                goal_handle = active_goal_;
                active_goal_.reset();
                active_ = false;
                finalized_active_goal = true;
            }
            else if (pending_goal_handle_)
            {
                goal_handle = pending_goal_handle_;
                pending_goal_handle_.reset();
                pending_goal_msg_ = Goal{};
                active_ = false;
            }
            else
            {
                active_ = false;
                return;
            }
            clearActivateRequest();
        }

        RCLCPP_INFO(node_->get_logger(), "[%s] goal finalized reason=%s", name_.c_str(), reason == StopReason::CANCELED ? "canceled" : reason == StopReason::SUCCEEDED ? "succeeded" : "aborted");

        if (finalized_active_goal)
        {
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

    // Accepted goal waiting for activation due to controller priority / selection.
    std::shared_ptr<GoalHandle> pending_goal_handle_;
    Goal pending_goal_msg_{};

    // Same-server preemption: pending new goal that will replace the current one in update()
    std::shared_ptr<GoalHandle> preempt_goal_handle_;
    Goal preempt_goal_msg_{};
    std::atomic<bool> preempt_pending_{false};
    std::atomic<bool> pending_cancel_finalize_requested_{false};
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
