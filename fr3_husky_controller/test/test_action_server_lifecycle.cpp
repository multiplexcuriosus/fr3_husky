#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/servers/motion_gate.hpp>
#include <fr3_husky_msgs/action/line_trajectory.hpp>

namespace
{
using namespace std::chrono_literals;
using fr3_husky_controller::servers::ActionServerBase;
using fr3_husky_controller::servers::ActionServerManager;
using fr3_husky_controller::servers::MotionGate;
using LineAction = fr3_husky_msgs::action::LineTrajectory;

struct FakeModelUpdater : fr3_husky_controller::ModelUpdaterBase
{
    void updateJointStates() override {}
    void updateRobotData() override {}
    void haltCommands() override { ++halt_count; }
    bool getHandlesReady() const override { return true; }

    std::atomic<int> halt_count{0};
};

class TestActionServer : public ActionServerBase<LineAction>
{
public:
    using Base = ActionServerBase<LineAction>;
    using Goal = LineAction::Goal;

    TestActionServer(
        const std::string& name,
        const NodePtr& node,
        fr3_husky_controller::ModelUpdaterBase& model_updater,
        int server_priority,
        const std::shared_ptr<MotionGate>& gate,
        MotionGate::Owner gate_owner,
        bool halt_on_stop)
    : Base(name, node, model_updater),
      priority_(server_priority),
      gate_(gate),
      gate_owner_(gate_owner),
      halt_on_stop_(halt_on_stop)
    {
    }

    int priority() const override { return priority_; }

    bool acceptGoal(const Goal&) override { return true; }

    void onGoalAccepted(const Goal&) override
    {
        if (throw_on_goal_accept_.load(std::memory_order_acquire))
        {
            throw std::runtime_error("test onGoalAccepted failure");
        }
    }

    void onStart() override
    {
        ++start_count_;
        control_session_started_ = false;

        if (use_gate_)
        {
            const auto result = gate_->tryAcquireRT(gate_owner_);
            if (result == MotionGate::AcquireResult::BUSY)
            {
                throw std::runtime_error("gate busy");
            }
            gate_acquired_ = true;
        }

        if (fail_on_start_.load(std::memory_order_acquire))
        {
            throw std::runtime_error("test onStart failure");
        }

        control_session_started_ = true;
    }

    ComputeResult compute(const rclcpp::Time&, const rclcpp::Duration&) override
    {
        if (finish_requested_.exchange(false, std::memory_order_acq_rel))
        {
            return ComputeResult::SUCCEEDED;
        }
        return ComputeResult::RUNNING;
    }

    void onStop(StopReason) override
    {
        ++stop_count_;
        if (halt_on_stop_ && control_session_started_)
        {
            model_updater_.haltCommands();
        }

        if (gate_acquired_)
        {
            gate_->releaseRT(gate_owner_);
            gate_acquired_ = false;
        }

        control_session_started_ = false;
    }

    void requestFinish() { finish_requested_.store(true, std::memory_order_release); }
    void setThrowOnGoalAccepted(bool v) { throw_on_goal_accept_.store(v, std::memory_order_release); }
    void setFailOnStart(bool v) { fail_on_start_.store(v, std::memory_order_release); }

    int startCount() const { return start_count_.load(std::memory_order_acquire); }
    int stopCount() const { return stop_count_.load(std::memory_order_acquire); }

private:
    int priority_{0};
    std::shared_ptr<MotionGate> gate_;
    MotionGate::Owner gate_owner_{MotionGate::Owner::NONE};
    bool halt_on_stop_{false};
    bool use_gate_{true};
    bool gate_acquired_{false};
    bool control_session_started_{false};

    std::atomic<bool> finish_requested_{false};
    std::atomic<bool> throw_on_goal_accept_{false};
    std::atomic<bool> fail_on_start_{false};
    std::atomic<int> start_count_{0};
    std::atomic<int> stop_count_{0};
};

class TestScheduler
{
public:
    void addServer(const std::shared_ptr<ActionServerManager>& server)
    {
        servers_.push_back(server);
    }

    void tick(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        for (auto& s : servers_)
        {
            if (s->hasCancelRequest())
            {
                s->handleCancelRequest();
                if (active_server_ == s && !s->isActive())
                {
                    active_server_.reset();
                }
            }
        }

        if (active_server_)
        {
            if (!active_server_->isActive())
            {
                active_server_->onDeactivated();
                active_server_.reset();
            }
        }

        std::shared_ptr<ActionServerManager> best;
        int best_priority = std::numeric_limits<int>::min();
        for (auto& s : servers_)
        {
            if (!s->hasActivateRequest())
            {
                continue;
            }
            if (!best || s->priority() > best_priority)
            {
                best = s;
                best_priority = s->priority();
            }
        }

        if (best)
        {
            const bool should_activate = !active_server_ || best_priority > active_server_->priority();
            if (should_activate)
            {
                best->consumeActivateRequest();
                if (active_server_)
                {
                    active_server_->onDeactivated();
                    active_server_.reset();
                }
                active_server_ = best;
                active_server_->onActivated();
            }
        }

        if (active_server_)
        {
            active_server_->update(time, period);
        }
    }

private:
    std::vector<std::shared_ptr<ActionServerManager>> servers_;
    std::shared_ptr<ActionServerManager> active_server_;
};

struct GoalSendState
{
    std::promise<rclcpp_action::ClientGoalHandle<LineAction>::SharedPtr> accepted_promise;
    std::promise<rclcpp_action::ResultCode> result_promise;
};

template <typename Predicate>
bool spinUntil(
    rclcpp::executors::SingleThreadedExecutor& exec,
    Predicate&& predicate,
    std::chrono::milliseconds timeout = 1500ms)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        exec.spin_some();
        if (predicate())
        {
            return true;
        }
        std::this_thread::sleep_for(2ms);
    }
    return predicate();
}

rclcpp_action::ClientGoalHandle<LineAction>::SharedPtr sendGoalAndWaitAccepted(
    rclcpp::executors::SingleThreadedExecutor& exec,
    const rclcpp_action::Client<LineAction>::SharedPtr& client,
    GoalSendState* state)
{
    LineAction::Goal goal;
    auto options = rclcpp_action::Client<LineAction>::SendGoalOptions();
    options.goal_response_callback = [state](const rclcpp_action::ClientGoalHandle<LineAction>::SharedPtr& handle)
    {
        state->accepted_promise.set_value(handle);
    };
    options.result_callback = [state](const rclcpp_action::ClientGoalHandle<LineAction>::WrappedResult& wrapped)
    {
        state->result_promise.set_value(wrapped.code);
    };

    client->async_send_goal(goal, options);

    auto accepted_future = state->accepted_promise.get_future();
    EXPECT_TRUE(spinUntil(exec, [&accepted_future]() {
        return accepted_future.wait_for(0ms) == std::future_status::ready;
    }));

    return accepted_future.get();
}

TEST(MotionGateTest, RtAcquireReleaseSemantics)
{
    MotionGate gate;

    EXPECT_EQ(gate.tryAcquireRT(MotionGate::Owner::TRAJECTORY_EXECUTOR), MotionGate::AcquireResult::ACQUIRED);
    EXPECT_EQ(gate.tryAcquireRT(MotionGate::Owner::TRAJECTORY_EXECUTOR), MotionGate::AcquireResult::ALREADY_OWNED);
    EXPECT_EQ(gate.tryAcquireRT(MotionGate::Owner::CONT_TRACKER), MotionGate::AcquireResult::BUSY);

    EXPECT_EQ(gate.releaseRT(MotionGate::Owner::CONT_TRACKER), MotionGate::ReleaseResult::NOT_OWNER);
    EXPECT_EQ(gate.releaseRT(MotionGate::Owner::TRAJECTORY_EXECUTOR), MotionGate::ReleaseResult::RELEASED);
    EXPECT_EQ(gate.releaseRT(MotionGate::Owner::TRAJECTORY_EXECUTOR), MotionGate::ReleaseResult::ALREADY_FREE);

    EXPECT_EQ(gate.tryAcquireRT(MotionGate::Owner::CONT_TRACKER), MotionGate::AcquireResult::ACQUIRED);
    EXPECT_EQ(gate.releaseRT(MotionGate::Owner::CONT_TRACKER), MotionGate::ReleaseResult::RELEASED);
    EXPECT_EQ(gate.owner(), MotionGate::Owner::NONE);
}

TEST(ActionLifecycleTest, PendingActivationRetainedUntilHigherPriorityFinishes)
{
    auto lifecycle_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lifecycle_handoff");
    auto client_node = rclcpp::Node::make_shared("test_lifecycle_handoff_client");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(lifecycle_node->get_node_base_interface());
    exec.add_node(client_node);

    auto gate = std::make_shared<MotionGate>();
    FakeModelUpdater high_updater;
    FakeModelUpdater low_updater;

    auto high = std::make_shared<TestActionServer>(
        "trajectory_executor", lifecycle_node, high_updater, 8, gate,
        MotionGate::Owner::TRAJECTORY_EXECUTOR, false);
    auto low = std::make_shared<TestActionServer>(
        "cont_tracker", lifecycle_node, low_updater, 0, gate,
        MotionGate::Owner::CONT_TRACKER, true);

    TestScheduler scheduler;
    scheduler.addServer(high);
    scheduler.addServer(low);

    auto high_client = rclcpp_action::create_client<LineAction>(client_node, "trajectory_executor");
    auto low_client = rclcpp_action::create_client<LineAction>(client_node, "cont_tracker");
    ASSERT_TRUE(spinUntil(exec, [&]() { return high_client->action_server_is_ready() && low_client->action_server_is_ready(); }));

    GoalSendState high_state;
    GoalSendState low_state;

    auto high_goal_handle = sendGoalAndWaitAccepted(exec, high_client, &high_state);
    ASSERT_NE(high_goal_handle, nullptr);

    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));
    EXPECT_EQ(high->startCount(), 1);
    EXPECT_EQ(gate->owner(), MotionGate::Owner::TRAJECTORY_EXECUTOR);

    auto low_goal_handle = sendGoalAndWaitAccepted(exec, low_client, &low_state);
    ASSERT_NE(low_goal_handle, nullptr);

    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));
    EXPECT_EQ(low->startCount(), 0);
    EXPECT_TRUE(low->hasActivateRequest());

    high->requestFinish();
    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));
    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));

    EXPECT_EQ(low->startCount(), 1);
}

TEST(ActionLifecycleTest, PendingCancelDoesNotHaltOrStealGate)
{
    auto lifecycle_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_pending_cancel");
    auto client_node = rclcpp::Node::make_shared("test_pending_cancel_client");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(lifecycle_node->get_node_base_interface());
    exec.add_node(client_node);

    auto gate = std::make_shared<MotionGate>();
    FakeModelUpdater high_updater;
    FakeModelUpdater low_updater;

    auto high = std::make_shared<TestActionServer>(
        "trajectory_executor_cancel", lifecycle_node, high_updater, 8, gate,
        MotionGate::Owner::TRAJECTORY_EXECUTOR, false);
    auto low = std::make_shared<TestActionServer>(
        "cont_tracker_cancel", lifecycle_node, low_updater, 0, gate,
        MotionGate::Owner::CONT_TRACKER, true);

    TestScheduler scheduler;
    scheduler.addServer(high);
    scheduler.addServer(low);

    auto high_client = rclcpp_action::create_client<LineAction>(client_node, "trajectory_executor_cancel");
    auto low_client = rclcpp_action::create_client<LineAction>(client_node, "cont_tracker_cancel");
    ASSERT_TRUE(spinUntil(exec, [&]() { return high_client->action_server_is_ready() && low_client->action_server_is_ready(); }));

    GoalSendState high_state;
    GoalSendState low_state;
    auto high_goal_handle = sendGoalAndWaitAccepted(exec, high_client, &high_state);
    ASSERT_NE(high_goal_handle, nullptr);

    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));
    EXPECT_EQ(high->startCount(), 1);
    EXPECT_EQ(gate->owner(), MotionGate::Owner::TRAJECTORY_EXECUTOR);

    auto low_goal_handle = sendGoalAndWaitAccepted(exec, low_client, &low_state);
    ASSERT_NE(low_goal_handle, nullptr);

    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));
    EXPECT_EQ(low->startCount(), 0);

    auto cancel_future = low_client->async_cancel_goal(low_goal_handle);
    ASSERT_TRUE(spinUntil(exec, [&]() { return cancel_future.wait_for(0ms) == std::future_status::ready; }));
    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));

    auto low_result_future = low_state.result_promise.get_future();
    ASSERT_TRUE(spinUntil(exec, [&]() { return low_result_future.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(low_result_future.get(), rclcpp_action::ResultCode::CANCELED);

    EXPECT_EQ(low_updater.halt_count.load(std::memory_order_acquire), 0);
    EXPECT_EQ(low->stopCount(), 0);
    EXPECT_FALSE(low->hasActivateRequest());
    EXPECT_TRUE(high->isActive());
    EXPECT_EQ(high->stopCount(), 0);
    EXPECT_EQ(gate->owner(), MotionGate::Owner::TRAJECTORY_EXECUTOR);
}

TEST(ActionLifecycleTest, ActiveCancelHaltsAndReleasesGate)
{
    auto lifecycle_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_active_cancel");
    auto client_node = rclcpp::Node::make_shared("test_active_cancel_client");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(lifecycle_node->get_node_base_interface());
    exec.add_node(client_node);

    auto gate = std::make_shared<MotionGate>();
    FakeModelUpdater updater;
    auto server = std::make_shared<TestActionServer>(
        "cont_tracker_active_cancel", lifecycle_node, updater, 0, gate,
        MotionGate::Owner::CONT_TRACKER, true);

    TestScheduler scheduler;
    scheduler.addServer(server);

    auto client = rclcpp_action::create_client<LineAction>(client_node, "cont_tracker_active_cancel");
    ASSERT_TRUE(spinUntil(exec, [&]() { return client->action_server_is_ready(); }));

    GoalSendState state;
    auto goal_handle = sendGoalAndWaitAccepted(exec, client, &state);
    ASSERT_NE(goal_handle, nullptr);

    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));
    EXPECT_EQ(server->startCount(), 1);
    EXPECT_EQ(gate->owner(), MotionGate::Owner::CONT_TRACKER);

    auto cancel_future = client->async_cancel_goal(goal_handle);
    ASSERT_TRUE(spinUntil(exec, [&]() { return cancel_future.wait_for(0ms) == std::future_status::ready; }));

    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));

    auto result_future = state.result_promise.get_future();
    ASSERT_TRUE(spinUntil(exec, [&]() { return result_future.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(result_future.get(), rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(updater.halt_count.load(std::memory_order_acquire), 1);
    EXPECT_EQ(gate->owner(), MotionGate::Owner::NONE);
}

TEST(ActionLifecycleTest, OnGoalAcceptedExceptionCleansState)
{
    auto lifecycle_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_accept_exception");
    auto client_node = rclcpp::Node::make_shared("test_accept_exception_client");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(lifecycle_node->get_node_base_interface());
    exec.add_node(client_node);

    auto gate = std::make_shared<MotionGate>();
    FakeModelUpdater updater;
    auto server = std::make_shared<TestActionServer>(
        "cont_tracker_accept_exception", lifecycle_node, updater, 0, gate,
        MotionGate::Owner::CONT_TRACKER, true);

    server->setThrowOnGoalAccepted(true);

    TestScheduler scheduler;
    scheduler.addServer(server);

    auto client = rclcpp_action::create_client<LineAction>(client_node, "cont_tracker_accept_exception");
    ASSERT_TRUE(spinUntil(exec, [&]() { return client->action_server_is_ready(); }));

    GoalSendState first_state;
    auto first_goal_handle = sendGoalAndWaitAccepted(exec, client, &first_state);
    ASSERT_NE(first_goal_handle, nullptr);

    auto first_result_future = first_state.result_promise.get_future();
    ASSERT_TRUE(spinUntil(exec, [&]() { return first_result_future.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(first_result_future.get(), rclcpp_action::ResultCode::ABORTED);
    EXPECT_FALSE(server->hasActivateRequest());
    EXPECT_FALSE(server->isActive());

    server->setThrowOnGoalAccepted(false);

    GoalSendState second_state;
    auto second_goal_handle = sendGoalAndWaitAccepted(exec, client, &second_state);
    ASSERT_NE(second_goal_handle, nullptr);

    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));
    EXPECT_EQ(server->startCount(), 1);
}

TEST(ActionLifecycleTest, StartupFailureReleasesGate)
{
    auto lifecycle_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_startup_failure");
    auto client_node = rclcpp::Node::make_shared("test_startup_failure_client");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(lifecycle_node->get_node_base_interface());
    exec.add_node(client_node);

    auto gate = std::make_shared<MotionGate>();
    FakeModelUpdater updater;
    auto server = std::make_shared<TestActionServer>(
        "cont_tracker_startup_failure", lifecycle_node, updater, 0, gate,
        MotionGate::Owner::CONT_TRACKER, true);
    server->setFailOnStart(true);

    TestScheduler scheduler;
    scheduler.addServer(server);

    auto client = rclcpp_action::create_client<LineAction>(client_node, "cont_tracker_startup_failure");
    ASSERT_TRUE(spinUntil(exec, [&]() { return client->action_server_is_ready(); }));

    GoalSendState state;
    auto goal_handle = sendGoalAndWaitAccepted(exec, client, &state);
    ASSERT_NE(goal_handle, nullptr);

    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));

    auto result_future = state.result_promise.get_future();
    ASSERT_TRUE(spinUntil(exec, [&]() { return result_future.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(result_future.get(), rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(updater.halt_count.load(std::memory_order_acquire), 0);
    EXPECT_EQ(gate->owner(), MotionGate::Owner::NONE);
    EXPECT_FALSE(server->hasActivateRequest());
    EXPECT_FALSE(server->isActive());
}

TEST(ActionLifecycleTest, GateBusyStartupFailureDoesNotHaltOrStealGate)
{
    auto lifecycle_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_gate_busy_startup_failure");
    auto client_node = rclcpp::Node::make_shared("test_gate_busy_startup_failure_client");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(lifecycle_node->get_node_base_interface());
    exec.add_node(client_node);

    auto gate = std::make_shared<MotionGate>();
    EXPECT_EQ(gate->tryAcquireRT(MotionGate::Owner::TRAJECTORY_EXECUTOR), MotionGate::AcquireResult::ACQUIRED);

    FakeModelUpdater updater;
    auto server = std::make_shared<TestActionServer>(
        "cont_tracker_gate_busy_start", lifecycle_node, updater, 0, gate,
        MotionGate::Owner::CONT_TRACKER, true);

    TestScheduler scheduler;
    scheduler.addServer(server);

    auto client = rclcpp_action::create_client<LineAction>(client_node, "cont_tracker_gate_busy_start");
    ASSERT_TRUE(spinUntil(exec, [&]() { return client->action_server_is_ready(); }));

    GoalSendState state;
    auto goal_handle = sendGoalAndWaitAccepted(exec, client, &state);
    ASSERT_NE(goal_handle, nullptr);

    scheduler.tick(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.01));

    auto result_future = state.result_promise.get_future();
    ASSERT_TRUE(spinUntil(exec, [&]() { return result_future.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(result_future.get(), rclcpp_action::ResultCode::ABORTED);

    EXPECT_EQ(updater.halt_count.load(std::memory_order_acquire), 0);
    EXPECT_EQ(gate->owner(), MotionGate::Owner::TRAJECTORY_EXECUTOR);
    EXPECT_FALSE(server->hasActivateRequest());
    EXPECT_FALSE(server->isActive());
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
