#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/servers/motion_gate.hpp>

namespace
{
using fr3_husky_controller::servers::ActionServerBase;
using fr3_husky_controller::servers::MotionGate;

struct StubModelUpdater : fr3_husky_controller::ModelUpdaterBase
{
    void updateJointStates() override {}
    void updateRobotData() override {}
    void haltCommands() override {}
    bool getHandlesReady() const override { return true; }
};

struct DummyAction
{
    using Goal = std::pair<int, int>;
    using Feedback = std::pair<int, int>;
    using Result = std::pair<int, int>;
};

class DummyServer : public ActionServerBase<DummyAction>
{
public:
    using Base = ActionServerBase<DummyAction>;
    using Goal = DummyAction::Goal;
    using Result = DummyAction::Result;

    DummyServer(const std::string& name, const NodePtr& node, fr3_husky_controller::ModelUpdaterBase& model_updater)
    : Base(name, node, model_updater)
    {
    }

    bool acceptGoal(const Goal&) override { return true; }
    ComputeResult compute(const rclcpp::Time&, const rclcpp::Duration&) override { return ComputeResult::RUNNING; }
    ResultPtr makeResult(StopReason) override { return std::make_shared<Result>(Result{42, 42}); }

    using Base::clearActivateRequest;
    using Base::consumeActivateRequest;
    using Base::hasActivateRequest;
    using Base::requestActivate;
    using Base::requestCancel;
    using Base::handleCancelRequest;
    using Base::hasCancelRequest;
    using Base::consumeCancelRequest;
};

TEST(ActionServerLifecycleTest, ActivateRequestIsRetainedUntilSelectedAndThenCleared)
{
    auto node = rclcpp::Node::make_shared("test_action_server_lifecycle");
    StubModelUpdater model_updater;
    DummyServer server("test_server", node, model_updater);

    EXPECT_FALSE(server.hasActivateRequest());
    server.requestActivate();
    EXPECT_TRUE(server.hasActivateRequest());
    EXPECT_TRUE(server.consumeActivateRequest());
    EXPECT_FALSE(server.hasActivateRequest());

    server.requestActivate();
    server.clearActivateRequest();
    EXPECT_FALSE(server.hasActivateRequest());
}

TEST(ActionServerLifecycleTest, CancelRequestIsConsumedOnce)
{
    auto node = rclcpp::Node::make_shared("test_action_server_lifecycle_cancel");
    StubModelUpdater model_updater;
    DummyServer server("test_server_cancel", node, model_updater);

    EXPECT_FALSE(server.hasCancelRequest());
    server.requestCancel();
    EXPECT_TRUE(server.hasCancelRequest());
    EXPECT_TRUE(server.handleCancelRequest());
    EXPECT_FALSE(server.hasCancelRequest());
    EXPECT_FALSE(server.handleCancelRequest());
}

TEST(MotionGateTest, RepeatedReleaseIsIdempotentAndOwnerChecked)
{
    MotionGate gate;
    EXPECT_TRUE(gate.tryAcquire(MotionGate::Owner::CONT_TRACKER, "test_owner", nullptr));
    EXPECT_TRUE(gate.releaseRT(MotionGate::Owner::CONT_TRACKER));
    EXPECT_FALSE(gate.releaseRT(MotionGate::Owner::CONT_TRACKER));
    EXPECT_EQ(gate.owner(), MotionGate::Owner::NONE);
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
