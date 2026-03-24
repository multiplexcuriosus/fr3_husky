#pragma once

#include <memory>

#include <fr3_husky_msgs/action/gravity_compensation.hpp>

#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/model/fr3_model_updater.hpp>

namespace fr3_husky_controller::servers::fr3
{

class GravityCompensation final : public ActionServerBase<fr3_husky_msgs::action::GravityCompensation>
{
public:
    using ActionT = fr3_husky_msgs::action::GravityCompensation;
    using Base = ActionServerBase<ActionT>;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason = typename Base::StopReason;
    using ResultPtr = typename Base::ResultPtr;

    GravityCompensation(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater);
    ~GravityCompensation() override = default;

    int priority() const override { return 0; }

private:
    bool acceptGoal(const ActionT::Goal& goal) override;
    void onGoalAccepted(const Goal& goal) override;
    void onStart() override;
    ComputeResult compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void onStop(StopReason reason) override;
    ResultPtr makeResult(StopReason reason) override;

private:
    FR3ModelUpdater& fr3_model_updater_;
    bool use_qp_{false};
};

}  // namespace fr3_husky_controller::servers::fr3
