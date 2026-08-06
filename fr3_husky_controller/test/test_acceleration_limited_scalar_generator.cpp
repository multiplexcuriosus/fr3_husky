#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

double clamp(double value, double lower, double upper)
{
    return std::max(lower, std::min(upper, value));
}

double sign(double value)
{
    return (value > 0.0) - (value < 0.0);
}

struct Sample
{
    double s;
    double v;
    double acc;
};

class AccelerationLimitedScalarGenerator
{
public:
    void configure(double v_max, double a_max, double min_s, double max_s, double deadband, double holding_velocity)
    {
        v_max_ = std::max(1e-5, std::abs(v_max));
        a_max_ = std::max(1e-5, std::abs(a_max));
        min_s_ = std::min(min_s, max_s);
        max_s_ = std::max(min_s, max_s);
        deadband_ = std::max(0.0, deadband);
        holding_velocity_ = std::max(1e-5, std::abs(holding_velocity));
    }

    void reset(double s, double v)
    {
        s_ = clamp(s, min_s_, max_s_);
        v_ = clamp(v, -v_max_, v_max_);
    }

    Sample step(double target_s, double dt)
    {
        target_s = clamp(target_s, min_s_, max_s_);

        const double error = target_s - s_;
        double desired_velocity = 0.0;
        if (std::abs(error) > deadband_ || std::abs(v_) > holding_velocity_)
        {
            const double stopping_speed = std::sqrt(
                std::max(0.0, 2.0 * a_max_ * std::max(0.0, std::abs(error) - 0.5 * deadband_)));
            desired_velocity = sign(error) * std::min(v_max_, stopping_speed);
        }

        const double old_v = v_;
        const double max_delta_v = a_max_ * dt;
        v_ += clamp(desired_velocity - v_, -max_delta_v, max_delta_v);
        v_ = clamp(v_, -v_max_, v_max_);
        s_ += 0.5 * (old_v + v_) * dt;

        if (s_ > max_s_)
        {
            s_ = max_s_;
            v_ = std::min(0.0, v_);
        }
        else if (s_ < min_s_)
        {
            s_ = min_s_;
            v_ = std::max(0.0, v_);
        }

        const double acc = (v_ - old_v) / dt;
        return {s_, v_, acc};
    }

private:
    double v_max_{0.4};
    double a_max_{1.0};
    double min_s_{-0.2};
    double max_s_{0.2};
    double deadband_{0.002};
    double holding_velocity_{0.005};
    double s_{0.0};
    double v_{0.0};
};

void expectWithinBounds(
    const std::vector<Sample>& samples,
    double v_max,
    double a_max,
    double min_s,
    double max_s,
    double acc_tol = 1e-6)
{
    for (const auto& sample : samples)
    {
        EXPECT_LE(std::abs(sample.v), v_max + 1e-6);
        EXPECT_LE(std::abs(sample.acc), a_max + acc_tol);
        EXPECT_GE(sample.s, min_s - 1e-6);
        EXPECT_LE(sample.s, max_s + 1e-6);
    }
}

TEST(AccelerationLimitedScalarGeneratorTest, StationaryTarget)
{
    AccelerationLimitedScalarGenerator generator;
    generator.configure(0.4, 1.0, -0.15, 0.15, 0.001, 0.003);
    generator.reset(0.02, 0.0);

    std::vector<Sample> samples;
    samples.reserve(2000);
    for (int i = 0; i < 2000; ++i)
    {
        samples.push_back(generator.step(0.02, 0.001));
    }

    expectWithinBounds(samples, 0.4, 1.0, -0.15, 0.15);
    EXPECT_NEAR(samples.back().s, 0.02, 1e-6);
    EXPECT_NEAR(samples.back().v, 0.0, 1e-6);
}

TEST(AccelerationLimitedScalarGeneratorTest, PositiveAndNegativeSteps)
{
    AccelerationLimitedScalarGenerator generator;
    generator.configure(0.4, 1.0, -0.15, 0.15, 0.002, 0.005);
    generator.reset(0.0, 0.0);

    std::vector<Sample> samples;
    samples.reserve(5000);
    for (int i = 0; i < 2500; ++i)
    {
        samples.push_back(generator.step(0.12, 0.001));
    }
    for (int i = 0; i < 2500; ++i)
    {
        samples.push_back(generator.step(-0.10, 0.001));
    }

    expectWithinBounds(samples, 0.4, 1.0, -0.15, 0.15);
    EXPECT_NEAR(samples.back().s, -0.10, 0.003);
}

TEST(AccelerationLimitedScalarGeneratorTest, RepeatedRetargetingAtThirtyHz)
{
    AccelerationLimitedScalarGenerator generator;
    generator.configure(0.45, 1.2, -0.2, 0.2, 0.002, 0.005);
    generator.reset(0.0, 0.0);

    std::vector<Sample> samples;
    samples.reserve(3000);

    int next_change_step = 0;
    double target = 0.0;
    for (int i = 0; i < 3000; ++i)
    {
        if (i >= next_change_step)
        {
            const double t = static_cast<double>(i) * 0.001;
            target = 0.12 * std::sin(0.8 * t);
            next_change_step += 33;  // approximately 30 Hz at 1 kHz
        }
        samples.push_back(generator.step(target, 0.001));
    }

    expectWithinBounds(samples, 0.45, 1.2, -0.2, 0.2, 1e-3);
}

TEST(AccelerationLimitedScalarGeneratorTest, TargetReversalWhileMoving)
{
    AccelerationLimitedScalarGenerator generator;
    generator.configure(0.4, 1.0, -0.2, 0.2, 0.002, 0.005);
    generator.reset(-0.1, 0.0);

    std::vector<Sample> samples;
    samples.reserve(3000);
    for (int i = 0; i < 600; ++i)
    {
        samples.push_back(generator.step(0.12, 0.001));
    }
    for (int i = 0; i < 2400; ++i)
    {
        samples.push_back(generator.step(-0.12, 0.001));
    }

    expectWithinBounds(samples, 0.4, 1.0, -0.2, 0.2);
    EXPECT_LT(samples.back().s, -0.10);
}

TEST(AccelerationLimitedScalarGeneratorTest, BoundaryTargets)
{
    AccelerationLimitedScalarGenerator generator;
    generator.configure(0.4, 1.0, -0.15, 0.15, 0.002, 0.005);
    generator.reset(0.0, 0.0);

    std::vector<Sample> samples;
    samples.reserve(4500);
    for (int i = 0; i < 2200; ++i)
    {
        samples.push_back(generator.step(0.15, 0.001));
    }
    for (int i = 0; i < 2300; ++i)
    {
        samples.push_back(generator.step(-0.15, 0.001));
    }

    expectWithinBounds(samples, 0.4, 1.0, -0.15, 0.15);
}

TEST(AccelerationLimitedScalarGeneratorTest, VariableDt)
{
    AccelerationLimitedScalarGenerator generator;
    generator.configure(0.4, 1.0, -0.2, 0.2, 0.002, 0.005);
    generator.reset(0.0, 0.0);

    std::vector<Sample> samples;
    samples.reserve(3000);
    for (int i = 0; i < 3000; ++i)
    {
        const double dt = (i % 3 == 0) ? 0.0008 : ((i % 3 == 1) ? 0.0010 : 0.0012);
        samples.push_back(generator.step(0.1, dt));
    }

    expectWithinBounds(samples, 0.4, 1.0, -0.2, 0.2, 1e-3);
}

TEST(AccelerationLimitedScalarGeneratorTest, StaleHoldLastBehavior)
{
    // Hold-last means updates can stop and the controller keeps converging to
    // the latest accepted target without aborting.
    AccelerationLimitedScalarGenerator generator;
    generator.configure(0.4, 1.0, -0.2, 0.2, 0.002, 0.005);
    generator.reset(0.0, 0.0);

    std::vector<Sample> samples;
    samples.reserve(3000);

    // Initial stream (~30 Hz updates).
    int next_change_step = 0;
    double target = 0.10;
    for (int i = 0; i < 1000; ++i)
    {
        if (i >= next_change_step)
        {
            target = 0.10 + 0.01 * std::sin(0.03 * static_cast<double>(i));
            next_change_step += 33;
        }
        samples.push_back(generator.step(target, 0.001));
    }

    // Stale period: no updates, keep last target.
    const double held_target = target;
    for (int i = 0; i < 1200; ++i)
    {
        samples.push_back(generator.step(held_target, 0.001));
    }

    // Updates resume.
    for (int i = 0; i < 800; ++i)
    {
        samples.push_back(generator.step(-0.08, 0.001));
    }

    expectWithinBounds(samples, 0.4, 1.0, -0.2, 0.2);
    EXPECT_NEAR(samples[2200].s, held_target, 0.01);
    EXPECT_NEAR(samples.back().s, -0.08, 0.004);
}

TEST(AccelerationLimitedScalarGeneratorTest, TerminalBrakeAndCancellation)
{
    AccelerationLimitedScalarGenerator generator;
    generator.configure(0.5, 1.5, -0.3, 0.3, 0.002, 0.005);
    generator.reset(0.0, 0.0);

    std::vector<Sample> samples;
    samples.reserve(3500);
    for (int i = 0; i < 500; ++i)
    {
        samples.push_back(generator.step(0.2, 0.001));
    }

    // Terminal brake/cancel behavior: hold target at current reference to decelerate.
    const double brake_target = samples.back().s;
    for (int i = 0; i < 3000; ++i)
    {
        samples.push_back(generator.step(brake_target, 0.001));
    }

    expectWithinBounds(samples, 0.5, 1.5, -0.3, 0.3);
    EXPECT_NEAR(samples.back().v, 0.0, 0.01);
}

}  // namespace
