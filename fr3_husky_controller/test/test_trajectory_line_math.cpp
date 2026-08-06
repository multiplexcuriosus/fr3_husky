#include <gtest/gtest.h>

#include <fr3_husky_controller/servers/fr3/trajectory_line_math.hpp>

#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <limits>

namespace
{

using fr3_husky_controller::servers::fr3::canonicalizeAxisToBaseX;
using fr3_husky_controller::servers::fr3::projectTcpToLineS;

TEST(TrajectoryLineMathTest, TcpAtCenterGivesZeroS)
{
    const Eigen::Vector3d center(0.5, 0.1, 0.2);
    const Eigen::Vector3d axis(1.0, 0.0, 0.0);
    const Eigen::Vector3d tcp = center;

    double s = std::numeric_limits<double>::quiet_NaN();
    ASSERT_TRUE(projectTcpToLineS(tcp, center, axis, &s));
    EXPECT_DOUBLE_EQ(s, 0.0);
}

TEST(TrajectoryLineMathTest, TcpAlongPositiveXGivesPositiveS)
{
    const Eigen::Vector3d center(0.5, 0.0, 0.2);
    const Eigen::Vector3d axis(1.0, 0.0, 0.0);
    const Eigen::Vector3d tcp = center + Eigen::Vector3d(0.08, 0.0, 0.0);

    double s = 0.0;
    ASSERT_TRUE(projectTcpToLineS(tcp, center, axis, &s));
    EXPECT_GT(s, 0.0);
    EXPECT_NEAR(s, 0.08, 1e-12);
}

TEST(TrajectoryLineMathTest, TcpAlongNegativeXGivesNegativeS)
{
    const Eigen::Vector3d center(0.5, 0.0, 0.2);
    const Eigen::Vector3d axis(1.0, 0.0, 0.0);
    const Eigen::Vector3d tcp = center + Eigen::Vector3d(-0.06, 0.0, 0.0);

    double s = 0.0;
    ASSERT_TRUE(projectTcpToLineS(tcp, center, axis, &s));
    EXPECT_LT(s, 0.0);
    EXPECT_NEAR(s, -0.06, 1e-12);
}

TEST(TrajectoryLineMathTest, AxisTowardNegativeXIsFlipped)
{
    const Eigen::Vector3d axis_in(-1.0, 0.2, 0.0);
    Eigen::Vector3d axis_out = Eigen::Vector3d::Zero();
    bool flipped = false;

    ASSERT_TRUE(canonicalizeAxisToBaseX(axis_in, &axis_out, &flipped));
    EXPECT_TRUE(flipped);
    EXPECT_GT(axis_out.dot(Eigen::Vector3d::UnitX()), 0.0);
    EXPECT_NEAR(axis_out.norm(), 1.0, 1e-12);
}

TEST(TrajectoryLineMathTest, AxisWithNearZeroBaseXIsRejected)
{
    const Eigen::Vector3d axis_in(0.0, 1.0, 0.0);
    Eigen::Vector3d axis_out = Eigen::Vector3d::Zero();
    bool flipped = false;

    EXPECT_FALSE(canonicalizeAxisToBaseX(axis_in, &axis_out, &flipped));
}

TEST(TrajectoryLineMathTest, ProjectionOutsideEndpointsIsNotClamped)
{
    const Eigen::Vector3d center(0.4, 0.0, 0.3);
    const Eigen::Vector3d axis(1.0, 0.0, 0.0);
    const double half_length = 0.15;
    const Eigen::Vector3d tcp = center + Eigen::Vector3d(0.33, 0.0, 0.0);

    double s = 0.0;
    ASSERT_TRUE(projectTcpToLineS(tcp, center, axis, &s));
    EXPECT_GT(std::abs(s), half_length);
    EXPECT_NEAR(s, 0.33, 1e-12);
}

TEST(TrajectoryLineMathTest, NonFiniteGeometryOrTcpIsRejected)
{
    const Eigen::Vector3d center(0.4, 0.0, 0.3);
    const Eigen::Vector3d axis(1.0, 0.0, 0.0);
    double s = 0.0;

    Eigen::Vector3d bad_tcp = center;
    bad_tcp.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(projectTcpToLineS(bad_tcp, center, axis, &s));

    Eigen::Vector3d bad_center = center;
    bad_center.y() = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(projectTcpToLineS(center, bad_center, axis, &s));

    EXPECT_FALSE(projectTcpToLineS(center, center, Eigen::Vector3d::Zero(), &s));
}

TEST(TrajectoryLineMathTest, ProjectionMathIndependentOfActionState)
{
    // The projection helper depends only on measured pose and line geometry.
    const Eigen::Vector3d center(0.5, -0.2, 0.2);
    const Eigen::Vector3d axis(1.0, 0.0, 0.0);
    const Eigen::Vector3d tcp(0.62, -0.2, 0.2);

    double s_first = 0.0;
    double s_second = 0.0;
    ASSERT_TRUE(projectTcpToLineS(tcp, center, axis, &s_first));
    ASSERT_TRUE(projectTcpToLineS(tcp, center, axis, &s_second));
    EXPECT_DOUBLE_EQ(s_first, s_second);
}

}  // namespace
