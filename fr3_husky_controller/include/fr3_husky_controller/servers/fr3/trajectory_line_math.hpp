#pragma once

#include <eigen3/Eigen/Geometry>

#include <cmath>

namespace fr3_husky_controller::servers::fr3
{

inline bool finite3(const Eigen::Vector3d& v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline bool canonicalizeAxisToBaseX(
    const Eigen::Vector3d& input_axis,
    Eigen::Vector3d* canonical_axis,
    bool* was_flipped,
    const double min_norm = 1e-9,
    const double min_abs_x = 1e-6)
{
    if (canonical_axis == nullptr || was_flipped == nullptr)
    {
        return false;
    }

    *was_flipped = false;
    if (!finite3(input_axis))
    {
        return false;
    }

    const double norm = input_axis.norm();
    if (!std::isfinite(norm) || norm < min_norm)
    {
        return false;
    }

    Eigen::Vector3d axis = input_axis / norm;
    const double dot_x = axis.dot(Eigen::Vector3d::UnitX());
    if (!std::isfinite(dot_x) || std::abs(dot_x) < min_abs_x)
    {
        return false;
    }

    if (dot_x < 0.0)
    {
        axis = -axis;
        *was_flipped = true;
    }

    *canonical_axis = axis;
    return true;
}

inline bool projectTcpToLineS(
    const Eigen::Vector3d& tcp_position,
    const Eigen::Vector3d& line_center,
    const Eigen::Vector3d& line_axis,
    double* s)
{
    if (s == nullptr)
    {
        return false;
    }

    if (!finite3(tcp_position) || !finite3(line_center) || !finite3(line_axis))
    {
        return false;
    }

    const double axis_norm = line_axis.norm();
    if (!std::isfinite(axis_norm) || axis_norm < 1e-9)
    {
        return false;
    }

    const Eigen::Vector3d axis = line_axis / axis_norm;
    const double projected_s = axis.dot(tcp_position - line_center);
    if (!std::isfinite(projected_s))
    {
        return false;
    }

    *s = projected_s;
    return true;
}

}  // namespace fr3_husky_controller::servers::fr3
