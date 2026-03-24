// Copyright (c) 2024 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// #include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_utils 
{

    using namespace std::chrono_literals;
    const auto time_out = 1000ms;  // This is probably not really necessary

    struct JointControlInterfaces 
    {
        std::vector<std::string> command_interfaces;  ///< e.g., {"position", "velocity", "effort"}
        std::vector<std::string> state_interfaces;    ///< e.g., {"position", "velocity", "effort"}
    };

    /**
     * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \p t2
     * indices. If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated
     * mapping vector is <tt>"{2, 1}"</tt>. return empty vector if \p t1 is not a subset of \p t2.
     */
    template <class T>
    inline std::vector<size_t> mapping(const T & t1, const T & t2)
    {
    // t1 must be a subset of t2
    if (t1.size() > t2.size())
    {
        return std::vector<size_t>();
    }

    std::vector<size_t> mapping_vector(t1.size());  // Return value
    for (auto t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
    {
        auto t2_it = std::find(t2.begin(), t2.end(), *t1_it);
        if (t2.end() == t2_it)
        {
        return std::vector<size_t>();
        }
        else
        {
        const size_t t1_dist = static_cast<size_t>(std::distance(t1.begin(), t1_it));
        const size_t t2_dist = static_cast<size_t>(std::distance(t2.begin(), t2_it));
        mapping_vector[t1_dist] = t2_dist;
        }
    }
    return mapping_vector;
    }

    // helper used across multiple controllers; inline to avoid multiple definition at link time
    inline bool contains_interface_type(const std::vector<std::string> & interface_type_list,
                                        const std::string & interface_type)
    {
        return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) != interface_type_list.end();
    }

    // bool poseIsFinite(const geometry_msgs::msg::Pose& pose)
    // {
    //     return std::isfinite(pose.position.x) &&
    //            std::isfinite(pose.position.y) &&
    //            std::isfinite(pose.position.z) &&
    //            std::isfinite(pose.orientation.x) &&
    //            std::isfinite(pose.orientation.y) &&
    //            std::isfinite(pose.orientation.z) &&
    //            std::isfinite(pose.orientation.w);
    // }

    // Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::Pose& pose)
    // {
    //     Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    //     if (q.norm() < 1e-9)
    //     {
    //         q.setIdentity();
    //     }
    //     else
    //     {
    //         q.normalize();
    //     }
    //     Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    //     transform.linear() = q.toRotationMatrix();
    //     transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    //     return transform;
    // }

    // geometry_msgs::msg::Pose eigenToPoseMsg(const Eigen::Affine3d& pose)
    // {
    //     geometry_msgs::msg::Pose msg;
    //     msg.position.x = pose.translation().x();
    //     msg.position.y = pose.translation().y();
    //     msg.position.z = pose.translation().z();
    //     Eigen::Quaterniond q(pose.linear());
    //     q.normalize();
    //     msg.orientation.x = q.x();
    //     msg.orientation.y = q.y();
    //     msg.orientation.z = q.z();
    //     msg.orientation.w = q.w();
    //     return msg;
    // }

    static std::string execAndCaptureStdout(const std::string& cmd)
    {
        // Run command and open a pipe to read its stdout.
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe)
        {
            throw std::runtime_error("popen() failed. Is xacro in PATH?");
        }

        // Read stdout chunk-by-chunk into a string buffer.
        std::string output;
        std::array<char, 4096> buffer{};
        while (true)
        {
            const size_t n = std::fread(buffer.data(), 1, buffer.size(), pipe);
            if (n > 0)
            {
                output.append(buffer.data(), n);
            }
            if (n < buffer.size())
            {
                // Either EOF or error.
                break;
            }
        }

        // Close the pipe and check the command exit status.
        const int rc = pclose(pipe);
        if (rc != 0)
        {
            throw std::runtime_error("Command failed (non-zero exit): " + cmd);
        }

        return output;
    }
}  // namespace robot_utils
