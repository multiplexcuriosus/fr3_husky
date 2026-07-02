#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace fr3_husky_controller::servers
{

class MotionGate
{
public:
    enum class Owner
    {
        NONE = 0,
        CARTESIAN_EXECUTOR,
        TRAJECTORY_EXECUTOR,
        MOVE_TO_JOINT,
    };

    bool tryAcquire(Owner owner, const std::string& name, std::string* reason)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (owner_ == Owner::NONE)
        {
            owner_ = owner;
            owner_name_ = name;
            RCLCPP_INFO(rclcpp::get_logger("motion_gate"),
                        "acquire owner=%s name=%s",
                        ownerToString(owner_).c_str(), owner_name_.c_str());
            return true;
        }

        if (owner_ == owner && owner_name_ == name)
        {
            RCLCPP_INFO(rclcpp::get_logger("motion_gate"),
                        "re-acquire owner=%s name=%s",
                        ownerToString(owner_).c_str(), owner_name_.c_str());
            return true;
        }

        if (reason)
        {
            *reason = "motion busy by owner=" + ownerToString(owner_) + " name=" + owner_name_;
        }
        RCLCPP_WARN(rclcpp::get_logger("motion_gate"),
                    "acquire denied requester_owner=%s requester_name=%s current_owner=%s current_name=%s",
                    ownerToString(owner).c_str(), name.c_str(), ownerToString(owner_).c_str(), owner_name_.c_str());
        return false;
    }

    void release(Owner owner)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (owner_ != owner)
        {
            RCLCPP_WARN(rclcpp::get_logger("motion_gate"),
                        "release ignored requester_owner=%s current_owner=%s current_name=%s",
                        ownerToString(owner).c_str(), ownerToString(owner_).c_str(), owner_name_.c_str());
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("motion_gate"),
                    "release owner=%s name=%s",
                    ownerToString(owner_).c_str(), owner_name_.c_str());
        owner_ = Owner::NONE;
        owner_name_.clear();
    }

    Owner owner() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return owner_;
    }

    std::string ownerName() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return owner_name_;
    }

    static std::string ownerToString(Owner owner)
    {
        switch (owner)
        {
            case Owner::NONE: return "NONE";
            case Owner::CARTESIAN_EXECUTOR: return "CARTESIAN_EXECUTOR";
            case Owner::TRAJECTORY_EXECUTOR: return "TRAJECTORY_EXECUTOR";
            case Owner::MOVE_TO_JOINT: return "MOVE_TO_JOINT";
            default: return "UNKNOWN";
        }
    }

private:
    mutable std::mutex mutex_;
    Owner owner_{Owner::NONE};
    std::string owner_name_;
};

inline std::shared_ptr<MotionGate> getGlobalMotionGate()
{
    static std::shared_ptr<MotionGate> gate = std::make_shared<MotionGate>();
    return gate;
}

}  // namespace fr3_husky_controller::servers
