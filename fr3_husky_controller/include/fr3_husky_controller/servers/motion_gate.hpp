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
        CONT_TRACKER,
        MOVE_TO_JOINT,
    };

    bool tryAcquireRT(Owner requested_owner, const std::string& name, std::string* reason) noexcept
    {
        Owner expected = Owner::NONE;
        if (!owner_.compare_exchange_strong(expected, requested_owner, std::memory_order_acq_rel, std::memory_order_acquire))
        {
            if (expected == requested_owner && ownerNameRT() == name)
            {
                return true;
            }
            if (reason)
            {
                *reason = "motion busy by owner=" + ownerToString(expected) + " name=" + ownerNameRT();
            }
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(name_mutex_);
            owner_name_ = name;
        }
        return true;
    }

    bool tryAcquire(Owner requested_owner, const std::string& name, std::string* reason)
    {
        const bool acquired = tryAcquireRT(requested_owner, name, reason);
        if (acquired)
        {
            RCLCPP_INFO(rclcpp::get_logger("motion_gate"),
                        "acquire owner=%s name=%s",
                        ownerToString(requested_owner).c_str(), name.c_str());
        }
        else if (reason)
        {
            RCLCPP_WARN(rclcpp::get_logger("motion_gate"),
                        "acquire denied requester_owner=%s requester_name=%s current_owner=%s current_name=%s",
                        ownerToString(requested_owner).c_str(),
                        name.c_str(),
                        ownerToString(owner()).c_str(),
                        ownerName().c_str());
        }
        return acquired;
    }

    bool releaseRT(Owner requested_owner) noexcept
    {
        Owner expected = requested_owner;
        if (!owner_.compare_exchange_strong(expected, Owner::NONE, std::memory_order_acq_rel, std::memory_order_acquire))
        {
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(name_mutex_);
            owner_name_.clear();
        }
        return true;
    }

    bool release(Owner requested_owner)
    {
        if (!releaseRT(requested_owner))
        {
            RCLCPP_WARN(rclcpp::get_logger("motion_gate"),
                        "release ignored requester_owner=%s current_owner=%s current_name=%s",
                        ownerToString(requested_owner).c_str(),
                        ownerToString(owner()).c_str(),
                        ownerName().c_str());
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("motion_gate"),
                    "release owner=%s name=%s",
                    ownerToString(requested_owner).c_str(),
                    ownerName().c_str());
        return true;
    }

    Owner owner() const noexcept
    {
        return owner_.load(std::memory_order_acquire);
    }

    std::string ownerNameRT() const noexcept
    {
        std::lock_guard<std::mutex> lock(name_mutex_);
        return owner_name_;
    }

    std::string ownerName() const
    {
        std::lock_guard<std::mutex> lock(name_mutex_);
        return owner_name_;
    }

    static std::string ownerToString(Owner owner)
    {
        switch (owner)
        {
            case Owner::NONE: return "NONE";
            case Owner::CARTESIAN_EXECUTOR: return "CARTESIAN_EXECUTOR";
            case Owner::TRAJECTORY_EXECUTOR: return "TRAJECTORY_EXECUTOR";
            case Owner::CONT_TRACKER: return "CONT_TRACKER";
            case Owner::MOVE_TO_JOINT: return "MOVE_TO_JOINT";
            default: return "UNKNOWN";
        }
    }

private:
    std::atomic<Owner> owner_{Owner::NONE};
    mutable std::mutex name_mutex_;
    std::string owner_name_;
};

inline std::shared_ptr<MotionGate> getGlobalMotionGate()
{
    static std::shared_ptr<MotionGate> gate = std::make_shared<MotionGate>();
    return gate;
}

}  // namespace fr3_husky_controller::servers
