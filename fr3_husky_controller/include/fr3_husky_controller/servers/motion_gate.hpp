#pragma once

#include <array>
#include <atomic>
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

    enum class AcquireResult
    {
        ACQUIRED,
        ALREADY_OWNED,
        BUSY,
    };

    enum class ReleaseResult
    {
        RELEASED,
        ALREADY_FREE,
        NOT_OWNER,
    };

    AcquireResult tryAcquireRT(Owner requested_owner) noexcept
    {
        Owner expected = Owner::NONE;
        if (!owner_.compare_exchange_strong(expected, requested_owner, std::memory_order_acq_rel, std::memory_order_acquire))
        {
            if (expected == requested_owner)
            {
                return AcquireResult::ALREADY_OWNED;
            }
            return AcquireResult::BUSY;
        }
        return AcquireResult::ACQUIRED;
    }

    bool tryAcquire(Owner requested_owner, const std::string& name, std::string* reason)
    {
        setOwnerName(requested_owner, name);

        const AcquireResult result = tryAcquireRT(requested_owner);
        if (result == AcquireResult::ACQUIRED)
        {
            RCLCPP_INFO(rclcpp::get_logger("motion_gate"),
                        "acquire owner=%s name=%s",
                        ownerToString(requested_owner).c_str(), ownerName(requested_owner).c_str());
            return true;
        }

        if (result == AcquireResult::ALREADY_OWNED)
        {
            RCLCPP_INFO(rclcpp::get_logger("motion_gate"),
                        "re-acquire owner=%s name=%s",
                        ownerToString(requested_owner).c_str(), ownerName(requested_owner).c_str());
            return true;
        }

        if (reason)
        {
            const Owner current_owner = owner();
            RCLCPP_WARN(rclcpp::get_logger("motion_gate"),
                        "acquire denied requester_owner=%s requester_name=%s current_owner=%s current_name=%s",
                        ownerToString(requested_owner).c_str(),
                        name.c_str(),
                        ownerToString(current_owner).c_str(),
                        ownerName(current_owner).c_str());
            *reason = "motion busy by owner=" + ownerToString(current_owner) + " name=" + ownerName(current_owner);
        }

        return false;
    }

    ReleaseResult releaseRT(Owner requested_owner) noexcept
    {
        Owner expected = requested_owner;
        if (!owner_.compare_exchange_strong(expected, Owner::NONE, std::memory_order_acq_rel, std::memory_order_acquire))
        {
            return expected == Owner::NONE ? ReleaseResult::ALREADY_FREE : ReleaseResult::NOT_OWNER;
        }
        return ReleaseResult::RELEASED;
    }

    bool release(Owner requested_owner)
    {
        const std::string released_name = ownerName(requested_owner);
        const ReleaseResult result = releaseRT(requested_owner);

        if (result != ReleaseResult::RELEASED)
        {
            const Owner current_owner = owner();
            RCLCPP_WARN(rclcpp::get_logger("motion_gate"),
                        "release ignored requester_owner=%s current_owner=%s current_name=%s",
                        ownerToString(requested_owner).c_str(),
                        ownerToString(current_owner).c_str(),
                        ownerName(current_owner).c_str());
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("motion_gate"),
                    "release owner=%s name=%s",
                    ownerToString(requested_owner).c_str(),
                    released_name.c_str());
        return true;
    }

    Owner owner() const noexcept
    {
        return owner_.load(std::memory_order_acquire);
    }

    std::string ownerName(Owner owner) const
    {
        const std::size_t idx = ownerIndex(owner);
        std::lock_guard<std::mutex> lock(name_mutex_);
        return owner_names_[idx].empty() ? ownerToString(owner) : owner_names_[idx];
    }

    std::string ownerName() const
    {
        return ownerName(owner());
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
    static constexpr std::size_t kOwnerCount = 5;

    std::size_t ownerIndex(Owner owner) const
    {
        switch (owner)
        {
            case Owner::NONE: return 0;
            case Owner::CARTESIAN_EXECUTOR: return 1;
            case Owner::TRAJECTORY_EXECUTOR: return 2;
            case Owner::CONT_TRACKER: return 3;
            case Owner::MOVE_TO_JOINT: return 4;
            default: return 0;
        }
    }

    void setOwnerName(Owner owner, const std::string& name)
    {
        if (name.empty())
        {
            return;
        }
        const std::size_t idx = ownerIndex(owner);
        std::lock_guard<std::mutex> lock(name_mutex_);
        owner_names_[idx] = name;
    }

    std::atomic<Owner> owner_{Owner::NONE};
    mutable std::mutex name_mutex_;
    std::array<std::string, kOwnerCount> owner_names_{};
};

inline std::shared_ptr<MotionGate> getGlobalMotionGate()
{
    static std::shared_ptr<MotionGate> gate = std::make_shared<MotionGate>();
    return gate;
}

}  // namespace fr3_husky_controller::servers
