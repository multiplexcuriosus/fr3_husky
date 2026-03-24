# Action Server Code Generation

`fr3_husky_controller` provides two scripts that generate action server boilerplate and automatically patch `CMakeLists.txt`.

| Script | Target |
|---|---|
| `generate_fr3_action_server.py` | FR3 arm action server |
| `generate_fr3_husky_action_server.py` | FR3 + Husky (with mobile base) action server |

---

## Table of Contents
- [What It Generates](#what-it-generates)
- [Usage](#usage)
- [Options](#options)
- [Generated File Structure](#generated-file-structure)
- [Implementing the TODOs](#implementing-the-todos)
- [Registration Macro](#registration-macro)
- [After Generation](#after-generation)

---

## What It Generates

Each script performs three actions:

1. **HPP file** — action server class header
2. **CPP file** — action server class implementation with TODO stubs
3. **CMakeLists.txt patch** — inserts the new `.cpp` into the `add_library` block (deduplicated)

---

## Usage

```bash
# FR3 action server (default action type: control_msgs::action::FollowJointTrajectory)
python3 fr3_husky_controller/generate_fr3_action_server.py GravityCompensation

# Custom action type
python3 fr3_husky_controller/generate_fr3_action_server.py GravityCompensation \
  --action-include "fr3_husky_msgs/action/gravity_compensation.hpp" \
  --action-type "fr3_husky_msgs::action::GravityCompensation"

# Override action name (default: fr3_<snake>)
python3 fr3_husky_controller/generate_fr3_action_server.py GravityCompensation \
  --action-name fr3_gravity_compensation

# Overwrite existing files
python3 fr3_husky_controller/generate_fr3_action_server.py GravityCompensation --force

# FR3+Husky action server
python3 fr3_husky_controller/generate_fr3_husky_action_server.py GravityCompensation \
  --action-include "fr3_husky_msgs/action/gravity_compensation.hpp" \
  --action-type "fr3_husky_msgs::action::GravityCompensation"
```

---

## Options

| Option | Default | Description |
|---|---|---|
| `name` | (required) | Server name. CamelCase, snake_case, or spaces all accepted. |
| `--action-name` | `fr3_<snake>` / `fr3_husky_<snake>` | Runtime registration name (used as the action topic name) |
| `--action-include` | `control_msgs/action/follow_joint_trajectory.hpp` | C++ include path |
| `--action-type` | `control_msgs::action::FollowJointTrajectory` | C++ action type |
| `--suffix` | `_action_server` | File name suffix (`<snake><suffix>.hpp/.cpp`) |
| `--force` | false | Overwrite existing files |

Name conversion rules:
- `GravityCompensation` → snake: `gravity_compensation`, class: `GravityCompensation`
- `"My New Server"` → snake: `my_new_server`, class: `MyNewServer`
- `hold-position` → snake: `hold_position`, class: `HoldPosition`

---

## Generated File Structure

### FR3 (`generate_fr3_action_server.py`)

```
fr3_husky_controller/
├── include/fr3_husky_controller/servers/fr3/
│   └── <snake>_action_server.hpp
└── src/servers/fr3/
    └── <snake>_action_server.cpp
```

- Namespace: `fr3_husky_controller::servers::fr3`
- Base class: `ActionServerBase<ActionT>`
- Model updater: `FR3ModelUpdater`
- Registration macro: `REGISTER_FR3_ACTION_SERVER(ClassName, "action_name")`

### FR3+Husky (`generate_fr3_husky_action_server.py`)

```
fr3_husky_controller/
├── include/fr3_husky_controller/servers/fr3_husky/
│   └── <snake>_action_server.hpp
└── src/servers/fr3_husky/
    └── <snake>_action_server.cpp
```

- Namespace: `fr3_husky_controller::servers::fr3_husky`
- Base class: `ActionServerBase<ActionT>`
- Model updater: `FR3HuskyModelUpdater`
- Registration macro: `REGISTER_FR3_HUSKY_ACTION_SERVER(ClassName, "action_name")`

---

## Implementing the TODOs

### `acceptGoal(const ActionT::Goal& goal)`
Validate the goal and decide whether to accept it. Return `true` to accept.

```cpp
bool GravityCompensation::acceptGoal(const ActionT::Goal& goal)
{
    if (!goal.use_qp) return false;
    return true;
}
```

### `onGoalAccepted(const ActionT::Goal& goal)`
Called once after the goal is accepted, before `compute()` starts. Use this to cache goal values.

```cpp
void GravityCompensation::onGoalAccepted(const ActionT::Goal& goal)
{
    use_qp_ = goal.use_qp;
}
```

### `compute(const rclcpp::Time& time, const rclcpp::Duration& period)`
Called every control cycle. Read robot state via `fr3_model_updater_` (or `fr3_husky_model_updater_`) and write commands.

Return values:
- `ComputeResult::RUNNING` — keep running
- `ComputeResult::SUCCEEDED` — finish successfully
- `ComputeResult::ABORTED` — finish with failure

```cpp
GravityCompensation::ComputeResult GravityCompensation::compute(
    const rclcpp::Time& time,
    const rclcpp::Duration& period)
{
    const auto& state = fr3_model_updater_.robotState();

    auto fb = std::make_shared<ActionT::Feedback>();
    publishFeedback(fb);

    fr3_model_updater_.setTorqueCommand(torques);
    return ComputeResult::RUNNING;
}
```

### `onStop(StopReason reason)`
Called on termination (succeeded / canceled / aborted). `model_updater_.haltCommands()` is already included in the generated stub.

### `makeResult(StopReason reason)`
Build and return the final result message. Fill any required fields before returning.

---

## Registration Macro

The last line of the generated CPP registers the server at static initialization time:

```cpp
REGISTER_FR3_ACTION_SERVER(GravityCompensation, "fr3_gravity_compensation")
// or
REGISTER_FR3_HUSKY_ACTION_SERVER(GravityCompensation, "fr3_husky_gravity_compensation")
```

When the controller calls `ActionServerManager::createAllFR3(...)` / `createAllFR3Husky(...)`, all registered servers are instantiated automatically.

---

## After Generation

```bash
# 1) Implement the TODOs in the generated files:
#      include/fr3_husky_controller/servers/fr3/<snake>_action_server.hpp
#      src/servers/fr3/<snake>_action_server.cpp

# 2) Build
colcon build --packages-select fr3_husky_controller

# 3) Verify the action is available after launching the controller
ros2 action list
```
