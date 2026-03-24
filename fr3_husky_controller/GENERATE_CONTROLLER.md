# Controller Code Generation

`fr3_husky_controller` provides two code-generation scripts, one for FR3-only controllers and one for combined FR3+Husky (wholebody) controllers. Both live in the `fr3_husky_controller/` package directory.

---

## Scripts

| Script | Target robot | Config file patched |
|---|---|---|
| `generate_fr3_controller.py` | FR3 arm only | `config/fr3_ros_controllers.yaml` |
| `generate_fr3_husky_controller.py` | FR3 arm + Husky base | `config/fr3_husky_ros_controllers.yaml` |

---

## What each script generates

Both scripts produce the same file set:

| Output | Location |
|---|---|
| Header file | `include/fr3_husky_controller/<snake_name>.hpp` |
| Source file | `src/<snake_name>.cpp` |

### What they patch

| File | Change |
|---|---|
| `CMakeLists.txt` | Adds `src/<snake_name>.cpp` to the `add_library` block |
| `fr3_husky_plugin.xml` | Adds a `<class>` entry for the new plugin type |
| `config/fr3_ros_controllers.yaml` *(FR3 only)* | Adds `left_<name>`, `right_<name>`, `dual_<name>` controller entries |
| `config/fr3_husky_ros_controllers.yaml` *(FR3+Husky)* | Adds `left_<name>`, `right_<name>`, `dual_<name>` controller entries |

---

## Generated class anatomy

### FR3-only (`generate_fr3_controller.py`)

The generated class inherits from `controller_interface::ControllerInterface` and handles:
- FR3 arm state interfaces: `position`, `velocity`, `effort`, `robot_state`, `robot_model`, `robot_time`
- FR3 arm command interfaces: one of `position` / `velocity` / `effort` (selected at generation time)
- No mobile base interfaces

### FR3+Husky (`generate_fr3_husky_controller.py`)

Inherits from the same base and additionally handles:
- Husky wheel state/command interfaces (`left_wheel_names`, `right_wheel_names`)
- `mobile_dof = 2` (left-side average, right-side average)
- Pinocchio kinematics: computes `T_base_link0` (footprint → arm base) at `on_configure()`
- `wheel_pos_`, `wheel_vel_`, `wheel_vel_desired_` Eigen vectors populated each cycle

---

## Usage

```bash
cd fr3_husky_controller

# FR3-only controller
python3 generate_fr3_controller.py <ControllerName> --control_mode <mode> [--force]

# FR3+Husky wholebody controller
python3 generate_fr3_husky_controller.py <ControllerName> --control_mode <mode> [--force]
```

### Arguments

| Argument | Required | Description |
|---|---|---|
| `name` | yes | Controller name in any style (e.g. `MyNewController`, `my_new_controller`). Converted to `snake_case` for files and `CamelCase` for the class. |
| `--control_mode` | yes | Arm command interface: `position`, `velocity`, or `effort` |
| `--force` | no | Overwrite existing `.hpp` / `.cpp` files if they already exist |

### Examples

```bash
# Effort-mode FR3 controller
python3 generate_fr3_controller.py MyNewController --control_mode effort

# Position-mode FR3+Husky wholebody controller
python3 generate_fr3_husky_controller.py MyWholebody --control_mode position --force
```

---

## After generation

1. Implement your control algorithm in the `update()` TODO block inside `src/<snake_name>.cpp`.
2. Rebuild the package:
   ```bash
   colcon build --packages-select fr3_husky_controller
   ```
3. Launch with the new controller name (FR3-only example):
   ```bash
   ros2 launch fr3_husky_controller fr3_controller.launch.py \
     robot_side:=left controller_name:=<snake_name>
   ```
   For FR3+Husky:
   ```bash
   ros2 launch fr3_husky_controller fr3_husky_controller.launch.py \
     robot_side:=left controller_name:=<snake_name>
   ```

Both scripts print a summary on completion:

```
Generated:
  - include/fr3_husky_controller/<snake_name>.hpp
  - src/<snake_name>.cpp
CMakeLists.txt:
  - Inserted into add_library: src/<snake_name>.cpp
fr3_husky_plugin.xml:
  - Added class entry
config/<yaml_file>:
  - Added YAML blocks

Controller class : fr3_husky_controller::<ClassName>
Plugin type      : fr3_husky_controller/<ClassName>
YAML controllers : left_<snake_name>, right_<snake_name>, dual_<snake_name>
Control mode     : <mode>
```
