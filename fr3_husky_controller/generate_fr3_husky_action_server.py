#!/usr/bin/env python3
"""
generate_fr3_husky_action_server.py

It generates FR3-Husky action-server boilerplate:
- HPP: include/fr3_husky_controller/servers/fr3_husky/<snake>_action_server.hpp
- CPP: src/servers/fr3_husky/<snake>_action_server.cpp

It also patches CMakeLists.txt to add the new .cpp source (deduped).

Usage:
  python3 generate_fr3_husky_action_server.py GravityCompensation
  python3 generate_fr3_husky_action_server.py Docking --action-name fr3_husky_docking
  python3 generate_fr3_husky_action_server.py "My New Server" --force
"""

import argparse
import re
from pathlib import Path
from typing import Optional, Tuple


def script_pkg_root() -> Path:
    # This file lives in <pkg_root>/generate_fr3_husky_action_server.py
    return Path(__file__).resolve().parent


def to_snake(name: str) -> str:
    s = name.strip()
    s = re.sub(r"[\-\s]+", "_", s)
    s = re.sub(r"[^0-9a-zA-Z_]", "_", s)
    s = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", s)
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s)
    s = s.lower()
    s = re.sub(r"_+", "_", s).strip("_")
    if not s:
        raise ValueError("Name becomes empty after normalization.")
    if s[0].isdigit():
        s = f"server_{s}"
    return s


def to_camel(snake: str) -> str:
    parts = [p for p in snake.split("_") if p]
    out = "".join(p[:1].upper() + p[1:] for p in parts)
    if not out:
        raise ValueError("Cannot create CamelCase class name.")
    if out[0].isdigit():
        out = f"Server{out}"
    return out


def default_action_name(prefix: str, snake: str) -> str:
    return snake if snake.startswith(prefix + "_") else f"{prefix}_{snake}"


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def render_hpp(class_name: str, action_type_include: str, action_type: str) -> str:
    return f"""#pragma once

#include <memory>

#include <{action_type_include}>

#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/model/fr3_husky_model_updater.hpp>

namespace fr3_husky_controller::servers::fr3_husky
{{

class {class_name} final : public ActionServerBase<{action_type}>
{{
public:
    using ActionT = {action_type};
    using Base = ActionServerBase<ActionT>;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason = typename Base::StopReason;
    using ResultPtr = typename Base::ResultPtr;

    {class_name}(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater);
    ~{class_name}() override = default;

    int priority() const override {{ return 0; }}

private:
    bool acceptGoal(const ActionT::Goal& goal) override;
    void onGoalAccepted(const ActionT::Goal& goal) override;
    void onStart() override;
    ComputeResult compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void onStop(StopReason reason) override;
    ResultPtr makeResult(StopReason reason) override;

private:
    FR3HuskyModelUpdater& fr3_husky_model_updater_;
}};

}}  // namespace fr3_husky_controller::servers::fr3_husky
"""


def render_cpp(class_name: str, hpp_include: str, action_name: str) -> str:
    return f"""#include <{hpp_include}>

#include <stdexcept>

namespace fr3_husky_controller::servers::fr3_husky
{{

namespace
{{
FR3HuskyModelUpdater& getFR3HuskyModelUpdater(ModelUpdaterBase& model_updater, const std::string& server_name)
{{
    auto* fr3_husky_model_updater = dynamic_cast<FR3HuskyModelUpdater*>(&model_updater);
    if (!fr3_husky_model_updater)
    {{
        throw std::runtime_error("[" + server_name + "] requires FR3HuskyModelUpdater");
    }}
    return *fr3_husky_model_updater;
}}

}}  // namespace

{class_name}::{class_name}(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater)
: Base(name, node, model_updater),
  fr3_husky_model_updater_(getFR3HuskyModelUpdater(model_updater, name))
{{
    RCLCPP_INFO(node_->get_logger(), "[%s] {class_name} created", name_.c_str());
}}

bool {class_name}::acceptGoal(const ActionT::Goal& goal)
{{
    // TODO: Validate goal and decide accept/reject.
    (void)goal;
    return true;
}}

void {class_name}::onGoalAccepted(const ActionT::Goal& goal)
{{
    // TODO: Cache goal values used during compute().
    (void)goal;
}}

void {class_name}::onStart()
{{
    RCLCPP_INFO(node_->get_logger(), "[%s] started", name_.c_str());
}}

{class_name}::ComputeResult {class_name}::compute(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
{{
    // TODO: Implement control logic using fr3_husky_model_updater_.
    // Example:
    // auto fb = std::make_shared<ActionT::Feedback>();
    // publishFeedback(fb);
    return ComputeResult::RUNNING;
}}

void {class_name}::onStop(StopReason reason)
{{
    model_updater_.haltCommands();

    const char* reason_str = "none";
    if (reason == StopReason::CANCELED)
    {{
        reason_str = "canceled";
    }}
    else if (reason == StopReason::SUCCEEDED)
    {{
        reason_str = "succeeded";
    }}
    else if (reason == StopReason::ABORTED)
    {{
        reason_str = "aborted";
    }}

    RCLCPP_INFO(node_->get_logger(), "[%s] stopped (%s)", name_.c_str(), reason_str);
}}

{class_name}::ResultPtr {class_name}::makeResult(StopReason reason)
{{
    auto result = std::make_shared<ActionT::Result>();
    (void)reason;
    return result;
}}

// Register this server into global registry (executed when this TU is linked)
REGISTER_FR3_HUSKY_ACTION_SERVER({class_name}, "{action_name}")

}}  // namespace fr3_husky_controller::servers::fr3_husky
"""


def patch_cmakelists(cmake_path: Path, new_cpp_rel: str, preferred_marker: str) -> Tuple[bool, str]:
    """
    Insert new_cpp_rel into an add_library/add_executable block.

    Preference:
    - a block that already contains preferred_marker
    - else a block that contains "src/servers/"
    - else first add_library/add_executable block

    Deduped: no-op if already present.
    """
    text = cmake_path.read_text(encoding="utf-8")
    if new_cpp_rel in text:
        return False, f"Already present: {new_cpp_rel}"

    lines = text.splitlines(True)

    def find_blocks():
        blocks = []
        i = 0
        while i < len(lines):
            if re.match(r"^\s*add_(library|executable)\s*\(", lines[i]):
                start = i
                depth = lines[i].count("(") - lines[i].count(")")
                i += 1
                while i < len(lines) and depth > 0:
                    depth += lines[i].count("(") - lines[i].count(")")
                    i += 1
                blocks.append((start, i))
            else:
                i += 1
        return blocks

    blocks = find_blocks()

    def choose_block(marker: Optional[str]):
        if marker is None:
            return None
        for s, e in blocks:
            if marker in "".join(lines[s:e]):
                return (s, e)
        return None

    chosen = choose_block(preferred_marker)
    if chosen is None:
        chosen = choose_block("src/servers/")
    if chosen is None and blocks:
        chosen = blocks[0]

    if chosen is None:
        hint = (
            "\n# ---- Added by generate_fr3_husky_action_server.py ----\n"
            "# Please add this source to your target:\n"
            f"#   {new_cpp_rel}\n"
            "# -----------------------------------------------------\n"
        )
        cmake_path.write_text(text + hint, encoding="utf-8")
        return True, "No add_library/add_executable block found; appended hint."

    s, e = chosen

    insert_at = None
    for j in range(e - 1, s - 1, -1):
        if ")" in lines[j]:
            insert_at = j
            break
    if insert_at is None:
        insert_at = e

    indent = "  "
    for j in range(s, e):
        if "src/" in lines[j]:
            m = re.match(r"^(\s+)\S", lines[j])
            if m:
                indent = m.group(1)
                break

    lines.insert(insert_at, f"{indent}{new_cpp_rel}\n")
    cmake_path.write_text("".join(lines), encoding="utf-8")
    return True, f"Inserted: {new_cpp_rel}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="Server name (any style). Used for file/class names.")
    ap.add_argument(
        "--action-name",
        default=None,
        help='Action name string for registration (default: "fr3_husky_<snake>").',
    )
    ap.add_argument(
        "--action-include",
        default="control_msgs/action/follow_joint_trajectory.hpp",
        help="Action type include path.",
    )
    ap.add_argument(
        "--action-type",
        default="control_msgs::action::FollowJointTrajectory",
        help="C++ action type name.",
    )
    ap.add_argument(
        "--suffix",
        default="_action_server",
        help='File suffix (default "_action_server"). Final files: <snake><suffix>.hpp/.cpp',
    )
    ap.add_argument("--force", action="store_true", help="Overwrite files if they already exist.")
    args = ap.parse_args()

    pkg_root = script_pkg_root()
    cmake_path = pkg_root / "CMakeLists.txt"
    if not cmake_path.exists():
        raise FileNotFoundError(f"CMakeLists.txt not found next to script: {cmake_path}")

    snake = to_snake(args.name)
    class_name = to_camel(snake)
    action_name = args.action_name or default_action_name("fr3_husky", snake)

    hpp_name = f"{snake}{args.suffix}.hpp"
    cpp_name = f"{snake}{args.suffix}.cpp"

    hpp_dir = pkg_root / "include" / "fr3_husky_controller" / "servers" / "fr3_husky"
    cpp_dir = pkg_root / "src" / "servers" / "fr3_husky"

    ensure_dir(hpp_dir)
    ensure_dir(cpp_dir)

    hpp_path = hpp_dir / hpp_name
    cpp_path = cpp_dir / cpp_name

    if (hpp_path.exists() or cpp_path.exists()) and not args.force:
        raise FileExistsError(
            f"Target file exists.\n- {hpp_path}\n- {cpp_path}\nUse --force to overwrite."
        )

    hpp_txt = render_hpp(class_name, args.action_include, args.action_type)
    hpp_include = f"fr3_husky_controller/servers/fr3_husky/{hpp_name}"
    cpp_txt = render_cpp(class_name, hpp_include, action_name)

    hpp_path.write_text(hpp_txt, encoding="utf-8")
    cpp_path.write_text(cpp_txt, encoding="utf-8")

    new_cpp_rel = f"src/servers/fr3_husky/{cpp_name}"
    changed, msg = patch_cmakelists(cmake_path, new_cpp_rel, "src/servers/fr3_husky/")

    print("Generated:")
    print(f"  - {hpp_path.relative_to(pkg_root)}")
    print(f"  - {cpp_path.relative_to(pkg_root)}")
    print("CMakeLists.txt:")
    print(f"  - {msg}")
    if not changed:
        print("  - No changes were needed.")

    print("\nNext steps:")
    print("  1) Fill the TODOs in the generated .hpp/.cpp")
    print("  2) Controller should call ActionServerManager::createAllFR3Husky(...)")
    print("  3) Build: colcon build --packages-select fr3_husky_controller")


if __name__ == "__main__":
    main()
