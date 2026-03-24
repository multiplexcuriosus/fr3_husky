cmake_minimum_required(VERSION 3.5)
project(fr3_husky_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "action/GravityCompensation.action"
    "action/OmegaHaptic.action"
    "action/MoveToJoint.action"
    "action/ViveTracker.action"
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
