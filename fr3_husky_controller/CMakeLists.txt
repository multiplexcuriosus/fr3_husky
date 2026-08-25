cmake_minimum_required(VERSION 3.5)
project(fr3_husky_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{ROS_DISTRO} MATCHES "humble")
    add_definitions(-DROS_DISTRO=22)
else()
    add_definitions(-DROS_DISTRO=24)
endif()

# find_package(ros2_control_cmake REQUIRED)
# set_compiler_options()
# export_windows_symbols()


set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    franka_semantic_components
    Franka
    Eigen3
    geometry_msgs
    sensor_msgs
    urdf
    dyros_robot_controller
    fr3_husky_msgs
    franka_msgs
    nav_msgs
    realtime_tools
    tf2
    tf2_msgs
    control_msgs
    controller_manager_msgs
    action_msgs
    std_srvs
    moveit_ros_planning_interface
    srdfdom
)


find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()



generate_parameter_library(fr3_controller_parameters
    src/fr3_controller_parameters.yaml
)

generate_parameter_library(fr3_husky_controller_parameters
    src/fr3_husky_controller_parameters.yaml
)

generate_parameter_library(fr3_action_controller_parameters
    src/fr3_action_controller_parameters.yaml
)

generate_parameter_library(fr3_husky_action_controller_parameters
    src/fr3_husky_action_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
    src/utils/dyros_math.cpp
    src/utils/motion_primitives.cpp

    src/model/model_updater_base.cpp
    src/model/fr3_model_updater.cpp
    src/model/fr3_husky_model_updater.cpp

    src/fr3_action_controller.cpp
    src/fr3_husky_action_controller.cpp

    src/servers/action_server_base.cpp  
    src/servers/idle_control.cpp

    src/servers/fr3/gravity_compensation_action_server.cpp
    src/servers/fr3_husky/gravity_compensation_action_server.cpp
    src/servers/fr3/omega_haptic_action_server.cpp
    src/servers/fr3_husky/omega_haptic_action_server.cpp
    
    src/test_fr3_controller.cpp
    src/test_fr3_husky_controller.cpp
    src/servers/fr3/joint_trajectory_controller_action_server.cpp
    src/servers/fr3/move_to_joint_action_server.cpp
    src/servers/fr3/vive_tracker_action_server.cpp
    src/servers/fr3/cartesian_executor_action_server.cpp
    src/servers/fr3/trajectory_executor_action_server.cpp
    src/servers/fr3/cont_tracker_action_server.cpp


    src/servers/fr3_husky/joint_trajectory_controller_action_server.cpp
    src/servers/fr3_husky/move_to_joint_action_server.cpp
    src/servers/fr3_husky/vive_tracker_action_server.cpp
    src/servers/fr3_husky/husky_move_action_server.cpp

)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wno-error=missing-braces)
endif()

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
                      fr3_controller_parameters
                      fr3_husky_controller_parameters
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_action::rclcpp_action
                      rclcpp_lifecycle::rclcpp_lifecycle
                      urdf::urdf
                      dyros_robot_controller::dyros_robot_controller
                      Franka::Franka
)

pluginlib_export_plugin_description_file(controller_interface fr3_husky_plugin.xml)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    ament_index_cpp
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    Franka
    franka_semantic_components
    geometry_msgs
    sensor_msgs
    urdf
    dyros_robot_controller
    fr3_husky_msgs
    franka_msgs
    nav_msgs
    realtime_tools
    tf2
    tf2_msgs
    controller_manager_msgs
    action_msgs
    std_srvs
    moveit_ros_planning_interface
    srdfdom
)

install(
    DIRECTORY include/
    DESTINATION include/fr3_husky_controller
)


install(TARGETS ${PROJECT_NAME} 
            fr3_controller_parameters 
            fr3_husky_controller_parameters
            fr3_action_controller_parameters
            fr3_husky_action_controller_parameters
        EXPORT export_${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)
    
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_acceleration_limited_scalar_generator
        test/test_acceleration_limited_scalar_generator.cpp)

    if(TARGET test_acceleration_limited_scalar_generator)
        target_include_directories(test_acceleration_limited_scalar_generator PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include)
    endif()

    ament_add_gtest(test_trajectory_line_math
        test/test_trajectory_line_math.cpp)

    if(TARGET test_trajectory_line_math)
        target_include_directories(test_trajectory_line_math PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include)
    endif()

    ament_add_gtest(test_action_server_lifecycle
        test/test_action_server_lifecycle.cpp)

    if(TARGET test_action_server_lifecycle)
        target_include_directories(test_action_server_lifecycle PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include)
        ament_target_dependencies(test_action_server_lifecycle
            rclcpp
            rclcpp_action
            rclcpp_lifecycle
            fr3_husky_msgs)
        target_link_libraries(test_action_server_lifecycle
            ${PROJECT_NAME})
    endif()
endif()

ament_package()
