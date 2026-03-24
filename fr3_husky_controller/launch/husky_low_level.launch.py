import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    robot_side = 'right'
    load_gripper = LaunchConfiguration('load_gripper').perform(context)
    load_mobile = LaunchConfiguration('load_mobile').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    right_robot_ip = LaunchConfiguration('right_robot_ip').perform(context)

    pkg_desc = get_package_share_directory('fr3_husky_description')
    pkg_ctrl = get_package_share_directory('fr3_husky_controller')

    urdf_path = os.path.join(pkg_desc, 'robots', 'single_fr3.urdf.xacro')
    xacro_mappings = {
        'ros2_control': 'true',
        'with_sc': 'false',
        'fix_finger': 'false',
        'side': robot_side,
        'hand': load_gripper,
        'mobile': load_mobile,
        'use_mujoco': 'false',
        'use_fake_hardware': 'false',
        'fake_sensor_commands': 'false',
    }

    robot_description = xacro.process_file(
        urdf_path, mappings=xacro_mappings
    ).toprettyxml(indent='  ')

    controllers_yaml = os.path.join(pkg_ctrl, 'config', 'fr3_ros_controllers.yaml')
    joint_states_topic = f'{robot_side}_fr3/joint_states'

    main_controller = f'{robot_side}_fr3_action_controller'
    franka_broadcaster_names = [f'{robot_side}_franka_robot_state_broadcaster']

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[controllers_yaml, {'robot_description': robot_description}],
        remappings=[('joint_states', joint_states_topic)],
        output='screen',
        on_exit=Shutdown(),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen',
    )

    franka_broadcaster_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=[broadcaster_name, '--controller-manager-timeout', '60'],
            output='screen',
        )
        for broadcaster_name in franka_broadcaster_names
    ]

    main_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[main_controller, '--controller-manager-timeout', '60'],
        output='screen',
    )

    gripper_includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_gripper'),
                    'launch',
                    'gripper.launch.py',
                ])
            ),
            launch_arguments={
                'robot_ip': right_robot_ip,
                'use_fake_hardware': 'false',
                'namespace': namespace,
                'hand_prefix': robot_side,
            }.items(),
            condition=IfCondition(LaunchConfiguration('load_gripper')),
        )
    ]

    return [
        robot_state_publisher_node,
        ros2_control_node,
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=3.0, actions=franka_broadcaster_spawners),
        TimerAction(period=4.0, actions=[main_controller_spawner]),
        TimerAction(period=5.0, actions=gripper_includes),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for the robot'),
        DeclareLaunchArgument('load_gripper', default_value='true', description='Load gripper (true/false)'),
        DeclareLaunchArgument('load_mobile', default_value='false', description='Load mobile base (true/false)'),
        DeclareLaunchArgument('right_robot_ip', default_value='172.16.6.6', description='IP of right Franka'),
        OpaqueFunction(function=_launch_setup),
    ])
