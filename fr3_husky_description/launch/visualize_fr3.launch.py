import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def robot_state_publisher_spawner(context: LaunchContext, robot_side, load_gripper, load_mobile):
    robot_side_str = context.perform_substitution(robot_side).strip().lower()
    load_gripper_str = context.perform_substitution(load_gripper)
    load_mobile_str = context.perform_substitution(load_mobile)
    if robot_side_str in ('left', 'right'):
        franka_xacro_filepath = os.path.join(
            get_package_share_directory('fr3_husky_description'),
            'robots',
            'single_fr3.urdf.xacro',
        )
        mappings = {
            'hand': load_gripper_str,
            'mobile': load_mobile_str,
            'side': robot_side_str,
            'with_sc': 'false',
            'ros2_control': 'false',
            'use_fake_hardware': 'false',
            'fake_sensor_commands': 'false',
            'fix_finger': 'false',
        }
    elif robot_side_str == 'dual':
        franka_xacro_filepath = os.path.join(
            get_package_share_directory('fr3_husky_description'),
            'robots',
            'dual_fr3.urdf.xacro',
        )
        mappings = {
            'hand': load_gripper_str,
            'mobile': load_mobile_str,
            'with_sc': 'false',
            'ros2_control': 'false',
            'use_fake_hardware': 'false',
            'fake_sensor_commands': 'false',
            'fix_finger': 'false',
        }
    else:
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")
    
    rviz_file = os.path.join(
        get_package_share_directory('fr3_husky_description'),
        'rviz',
        'visualize_fr3.rviz',
    )

    robot_description = xacro.process_file(franka_xacro_filepath, mappings=mappings).toprettyxml(indent='  ')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_file],
        ),
    ]


def generate_launch_description():
    robot_side_parameter_name = 'robot_side'
    robot_side = LaunchConfiguration(robot_side_parameter_name)

    load_gripper_parameter_name = 'load_gripper'
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)

    load_mobile_parameter_name = 'load_mobile'
    load_mobile = LaunchConfiguration(load_mobile_parameter_name)

    robot_state_publisher_spawner_opaque_function = OpaqueFunction(
        function=robot_state_publisher_spawner, args=[robot_side, load_gripper, load_mobile]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_side_parameter_name,
                default_value='left',
                description="Robot side: left, right, or dual",
            ),
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value='true',
                description='Use end-effector if true. Default value is franka hand. '
                'Robot is loaded without end-effector otherwise',
            ),
            DeclareLaunchArgument(
                load_mobile_parameter_name,
                default_value='false',
                description='Use Mobile Husky as base if true'
                'Base is loaded box-shaped otherwise',
            ),
            robot_state_publisher_spawner_opaque_function,
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
            ),
        ]
    )
