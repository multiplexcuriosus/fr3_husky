import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_robot_side(raw_value):
    if raw_value is None:
        return []
    raw_value = raw_value.strip()
    if not raw_value:
        return []
    try:
        parsed = yaml.safe_load(raw_value)
    except yaml.YAMLError:
        parsed = raw_value
    if isinstance(parsed, list):
        values = parsed
    elif isinstance(parsed, str):
        if ',' in parsed:
            values = [item.strip() for item in parsed.split(',') if item.strip()]
        else:
            values = [parsed.strip()] if parsed.strip() else []
    else:
        values = [str(parsed).strip()]
    return [value for value in values if value]


def _normalize_robot_sides(robot_sides):
    if not robot_sides:
        return []
    normalized = [side.strip().lower() for side in robot_sides if side.strip()]
    if len(normalized) == 1 and normalized[0] == 'dual':
        return ['left', 'right']
    if 'dual' in normalized:
        raise RuntimeError("robot_side cannot mix 'dual' with other values.")
    return normalized


def _launch_setup(context, *args, **kwargs):
    robot_sides = _normalize_robot_sides(
        _parse_robot_side(LaunchConfiguration('robot_side').perform(context))
    )
    use_mujoco        = LaunchConfiguration('use_mujoco').perform(context)
    load_gripper      = LaunchConfiguration('load_gripper').perform(context)
    load_mobile       = LaunchConfiguration('load_mobile').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands').perform(context)
    namespace         = LaunchConfiguration('namespace').perform(context)
    controller_name   = LaunchConfiguration('controller_name').perform(context)

    if not robot_sides:
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")
    allowed = {'left', 'right'}
    if any(side not in allowed for side in robot_sides):
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")
    if len(set(robot_sides)) != len(robot_sides):
        raise RuntimeError("robot_side entries must be unique.")

    is_dual = len(robot_sides) == 2

    pkg_desc = get_package_share_directory('fr3_husky_description')
    pkg_ctrl = get_package_share_directory('fr3_husky_controller')

    # URDF + MJCF paths 
    if is_dual:
        urdf_path = os.path.join(pkg_desc, 'robots', 'dual_fr3.urdf.xacro')
        mjcf_path = os.path.join(pkg_desc, 'mjcf', 'dual_fr3.xml.xacro')
        xacro_mappings = {
            'ros2_control': 'true', 'with_sc': 'false', 'fix_finger': 'false',
            'hand': load_gripper, 'mobile': load_mobile,
            'use_mujoco': use_mujoco, 'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
        }
    else:
        urdf_path = os.path.join(pkg_desc, 'robots', 'single_fr3.urdf.xacro')
        mjcf_path = os.path.join(pkg_desc, 'mjcf', 'single_fr3.xml.xacro')
        xacro_mappings = {
            'ros2_control': 'true', 'with_sc': 'false', 'fix_finger': 'false',
            'side': robot_sides[0], 'hand': load_gripper, 'mobile': load_mobile,
            'use_mujoco': use_mujoco, 'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
        }

    robot_description = xacro.process_file(urdf_path, mappings=xacro_mappings).toprettyxml(indent='  ')

    # Controllers YAML
    controllers_yaml = os.path.join(pkg_ctrl, 'config', 'fr3_ros_controllers.yaml')

    # Topic names 
    joint_states_topic = 'dual_fr3/joint_states' if is_dual else f'{robot_sides[0]}_fr3/joint_states'
    if is_dual:
        jsp_sources = [joint_states_topic,
                       'left_franka_gripper/joint_states',
                       'right_franka_gripper/joint_states']
    else:
        jsp_sources = [joint_states_topic, f'{robot_sides[0]}_franka_gripper/joint_states']

    #  controller_manager parameters
    cm_params = [controllers_yaml, {'robot_description': robot_description}]
    if use_mujoco.lower() == 'true':
        xacro_args = f' hand:={load_gripper} mobile:={load_mobile}'
        if not is_dual:
            xacro_args += f' side:={robot_sides[0]}'
        cm_params.extend([
            {'mujoco_scene_xacro_path': mjcf_path},
            {'mujoco_scene_xacro_args': xacro_args},
        ])

    # ── robot_state_publisher: direct subscription when using MuJoCo ──────
    # (avoids invalid JointState messages from joint_state_publisher at startup)
    rsp_remappings = [('joint_states', joint_states_topic)] if use_mujoco.lower() == 'true' else []

    # Controller / broadcaster names
    main_controller = f'dual_{controller_name}' if is_dual else f'{robot_sides[0]}_{controller_name}'
    franka_broadcaster_names = (
        ['left_franka_robot_state_broadcaster', 'right_franka_robot_state_broadcaster']
        if is_dual
        else [f'{robot_sides[0]}_franka_robot_state_broadcaster']
    )

    # Gripper IPs
    side_ips = {'left': '172.16.5.5', 'right': '172.16.6.6'}

    #  Node list
    nodes = [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', os.path.join(pkg_ctrl, 'rviz', 'fr3.rviz')],
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': robot_description}],
            remappings=rsp_remappings,
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=cm_params,
            remappings=[('joint_states', joint_states_topic)],
            output='screen',
            on_exit=Shutdown(),
        ),
        # joint_state_publisher: only for real hardware (merges arm + gripper joint states).
        # For MuJoCo, robot_state_publisher subscribes directly via rsp_remappings above.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{'source_list': jsp_sources, 'rate': 30}],
            output='screen',
            condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('use_mujoco'), "' == 'true'"])),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=[main_controller, '--controller-manager-timeout', '60'],
            output='screen',
        ),
    ]

    # franka_robot_state_broadcaster: real hardware only (skip for fake or mujoco)
    for broadcaster_name in franka_broadcaster_names:
        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=[broadcaster_name],
                output='screen',
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('use_fake_hardware'), "' == 'false' and '",
                    LaunchConfiguration('use_mujoco'), "' != 'true'",
                ])),
            )
        )

    # gripper: real hardware only (skip for mujoco), one per side
    gripper_sides = ['left', 'right'] if is_dual else [robot_sides[0]]
    for side in gripper_sides:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])
                ),
                launch_arguments={
                    'robot_ip': side_ips[side],
                    'use_fake_hardware': use_fake_hardware,
                    'namespace': namespace,
                    'hand_prefix': side,
                }.items(),
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('load_gripper'), "' == 'true' and '",
                    LaunchConfiguration('use_mujoco'), "' != 'true'",
                ])),
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller_name',   default_value='test_fr3_controller', description='Base controller name (prefixed with left_/right_/dual_)'),
        DeclareLaunchArgument('robot_side',        default_value='left',  description="Robot side: left, right, or dual"),
        DeclareLaunchArgument('namespace',         default_value='',      description='Namespace for the robot'),
        DeclareLaunchArgument('load_gripper',      default_value='true',  description='Load gripper (true/false)'),
        DeclareLaunchArgument('load_mobile',       default_value='false', description='Load mobile base (true/false)'),
        DeclareLaunchArgument('use_mujoco',        default_value='false', description='Use MuJoCo hardware interface'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Use fake hardware'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false', description='Fake sensor commands'),
        OpaqueFunction(function=_launch_setup),
    ])
