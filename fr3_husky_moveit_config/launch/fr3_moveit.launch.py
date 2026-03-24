import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


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


def _launch_setup(context, *args, **kwargs):
    robot_side_raw = LaunchConfiguration('robot_side').perform(context)
    robot_sides = _parse_robot_side(robot_side_raw)

    if not robot_sides:
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")

    normalized_sides = [side.strip().lower() for side in robot_sides if side.strip()]
    if len(normalized_sides) == 1 and normalized_sides[0] == 'dual':
        normalized_sides = ['left', 'right']
    elif 'dual' in normalized_sides:
        raise RuntimeError("robot_side cannot mix 'dual' with other values.")

    allowed = {'left', 'right'}
    if any(side not in allowed for side in normalized_sides):
        raise RuntimeError("robot_side must be 'left', 'right', or 'dual'.")
    if len(set(normalized_sides)) != len(normalized_sides):
        raise RuntimeError("robot_side entries must be unique.")

    is_dual = len(normalized_sides) == 2
    robot_side = normalized_sides[0]  # used in single mode

    use_mujoco        = LaunchConfiguration('use_mujoco').perform(context)
    load_gripper      = LaunchConfiguration('load_gripper').perform(context)
    load_mobile       = LaunchConfiguration('load_mobile').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands').perform(context)
    namespace         = LaunchConfiguration('namespace').perform(context)

    pkg_desc   = get_package_share_directory('fr3_husky_description')
    pkg_moveit = get_package_share_directory('fr3_husky_moveit_config')
    pkg_ctrl   = get_package_share_directory('fr3_husky_controller')

    # ---- URDF for move_group (ros2_control:=false — no hw tags needed for planning) ----
    if is_dual:
        urdf_path    = os.path.join(pkg_desc, 'robots', 'dual_fr3.urdf.xacro')
        srdf_path    = os.path.join(pkg_desc, 'robots', 'dual_fr3.srdf.xacro')
        urdf_mappings = {
            'ros2_control': 'false', 'with_sc': 'false', 'fix_finger': 'false',
            'hand': load_gripper, 'mobile': load_mobile,
        }
        srdf_mappings = {'hand': load_gripper, 'mobile': load_mobile}
        config_subdir        = 'dual'
        controllers_yaml_rel = os.path.join('config', 'dual', 'dual_fr3_controllers.yaml')
        jsp_sources          = ['dual_fr3/joint_states']
    else:
        urdf_path    = os.path.join(pkg_desc, 'robots', 'single_fr3.urdf.xacro')
        srdf_path    = os.path.join(pkg_desc, 'robots', 'single_fr3.srdf.xacro')
        urdf_mappings = {
            'ros2_control': 'false', 'with_sc': 'false', 'fix_finger': 'false',
            'side': robot_side, 'hand': load_gripper, 'mobile': load_mobile,
        }
        srdf_mappings = {'side': robot_side, 'hand': load_gripper, 'mobile': load_mobile}
        config_subdir        = robot_side
        controllers_yaml_rel = os.path.join('config', robot_side, 'single_fr3_controllers.yaml')
        jsp_sources          = [f'{robot_side}_fr3/joint_states']

    robot_description_str = xacro.process_file(urdf_path, mappings=urdf_mappings).toprettyxml(indent='  ')
    robot_description_semantic_str = xacro.process_file(srdf_path, mappings=srdf_mappings).toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_str}
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_str}

    kinematics_yaml = load_yaml(
        'fr3_husky_moveit_config', os.path.join('config', config_subdir, 'kinematics.yaml')
    ) or {}
    moveit_simple_controllers_yaml = load_yaml(
        'fr3_husky_moveit_config', controllers_yaml_rel
    ) or {}
    ompl_planning_yaml = load_yaml(
        'fr3_husky_moveit_config', os.path.join('config', 'ompl_planning.yaml')
    ) or {}

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    nodes = []

    # ---- Include fr3_action_controller.launch.py (hardware, ros2_control_node, RSP, JSP for real hw) ----
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ctrl, 'launch', 'fr3_action_controller.launch.py')
            ),
            launch_arguments={
                'robot_side': robot_side_raw,
                'use_mujoco': use_mujoco,
                'load_gripper': load_gripper,
                'load_mobile': load_mobile,
                'use_fake_hardware': use_fake_hardware,
                'fake_sensor_commands': fake_sensor_commands,
                'namespace': namespace,
                'launch_rviz': 'false',
            }.items(),
        )
    )

    # ---- move_group ----
    nodes.append(
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            namespace=namespace,
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                ompl_planning_pipeline_config,
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor_parameters,
            ],
        )
    )

    # ---- RViz2 with MoveIt config ----
    rviz_config = os.path.join(pkg_moveit, 'rviz', 'moveit.rviz')
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config],
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_planning_pipeline_config,
                kinematics_yaml,
            ],
        )
    )

    # ---- joint_state_publisher: MuJoCo only ----
    # In MuJoCo mode, fr3_action_controller skips JSP (RSP subscribes directly to {side}_fr3/joint_states).
    # move_group needs /joint_states, so we bridge here.
    if use_mujoco.lower() == 'true':
        nodes.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher_moveit',
                namespace=namespace,
                parameters=[{'source_list': jsp_sources, 'rate': 30}],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_side',          default_value='left',  description='Robot side: left, right, or dual'),
        DeclareLaunchArgument('namespace',           default_value='',      description='Namespace'),
        DeclareLaunchArgument('load_gripper',        default_value='true',  description='Load gripper (true/false)'),
        DeclareLaunchArgument('load_mobile',         default_value='false', description='Load mobile base (true/false)'),
        DeclareLaunchArgument('use_mujoco',          default_value='false', description='Use MuJoCo hardware interface'),
        DeclareLaunchArgument('use_fake_hardware',   default_value='false', description='Use fake hardware'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false', description='Fake sensor commands'),
        OpaqueFunction(function=_launch_setup),
    ])
