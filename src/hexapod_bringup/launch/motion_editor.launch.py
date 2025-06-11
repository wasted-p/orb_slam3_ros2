from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Paths
    share_dir = get_package_share_directory('hexapod_bringup')
    default_rviz_config = os.path.join(
        share_dir, 'rviz', 'motion_editor.rviz')
    urdf_path = os.path.join(get_package_share_directory(
        'hexapod_description'), 'robots', 'hexapod.urdf')

    share_dir = get_package_share_directory('hexapod_bringup')
    initial_pose_path = os.path.join(share_dir, 'config', 'initial_pose.yml')
    initial_positions_path = os.path.join(
        share_dir, 'config', 'initial_positions.yml')
    # actions_path = os.path.join(share_dir, 'config', 'actions.yml')
    # motion_definitions_path = os.path.join(
    #     share_dir, 'config', 'motion_definitions.yml')

    with open(urdf_path, 'r') as f:
        robot_urdf = f.read()

    with open(initial_positions_path, 'r') as f:
        initial_positions = f.read()

    # Launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz_args_arg = DeclareLaunchArgument(
        'rviz_args',
        default_value='',
        description='Extra arguments to pass to RViz2'
    )

    use_joint_state_gui_arg = DeclareLaunchArgument(
        'use_joint_state_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui if true'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # Substitutions
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_args = LaunchConfiguration('rviz_args')
    # log_level = LaunchConfiguration('log_level')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time}
        ],
        # arguments=['--ros-args', '--log-level', log_level],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config, rviz_args],
    )

    motion_server_node = Node(
        package='hexapod_motion',
        executable='motion_server',
        name='motion_server',
        output="screen",
        parameters=[initial_pose_path]
    )

    # Nodes
    joint_state_publisher = Node(
        package='hexapod_control',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output="screen",
        parameters=[{'initial_positions': initial_positions}],
    )

    kinematics_service = Node(
        package='hexapod_control',
        executable='kinematics_service',
        name='kinematics_service',
        output="screen",
        parameters=[{'robot_description': robot_urdf}],
    )

    pose_publisher = Node(
        package='hexapod_control',
        executable='pose_publisher',
        name='pose_publisher',
        output="screen",
    )

    visualization_server = Node(
        package='hexapod_control',
        executable='visualization_server',
        name='visualization_server',
        output="screen",
    )

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        rviz_args_arg,
        use_joint_state_gui_arg,
        log_level_arg,
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher,
        motion_server_node,
        kinematics_service,
        pose_publisher,
        visualization_server
    ])
