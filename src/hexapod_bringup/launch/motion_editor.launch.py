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
    hexapod_urdf_path = os.path.join(get_package_share_directory(
        'hexapod_description'), 'robots', 'hexapod.urdf')
    arm_urdf_path = os.path.join(get_package_share_directory(
        'hexapod_description'), 'robots', 'arm.urdf')
    share_dir = get_package_share_directory('hexapod_bringup')
    initial_pose_path = os.path.join(share_dir, 'config', 'initial_pose.yml')
    initial_positions_path = os.path.join(
        share_dir, 'config', 'initial_positions.yml')
    arm_initial_positions_path = os.path.join(
        share_dir, 'config', 'arm_initial_positions.yml')
    # actions_path = os.path.join(share_dir, 'config', 'actions.yml')
    # motion_definitions_path = os.path.join(
    #     share_dir, 'config', 'motion_definitions.yml')

    with open(hexapod_urdf_path, 'r') as f:
        hexapod_urdf = f.read()

    with open(arm_urdf_path, 'r') as f:
        arm_urdf = f.read()

    with open(initial_positions_path, 'r') as f:
        initial_positions = f.read()

    with open(initial_pose_path, 'r') as f:
        hexapod_initial_pose = f.read()

    with open(initial_pose_path, 'r') as f:
        arm_initial_pose = f.read()

    with open(arm_initial_positions_path, 'r') as f:
        arm_initial_positions = f.read()

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
    hexapod_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='hexapod',
        parameters=[
            {'robot_description': hexapod_urdf},
            {'use_sim_time': use_sim_time},
            {'prefix': 'hexapod'}
        ],
        remappings=[
            ('/joint_states', 'hexapod/joint_states')
        ]
    )

    arm_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='arm',
        parameters=[
            {'robot_description': arm_urdf},
            {'use_sim_time': use_sim_time},
            {'prefix': 'arm'}
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config, rviz_args],
    )

    # Nodes
    hexapod_joint_state_publisher = Node(
        package='hexapod_control',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output="screen",
        parameters=[{'initial_positions': initial_positions},
                    {'prefix': 'hexapod'}]
    )

    arm_joint_state_publisher = Node(
        package='hexapod_control',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output="screen",
        parameters=[{'initial_positions': arm_initial_positions},
                    {'prefix': 'arm'}]
    )

    hexapod_kinematics_service = Node(
        package='hexapod_control',
        executable='kinematics_service',
        name='kinematics_service',
        output="screen",
        parameters=[{'robot_description': hexapod_urdf},
                    {'prefix': 'hexapod'}],
    )

    arm_kinematics_service = Node(
        package='hexapod_control',
        executable='kinematics_service',
        name='kinematics_service',
        output="screen",
        parameters=[{'robot_description': hexapod_urdf},
                    {'prefix': 'arm'}],
    )

    hexapod_pose_publisher = Node(
        package='hexapod_control',
        executable='pose_publisher',
        name='pose_publisher',
        output="screen",
        parameters=[{'initial_pose': hexapod_initial_pose},
                    {'prefix': 'hexapod'}],
    )

    arm_pose_publisher = Node(
        package='hexapod_control',
        executable='pose_publisher',
        name='pose_publisher',
        output="screen",
        parameters=[{'initial_pose': arm_initial_pose},
                    {'prefix': 'arm'}],
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
        hexapod_robot_state_publisher,
        arm_robot_state_publisher,
        rviz_node,
        hexapod_joint_state_publisher,
        arm_joint_state_publisher,
        hexapod_kinematics_service,
        arm_kinematics_service,
        hexapod_pose_publisher,
        arm_pose_publisher,
        visualization_server,
    ])
