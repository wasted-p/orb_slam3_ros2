import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Share Dir
    share_dir = get_package_share_directory('hexapod_bringup')


    # Load robot description and controller configurations from files
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("hexapod_bringup"),
        "config",
        "hexapod_controllers.yaml",
    ])

    # Process the XACRO file to get robot URDF description
    # xacro_file = os.path.join(get_package_share_directory('hexapod_description'), 'robots', 'hexapod.urdf.xacro')
    # doc = xacro.process_file(xacro_file, mappings={'use_ros2_control': 'true'})

    urdf_file = os.path.join(get_package_share_directory('hexapod_description'), 'robots', 'hexapod.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()


    # Set Gazebo simulation resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(get_package_share_directory('hexapod_sim'), 'worlds'),
            ':' + str(Path(get_package_share_directory('hexapod_description')).parent.resolve())
        ]
    )

    # World argument to specify the Gazebo world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='hexapod_world',
        description='Gazebo world'
    )

    # Include Gazebo simulation launch description
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 4', ' -r'])
        ]
    )

    # ROS-Gazebo bridge parameters file
    bridge_params = os.path.join(share_dir, 'config', 'hexapod_bridge.yaml')

    # ROS-Gazebo bridge node to bridge communication between ROS and Gazebo
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )


    # Set the robot description as a parameter
    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

    # Gazebo spawn entity to load the robot in simulation
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-x', '0.0', '-y', '0.0', '-z', '0.0', '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            '-name', 'hexapod', '-topic', 'robot_description', '-allow_renaming', 'true'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz configuration file for visualization
    rviz_config_file = os.path.join(share_dir, 'rviz', 'sim.rviz')

    # RViz node configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Control node (ros2_control) for managing controllers
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {'use_sim_time': use_sim_time}],
        output="both",
    )

    # Joint state broadcaster spawner node to broadcast joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Robot controller spawner node to spawn controllers for robot
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "legs_joint_trajectory_controller",
            "arm_joint_group_position_controller",
            "--param-file", robot_controllers
        ],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Robot state publisher node to publish the robot state
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters = [{'publish_frequency':50.0,'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # Event handlers to delay node start orders
    delay_control_node_after_robot_state_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[control_node]
        )
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    point_cloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz',
        remappings=[
            ('image_rect', '/depth_camera/depth/image_raw'),
            ('camera_info', '/depth_camera/depth/camera_info'),
            ('points', '/depth_camera/points')
        ],
        parameters=[{'use_sim_time': use_sim_time,
        'queue_size': 10,                      # Increase queue for better sync buffering
        'approximate_sync': True,              # Allow approximate timestamps
                     }],
        output='screen',
    )

    # Return launch description with hexapod-specific nodes
    return LaunchDescription([
        gz_spawn_entity,
        robot_state_pub_node,
        point_cloud_node,
        delay_control_node_after_robot_state_publisher,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        gazebo_resource_path,
        world_arg,
        gazebo,
        ros_gz_bridge_node
    ])
