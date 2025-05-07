import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Define paths to package directories
    pkg_hexapod_description = os.path.join(get_package_share_directory('hexapod_description'))
    pkg_hexapod_sim = os.path.join(get_package_share_directory('hexapod_sim'))

    # Set Gazebo simulation resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_hexapod_sim, 'worlds'),
            ':' + str(Path(pkg_hexapod_description).parent.resolve())
        ]
    )

    # World argument to specify the Gazebo world file
    arguments = LaunchDescription([
        DeclareLaunchArgument('world', default_value='hexapod_world', description='Gazebo world'),
    ])

    # Include Gazebo simulation launch description
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 4', ' -r'])
        ]
    )

    # Load robot description and controller configurations from files
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("hexapod_description"),
        "config",
        "hexapod_controllers.yaml",
    ])

    # Process the XACRO file to get robot URDF description
    xacro_file = os.path.join(pkg_hexapod_description, 'robots', 'hexapod.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_ros2_control': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    # Set the robot description as a parameter
    params = {'robot_description': robot_desc}

    # Gazebo spawn entity to load the robot in simulation
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-x', '0.0', '-y', '0.0', '-z', '0.0', '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            '-name', 'hexapod', '-topic', 'robot_description', '-allow_renaming', 'true'
        ],
    )

    # RViz configuration file for visualization
    rviz_config_file = os.path.join(pkg_hexapod_description, 'rviz', 'hexapod.rviz')

    # RViz node configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Control node (ros2_control) for managing controllers
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {'use_sim_time': False}],
        output="both",
    )

    # ROS-Gazebo bridge parameters file
    bridge_params = os.path.join(pkg_hexapod_sim, 'config', 'hexapod_bridge.yaml')

    # ROS-Gazebo bridge node to bridge communication between ROS and Gazebo
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # Joint state broadcaster spawner node to broadcast joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
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
        output="screen"
    )

    # Robot state publisher node to publish the robot state
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[params],
    )

    # Event handlers to delay node start orders
    # Delay the start of control_node until robot_state_pub_node has exited
    delay_control_node_after_robot_state_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=robot_state_pub_node, on_exit=[control_node])
    )

    # Delay RViz start after joint_state_broadcaster has started
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node])
    )

    # Delay joint_state_broadcaster start after robot_controller_spawner has started
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=robot_controller_spawner, on_exit=[joint_state_broadcaster_spawner])
    )

    # List of all nodes to launch
    nodes = [
        gazebo_resource_path,         # Set simulation resource path
        arguments,                    # Launch arguments
        gazebo,                       # Gazebo simulation
        gz_spawn_entity,              # Spawn the robot in Gazebo
        robot_state_pub_node,         # Robot state publisher
        delay_control_node_after_robot_state_publisher,  # Delay control node
        robot_controller_spawner,     # Spawn robot controllers
        delay_rviz_after_joint_state_broadcaster_spawner,  # Delay RViz start
        delay_joint_state_broadcaster_after_robot_controller_spawner,  # Delay joint state broadcaster
        ros_gz_bridge_node            # ROS-Gazebo bridge node
    ]

    # Return launch description with the nodes and declared arguments
    return LaunchDescription(declared_arguments + nodes)
