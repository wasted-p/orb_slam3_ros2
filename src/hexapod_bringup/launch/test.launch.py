
import os
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnShutdown


def generate_launch_description():
    # FIXME: Remove this
    # use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    # # Share Dir
    # pkg_hexapod_description = get_package_share_directory(
    #     'hexapod_description')
    #
    # # Process the URDF file
    # urdf_file = os.path.join(pkg_hexapod_description, 'robots', 'hexapod.urdf')
    # with open(urdf_file, 'r') as f:
    #     robot_desc = f.read()
    #
    # # Robot state publisher node
    # robot_state_pub_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name='robot_state_publisher',
    #     output="both",
    #     parameters=[{'publish_frequency': 50.0,
    #                  'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    # )
    #
    # Hexapod gait planner node
    # hexapod_action_server_node = Node(
    #     package='hexapod_gait',
    #     executable='action_server_node',
    #     name='action_server',
    #     output="screen",
    #     parameters=[initial_pose_path],
    # )

    startup_controllers_node = Node(
        package='hexapod_control',
        executable='startup_controllers_node',
        name='startup_controllers',
        output='screen'
    )

    hexapod_trajectory_node = Node(
        package='hexapod_control',
        executable='hexapod_trajectory_node',
        name='hexapod_trajectory_node',
        output='screen'
    )

    # shutdown_controllers_node = Node(
    #     package='hexapod_control',
    #     executable='shutdown_controllers_node',
    #     name='shutdown_controllers',
    #     output='screen'
    # )

    # Launch trajectory node after startup controllers exit
    launch_trajectory_on_startup_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=startup_controllers_node,
            on_exit=[hexapod_trajectory_node]
        )
    )

    return LaunchDescription([
        startup_controllers_node,
        launch_trajectory_on_startup_exit,
    ])
