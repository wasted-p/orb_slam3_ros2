
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler


def generate_launch_description():

    ####################################
    # Params
    ####################################
    # use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    share_dir = get_package_share_directory('hexapod_bringup')
    initial_pose_path = os.path.join(share_dir, 'config', 'initial_pose.yml')

    # action_server_node = Node(
    #     package='hexapod_action',
    #     executable='action_server_node',
    #     name='action_server',
    #     output="screen",
    #     parameters=[initial_pose_path],
    # )

    startup_controllers_node = Node(
        package='hexapod_control',
        executable='startup_controllers',
        name='startup_controllers',
        output='screen'
    )

    # hexapod_trajectory_node = Node(
    #     package='hexapod_control',
    #     executable='hexapod_trajectory_node',
    #     name='hexapod_trajectory_node',
    #     output='screen',
    #     # Change to 'debug', 'warn', 'error', etc.
    #     # arguments=['--ros-args', '--log-level', 'debug'],
    # )
    #
    # # Launch trajectory node after startup controllers exit
    # launch_trajectory_on_startup_exit = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=startup_controllers_node,
    #         on_exit=[hexapod_trajectory_node]
    #     )
    # )

    return LaunchDescription([
        # action_server_node,
        startup_controllers_node,
        # launch_trajectory_on_startup_exit,
        # use_sim_time
    ])
