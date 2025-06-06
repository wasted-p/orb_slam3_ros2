
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
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

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

    # Launch trajectory node after startup controllers exit
    launch_trajectory_on_startup_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=startup_controllers_node,
            on_exit=[hexapod_trajectory_node]
        )
    )

    return LaunchDescription([
        # hexapod_gait_planner_node ,
        startup_controllers_node,
        launch_trajectory_on_startup_exit,
    ])
