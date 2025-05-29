import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    pkg_hexapod_sim = get_package_share_directory('hexapod_sim')

    # ROS-Gazebo bridge parameters file
    robot_controllers = os.path.join(
        pkg_hexapod_sim, 'config', 'hexapod_controllers.yaml')

    # Joint state broadcaster spawner node
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Robot controller spawner node
    controller_spawner_node = Node(
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

    return LaunchDescription([
        controller_spawner_node,
        joint_state_broadcaster_spawner,
    ])
