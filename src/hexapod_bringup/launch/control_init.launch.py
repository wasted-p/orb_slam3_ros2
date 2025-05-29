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

    # # Hexapod IK node
    # hexapod_ik_gz_node = Node(
    #     package='hexapod_control',
    #     executable='hexapod_ik_gz_node',
    #     name='hexapod_ik_gz_node',
    #     output="screen",
    #     parameters=[{'robot_description': robot_desc}],
    # )
    #
    # # Hexapod gait planner node
    # hexapod_gait_planner_node = Node(
    #     package='hexapod_gait',
    #     executable='gait_planner_node',
    #     name='gait_planner_node',
    #     output="screen",
    #     parameters=[initial_pose_path],
    # )

    delay_joint_state_broadcaster_after_controller_spawner_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    return LaunchDescription([
        controller_spawner_node,
        delay_joint_state_broadcaster_after_controller_spawner_node,
    ])
