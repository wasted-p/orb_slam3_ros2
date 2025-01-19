from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
# Declare launch arguments (parameters you want to pass)
    DeclareLaunchArgument('controller_name', default_value='joint_trajectory_controller', description='The name of the joint trajectory controller'),
    DeclareLaunchArgument('joints', default_value='["arm_rotator_joint"]', description='List of joint names'),

    return LaunchDescription([
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),
        # Node(package = "joy",executable = "joy_node"),
        Node(package='trajectory_pub',
                name='joint_trajectory_controller_client',
                executable='jtc_client.py',
                output='screen',
                parameters=[
                {'controller_name': LaunchConfiguration('controller_name')},
                {'joints': LaunchConfiguration('joints')}
            ]),
    ])
