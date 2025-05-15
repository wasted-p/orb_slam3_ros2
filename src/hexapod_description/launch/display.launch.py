from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    share_dir = get_package_share_directory('hexapod_description')

    urdf_file = os.path.join(share_dir, 'robots', 'hexapod.urdf')
    with open(urdf_file, 'r') as f:
        robot_urdf = f.read()

    # RViz config files
    rviz_config = os.path.join(share_dir, 'config', 'default.rviz')

    # robot_state_publisher (used in standalone only)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': False}
        ],
    )

    # joint_state_publisher (standalone)
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': False}],
    # )

    # joint_state_publisher_gui (standalone)
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     parameters=[{'use_sim_time': False}],
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", rviz_config,
            # "--ros-args", "--log-level", "debug"  # Add this line
        ],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node,
    ])
