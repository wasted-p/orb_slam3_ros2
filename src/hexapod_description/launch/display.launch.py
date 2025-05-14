from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    # Declare 'sim' argument
    sim_arg = DeclareLaunchArgument(
        name='sim',
        default_value='false',
        description='Enable simulation mode (e.g., with Gazebo)'
    )

    sim = LaunchConfiguration('sim')

    # Paths
    share_dir = get_package_share_directory('hexapod_description')
    xacro_file = os.path.join(share_dir, 'robots', 'hexapod.urdf.xacro')
    robot_urdf = xacro.process_file(xacro_file).toxml()

    # RViz config files
    rviz_config_sim = os.path.join(share_dir, 'config', 'hexapod_sim.rviz')
    rviz_config_real = os.path.join(share_dir, 'config', 'hexapod_standalone.rviz')

    # robot_state_publisher (used in standalone only)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': False}
        ],
        condition=UnlessCondition(sim)
    )

    # joint_state_publisher (standalone)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
        condition=UnlessCondition(sim)
    )

    # joint_state_publisher_gui (standalone)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': False}],
        condition=UnlessCondition(sim)
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_sim],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(sim)
    )

    rviz_node_standalone = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_real],
        parameters=[{'use_sim_time': False}],
        condition=UnlessCondition(sim)
    )

    return LaunchDescription([
        sim_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        rviz_node_standalone
    ])
