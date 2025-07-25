from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    share_dir = get_package_share_directory('hexapod_bringup')
    default_rviz_config = os.path.join(share_dir, 'rviz', 'default.rviz')
    urdf_file = os.path.join(get_package_share_directory('hexapod_description'), 'robots', 'hexapod.urdf')

    with open(urdf_file, 'r') as f:
        robot_urdf = f.read()

    # Launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz_args_arg = DeclareLaunchArgument(
        'rviz_args',
        default_value='',
        description='Extra arguments to pass to RViz2'
    )

    use_joint_state_gui_arg = DeclareLaunchArgument(
        'use_joint_state_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui if true'
    )

    # Substitutions
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_args = LaunchConfiguration('rviz_args')
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time}
        ]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_joint_state_gui)
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", rviz_config,
            rviz_args
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        use_joint_state_gui_arg,
        rviz_args_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node ,
        rviz_node
    ])
