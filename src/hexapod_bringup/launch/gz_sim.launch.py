from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    # Share directory
    share_dir = get_package_share_directory('hexapod_bringup')
    pkg_hexapod_sim = get_package_share_directory('hexapod_sim')

    # Declare launch arguments to pass to included launch files
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='hexapod_world',
        description='Gazebo world'
    )

    # Include the world launch file
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(share_dir, 'launch', 'world.launch.py')
        ]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    # Include the spawn_hexapod launch file
    spawn_hexapod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(share_dir, 'launch', 'spawn_hexapod.launch.py')
        ])
    )

    # Include the spawn_hexapod launch file
    init_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(share_dir, 'launch', 'control_init.launch.py')
        ])
    )

    # RViz configuration file for visualization
    rviz_config_file = os.path.join(pkg_hexapod_sim, 'rviz', 'default.rviz')

    # RViz node configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        world_arg,
        world_launch,
        spawn_hexapod_launch,
        init_controllers_launch,
        rviz_node,
    ])
