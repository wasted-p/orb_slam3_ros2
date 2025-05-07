import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Define paths to package directories
    pkg_hexapod_description = os.path.join(get_package_share_directory('hexapod_description'))
    pkg_hexapod_sim = os.path.join(get_package_share_directory('hexapod_sim'))

    # Set Gazebo simulation resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_hexapod_sim, 'worlds'),
            ':' + str(Path(pkg_hexapod_description).parent.resolve())
        ]
    )

    # World argument to specify the Gazebo world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='hexapod_world',
        description='Gazebo world'
    )

    # Include Gazebo simulation launch description
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 4', ' -r'])
        ]
    )

    # ROS-Gazebo bridge parameters file
    bridge_params = os.path.join(pkg_hexapod_sim, 'config', 'hexapod_bridge.yaml')

    # ROS-Gazebo bridge node to bridge communication between ROS and Gazebo
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # Return launch description with world setup and bridge
    return LaunchDescription([
        gazebo_resource_path,
        world_arg,
        gazebo,
        ros_gz_bridge_node
    ])
