import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from pathlib import Path


def generate_launch_description():
    # Set Gazebo simulation resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(get_package_share_directory('hexapod_sim'), 'models') + ':' +
            os.path.join(get_package_share_directory('hexapod_sim'), 'worlds') +
            ':' +
            str(Path(get_package_share_directory(
                'hexapod_description')).parent.resolve())
        ]
    )

    # World argument to specify the Gazebo world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='world',
        description='Gazebo world'
    )

    # Share Dir
    share_dir = get_package_share_directory('hexapod_bringup')
    gui_config_path = os.path.join(share_dir, 'config', 'gazebo_gui.config')

    # Include Gazebo simulation launch description
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(
                'ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[
            ('gz_args', [
                'world.sdf',
                ' -v 4 -r ',
                '--gui-config ', gui_config_path
            ])
        ]
    )

    return LaunchDescription([
        gazebo_resource_path,
        world_arg,
        gazebo,
    ])
