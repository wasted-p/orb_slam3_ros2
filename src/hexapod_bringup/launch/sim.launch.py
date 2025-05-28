from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define paths to package directories
    share_dir = os.path.join(get_package_share_directory('hexapod_bringup'))

    # Include spawn_world.launch.py
    spawn_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, 'launch', 'spawn_world.launch.py')
        )
    )

    # Include spawn_hexapod.launch.py
    spawn_hexapod = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, 'launch', 'spawn_hexapod.launch.py')
        )
    )

    # Return launch description with both included launch files
    return LaunchDescription([
        spawn_world,
        spawn_hexapod
    ])
