from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for hexapod_description
    share_dir = get_package_share_directory('hexapod_description')
    
    # Path to your URDF file
    urdf_file = os.path.join(share_dir, 'robots', 'hexapod.urdf')
    
    # Read URDF contents
    with open(urdf_file, 'r') as infp:
        robot_urdf = infp.read()

    print("URDF length:", len(robot_urdf))
    # IK Node with robot_description parameter
    leg_control_node = Node(
        package='hexapod_control',           # CHANGE this to your IK package
        executable='leg_control_node',           # CHANGE this to your IK node executable
        name='leg_control',
        output='screen',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        leg_control_node,
        # Your other nodes can be added here as needed
    ])
