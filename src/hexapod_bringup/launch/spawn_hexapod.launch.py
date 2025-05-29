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

    # Share Dir
    share_dir = get_package_share_directory('hexapod_bringup')
    pkg_hexapod_sim = get_package_share_directory('hexapod_sim')
    pkg_hexapod_description = get_package_share_directory(
        'hexapod_description')
    # ROS-Gazebo bridge parameters file
    bridge_params = os.path.join(
        pkg_hexapod_sim, 'config', 'hexapod_bridge.yaml')
    robot_controllers = os.path.join(
        pkg_hexapod_sim, 'config', 'hexapod_controllers.yaml')

    # Process the URDF file
    urdf_file = os.path.join(pkg_hexapod_description, 'robots', 'hexapod.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    initial_pose_path = os.path.join(share_dir, 'config', 'initial_pose.yml')

    # ROS-Gazebo bridge node
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # Control node (ros2_control) for managing controllers
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {'use_sim_time': use_sim_time}],
        output="both",
    )

    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[{'publish_frequency': 50.0,
                     'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # Hexapod IK node
    hexapod_ik_gz_node = Node(
        package='hexapod_control',
        executable='hexapod_ik_gz_node',
        name='hexapod_ik_gz_node',
        output="screen",
        parameters=[{'robot_description': robot_desc}],
    )

    # Hexapod gait planner node
    hexapod_gait_planner_node = Node(
        package='hexapod_gait',
        executable='gait_planner_node',
        name='gait_planner_node',
        output="screen",
        parameters=[initial_pose_path],
    )

    # Point cloud node
    point_cloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz',
        remappings=[
            ('image_rect', '/depth_camera/depth/image_raw'),
            ('camera_info', '/depth_camera/depth/camera_info'),
            ('points', '/depth_camera/points')
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'queue_size': 10,
            'approximate_sync': True,
        }],
        output='screen',
    )

    # Event handlers to delay node start orders
    delay_control_node_after_robot_state_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[control_node]
        )
    )

    return LaunchDescription([
        robot_state_pub_node,
        point_cloud_node,
        delay_control_node_after_robot_state_publisher,
        ros_gz_bridge_node,
        hexapod_ik_gz_node,
        hexapod_gait_planner_node,
    ])
