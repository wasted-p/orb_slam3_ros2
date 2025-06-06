import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler


def generate_launch_description():

    ####################################
    # Params
    ####################################
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # World argument to specify the Gazebo world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='world',
        description='Gazebo world'
    )

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

    ####################################
    # Paths
    ####################################
    share_dir = get_package_share_directory('hexapod_bringup')
    gui_config_path = os.path.join(share_dir, 'config', 'gazebo_gui.config')
    pkg_hexapod_sim = get_package_share_directory('hexapod_sim')
    bridge_params = os.path.join(
        pkg_hexapod_sim, 'config', 'hexapod_bridge.yaml')
    pkg_hexapod_description = get_package_share_directory(
        'hexapod_description')

    ####################################
    # Variables
    ####################################
    urdf_file = os.path.join(pkg_hexapod_description, 'robots', 'hexapod.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    ####################################
    # Nodes
    ####################################
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[{'publish_frequency': 50.0,
                     'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

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

    # ROS-Gazebo bridge node
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        # output='screen'
    )

    # hexapod_gait_planner_node = Node(
    #     package='hexapod_gait',
    #     executable='gait_planner_node',
    #     name='gait_planner_node',
    #     output="screen",
    #     parameters=[initial_pose_path],
    # )

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
        # output='screen',
    )

    startup_controllers_node = Node(
        package='hexapod_control',
        executable='startup_controllers_node',
        name='startup_controllers',
        output='screen'
    )

    hexapod_trajectory_node = Node(
        package='hexapod_control',
        executable='hexapod_trajectory_node',
        name='hexapod_trajectory_node',
        output='screen'
    )

    # Launch trajectory node after startup controllers exit
    launch_trajectory_on_startup_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=startup_controllers_node,
            on_exit=[hexapod_trajectory_node]
        )
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
        gazebo_resource_path,
        world_arg,
        gazebo,
        robot_state_pub_node,
        ros_gz_bridge_node,
        # hexapod_gait_planner_node ,
        point_cloud_node,
        # startup_controllers_node,
        # launch_trajectory_on_startup_exit,
        rviz_node
    ])
