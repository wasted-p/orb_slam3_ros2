import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from pathlib import Path
from launch_ros.actions import Node
# from launch.event_handlers import OnProcessExit
# from launch.actions import RegisterEventHandler


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

    hexapod_initial_pose_path = os.path.join(
        share_dir, 'config', 'initial_pose.yml')
    arm_initial_pose_path = os.path.join(
        share_dir, 'config', 'arm_initial_pose.yml')

    hexapod_initial_positions_path = os.path.join(
        share_dir, 'config', 'initial_positions.yml')
    arm_initial_positions_path = os.path.join(
        share_dir, 'config', 'arm_initial_positions.yml')

    hexapod_urdf_path = os.path.join(get_package_share_directory(
        'hexapod_description'), 'robots', 'hexapod.urdf')
    arm_urdf_path = os.path.join(get_package_share_directory(
        'hexapod_description'), 'robots', 'arm.urdf')

    rviz_config_path = os.path.join(pkg_hexapod_sim, 'rviz', 'default.rviz')

    ####################################
    # Files
    ####################################
    with open(hexapod_urdf_path, 'r') as f:
        hexapod_urdf = f.read()

    with open(arm_urdf_path, 'r') as f:
        arm_urdf = f.read()

    with open(hexapod_initial_pose_path, 'r') as f:
        hexapod_initial_pose = f.read()

    with open(arm_initial_pose_path, 'r') as f:
        arm_initial_pose = f.read()

    with open(hexapod_initial_positions_path, 'r') as f:
        hexapod_initial_positions = f.read()

    with open(arm_initial_positions_path, 'r') as f:
        arm_initial_positions = f.read()

    ####################################
    # Nodes
    ####################################
    hexapod_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[{'publish_frequency': 50.0,
                     'robot_description': hexapod_urdf,
                     'use_sim_time': use_sim_time}]
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

    hexapod_kinematics_service = Node(
        package='hexapod_control',
        executable='kinematics_service',
        name='kinematics_service',
        output="screen",
        parameters=[{'robot_description': hexapod_urdf},
                    {'prefix': 'hexapod'}],
    )

    arm_kinematics_service = Node(
        package='hexapod_control',
        executable='kinematics_service',
        name='kinematics_service',
        output="screen",
        parameters=[{'robot_description': arm_urdf},
                    {'prefix': 'arm'}],
    )

    hexapod_pose_publisher = Node(
        package='hexapod_control',
        executable='pose_publisher',
        name='pose_publisher',
        output="screen",
        parameters=[{'initial_pose': hexapod_initial_pose},
                    {'prefix': 'hexapod'}],
    )

    arm_pose_publisher = Node(
        package='hexapod_control',
        executable='pose_publisher',
        name='pose_publisher',
        output="screen",
        parameters=[{'initial_pose': arm_initial_pose},
                    {'prefix': 'arm'}],
    )

    # Nodes
    hexapod_joint_state_publisher = Node(
        package='hexapod_control',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output="screen",
        parameters=[{'initial_positions': hexapod_initial_positions},
                    {'prefix': 'hexapod'}]
    )

    arm_joint_state_publisher = Node(
        package='hexapod_control',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output="screen",
        parameters=[{'initial_positions': arm_initial_positions},
                    {'prefix': 'arm'}]
    )

    motion_server = Node(
        package='hexapod_motion',
        executable='motion_server',
        name='motion_server',
        output="screen",
        parameters=[{'initial_pose': hexapod_initial_pose},
                    {'prefix': 'hexapod'}],
    )

    visualization_server = Node(
        package='hexapod_control',
        executable='visualization_server',
        name='visualization_server',
        output="screen",
    )

    hexapod_teleop = Node(
        package='hexapod_teleop',
        executable='teleop_joy',
        name='teleop_joy',
        output="screen",
        parameters=[{'prefix': 'hexapod'}],
    )

    # RViz node configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # joint_state_publisher = Node(
    #     package='hexapod_control',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output="screen",
    #     parameters=[{'initial_positions': initial_positions}],
    # )

    return LaunchDescription([
        gazebo_resource_path,
        world_arg,
        gazebo,
        hexapod_robot_state_publisher,
        ros_gz_bridge_node,
        point_cloud_node,
        rviz_node,
        joy_node,
        visualization_server,

        # Hexapod
        hexapod_kinematics_service,
        hexapod_pose_publisher,
        hexapod_joint_state_publisher,
        hexapod_teleop,
        motion_server,

        # Arm
        # arm_joint_state_publisher
        # startup_controllers
        # joint_state_publisher
    ])
