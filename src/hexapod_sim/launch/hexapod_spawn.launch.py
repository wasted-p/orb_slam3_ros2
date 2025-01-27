import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

 # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    ]

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    pkg_hexapod_description = os.path.join(
        get_package_share_directory('hexapod_description'))

    pkg_hexapod_moveit = os.path.join(
        get_package_share_directory('hexapod_moveit'))


    pkg_hexapod_sim = os.path.join(
        get_package_share_directory('hexapod_sim'))

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_hexapod_sim, 'worlds'), ':' +
            str(Path(pkg_hexapod_description).parent.resolve())
            ]
        )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='hexapod_world',
                          description='Gz sim World'),
           ]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hexapod_description"),
            "config",
            "hexapod_controllers.yaml",
        ]
    )

    xacro_file =  os.path.join(pkg_hexapod_description,
                              'robots',
                              'hexapod.urdf.xacro'
                               )

    doc = xacro.process_file(xacro_file,
                             mappings={'use_ros2_control': 'true'}
                             )

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}
    

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'hexapod',
                   '-allow_renaming', 'false',
                   '-topic', 'robot_description',
                   '-allow_renaming', 'true'],
    )


    # # Bridge
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    #                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    #                '/world/hexapod_world/model/hexapod/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    #             ],
    #     output='screen',
    # )

    rviz_config_file = os.path.join(pkg_hexapod_description, 'rviz', 'hexapod.rviz')


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {'use_sim_time': False}],
        output="both",
    )

    bridge_params = os.path.join(
        pkg_hexapod_sim,
        'config',
        'hexapod_bridge.yaml'
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
        # remappings=[
        #     ('/world/hexapod_world/model/hexapod/joint_state', 'joint_states'),
        # ],
    )


# [ros2_control_node-4] [WARN] [1736906350.219839447] [controller_manager]: Waiting for data on 'robot_description' topic to finish initialization
# [gazebo-1] [WARN] [1736906350.746360383] [controller_manager]: No clock received, using time argument instead! Check your node's clock configuration (use_sim_time parameter) 
# and if a valid clock source is available


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )


    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["legs_joint_trajectory_controller",
                   # "--param-file", robot_controllers
                   ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joint_group_position_controller",
                   # "--param-file", robot_controllers
                   ],
    )


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[params],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_controller_manager_after_joint_state_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[control_node],
        )
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        gazebo_resource_path,
        arguments,
        gazebo,
        gz_spawn_entity,
        # bridge,
        robot_state_pub_node,  # Moved up
        control_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        ros_gz_bridge_node,
#
# Node(
#             package='rqt_gui',
#             executable='rqt_gui',
#             name='rqt_joint_trajectory_controller',
#             arguments=['--force-discover'],
#             parameters=[{
#                 'initial_plugin': 'rqt_joint_trajectory_controller/JointTrajectoryController'
#             }],
#             output='screen'
#         )
    ]

    return LaunchDescription(declared_arguments + nodes)
