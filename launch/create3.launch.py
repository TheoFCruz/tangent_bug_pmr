import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package definition
    tangent_bug_pkg = FindPackageShare('tangent_bug_pmr')
    gazebo_pkg = FindPackageShare('ros_gz_sim')
    control_pkg = FindPackageShare('irobot_create_control')

    # Filepaths
    xacro_file = PathJoinSubstitution([tangent_bug_pkg, 'description/create3/create3.urdf.xacro'])
    bridge_file = PathJoinSubstitution([tangent_bug_pkg, 'config/bridge.yaml'])
    control_params_file = PathJoinSubstitution([control_pkg, 'config/control.yaml'])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file, ' gazebo:=ignition']),
            'use_sim_time': True
        }]
    )

    # Start gazebo and spawn robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_pkg, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'create3',
            '-topic', 'robot_description',
            '-z', '0.1' 
        ],
        output='screen'
    )

    # ros-gz bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_file,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Joint state broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen',
    )

    # Diff drive controller
    load_diffdrive_controller = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[control_params_file],
        arguments=['diffdrive_controller', '--controller-manager-timeout', '30'],
        output='screen',
    )

    # Make sure controllers start after robot
    spawn_broadcaster_after_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    spawn_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_diffdrive_controller],
        )
    )

    ld = LaunchDescription()

    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(spawn_broadcaster_after_robot)
    ld.add_action(spawn_controller_after_broadcaster)

    return ld
