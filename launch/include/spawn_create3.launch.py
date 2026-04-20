from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # arguments
    x_spawn = LaunchConfiguration('x')
    y_spawn = LaunchConfiguration('y')
    z_spawn = LaunchConfiguration('z')
    yaw_spawn = LaunchConfiguration('yaw')

    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0', description='Spawn x')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0', description='Spawn y')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.1', description='Spawn z')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw')

    # packages
    pmr_tp1_pkg = FindPackageShare('pmr_tp1')

    # filepaths
    xacro_file = PathJoinSubstitution([pmr_tp1_pkg, 'description/create3/create3.urdf.xacro'])
    bridge_file = PathJoinSubstitution([pmr_tp1_pkg, 'config/bridge.yaml'])
    control_launch_file = PathJoinSubstitution([pmr_tp1_pkg, 'launch/include/control_create3.launch.py'])

    # robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file, ' gazebo:=ignition']),
            'use_sim_time': True
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
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

    # spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'create3',
            '-topic', 'robot_description',
            '-x', x_spawn, 
            '-y', y_spawn,
            '-z', z_spawn, 
            '-Y', yaw_spawn
        ],
        output='screen'
    )

    # controller setup
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_file),
    )

    # stamper to correctly expose cmd_vel
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in','/cmd_vel'),
                    ('/cmd_vel_out','/diffdrive_controller/cmd_vel')]
    )


    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)

    # Add nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(controller)
    ld.add_action(twist_stamper)

    return ld

