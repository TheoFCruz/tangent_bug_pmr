from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    pmr_tp1_pkg = FindPackageShare('pmr_tp1')
    gazebo_pkg = FindPackageShare('ros_gz_sim')

    # Arguments
    world_name = LaunchConfiguration('world')
    map_path = LaunchConfiguration('map_path')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world name'
    )
    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value=PathJoinSubstitution([pmr_tp1_pkg, 'maps', 'empty_map.yaml']),
        description='Path to the map YAML file'
    )
    declare_rviz_config_path = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=PathJoinSubstitution([pmr_tp1_pkg, 'rviz', 'path_with_potential.rviz']),
        description='Path to the RViz configuration file'
    )

    # Filepaths
    world_path = PathJoinSubstitution([pmr_tp1_pkg, 'worlds', world_name])
    bridge_file = PathJoinSubstitution([pmr_tp1_pkg, 'config', 'bridge.yaml'])
    spawn_launch_file = PathJoinSubstitution(
        [pmr_tp1_pkg, 'launch', 'include', 'spawn_create3.launch.py']
    )
    rviz_map_launch_file = PathJoinSubstitution(
        [pmr_tp1_pkg, 'launch', 'include', 'rviz_map.launch.py']
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_pkg, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # Global ros-gz bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_file,
            'use_sim_time': True
        }],
        output='screen'
    )

    rviz_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_map_launch_file),
        launch_arguments={
            'map_path': map_path,
            'rviz_config_path': rviz_config_path
        }.items()
    )

    robot_configs = [
        {'id': 1, 'name': 'robot_1', 'x': '1.0',  'y': '1.0'},
        {'id': 2, 'name': 'robot_2', 'x': '-1.0', 'y': '1.0'},
        {'id': 3, 'name': 'robot_3', 'x': '-1.0', 'y': '-1.0'},
        {'id': 4, 'name': 'robot_4', 'x': '1.0',  'y': '-1.0'},
    ]

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_map_path)
    ld.add_action(declare_rviz_config_path)
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ld.add_action(rviz_map)

    for config in robot_configs:
        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch_file)
        )

        path_with_potential_node = Node(
            package='pmr_tp1',
            executable='path_with_potential',
            name='path_with_potential',
            namespace=config['name'],
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_id': config['id'],
                'num_robots': len(robot_configs)
            }]
        )

        # Scope repeated spawn arguments so each robot keeps its own namespace and pose.
        robot_group = GroupAction(
            scoped=True,
            forwarding=False,
            launch_configurations={
                'namespace': config['name'],
                'robot_name': config['name'],
                'x': config['x'],
                'y': config['y'],
                'z': '0.1',
                'yaw': '0.0'
            },
            actions=[
                spawn_robot,
                path_with_potential_node
            ]
        )

        ld.add_action(robot_group)

    return ld
