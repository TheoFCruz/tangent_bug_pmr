from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Packages
    pmr_tp1_pkg = FindPackageShare('pmr_tp1')

    # Filepaths
    default_map_path = PathJoinSubstitution([pmr_tp1_pkg, 'maps', 'test_map.yaml'])
    default_rviz_config_path = PathJoinSubstitution([pmr_tp1_pkg, 'rviz', 'bug.rviz'])

    # Arguments
    world_name = LaunchConfiguration('world')
    map_path = LaunchConfiguration('map_path')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='test_map.sdf',
        description='Gazebo world name'
    )
    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Path to the map YAML file'
    )
    declare_rviz_config_path = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=default_rviz_config_path,
        description='Path to the RViz configuration file'
    )

    # Includes
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pmr_tp1_pkg, 'launch', 'include', 'sim_create3.launch.py'])
        ),
        launch_arguments={'world': world_name}.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pmr_tp1_pkg, 'launch', 'include', 'rviz_map.launch.py'])
        ),
        launch_arguments={
            'map_path': map_path,
            'rviz_config_path': rviz_config_path
        }.items()
    )

    # Tangent Bug Node
    potential_function_node = Node(
        package='pmr_tp1',
        executable='potential_function',
        name='potential_function',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_world)
    ld.add_action(declare_map_path)
    ld.add_action(declare_rviz_config_path)

    # Add actions
    ld.add_action(sim_launch)
    ld.add_action(rviz_launch)
    ld.add_action(potential_function_node)

    return ld

