from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    world_name = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='test_map.sdf',
        description='Gazebo world name'
    )

    # Packages
    pmr_tp1_pkg = FindPackageShare('pmr_tp1')

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
        )
    )

    # Tangent Bug Node
    tangent_bug_node = Node(
        package='pmr_tp1',
        executable='tangent_bug',
        name='tangent_bug',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_world)

    # Add actions
    ld.add_action(sim_launch)
    ld.add_action(rviz_launch)
    ld.add_action(tangent_bug_node)

    return ld
