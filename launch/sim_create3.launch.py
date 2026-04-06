from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # arguments
    world_name = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument('world', default_value='empty.sdf', description='Gazebo world name')
    
    # packages
    tangent_bug_pkg = FindPackageShare('tangent_bug_pmr')
    gazebo_pkg = FindPackageShare('ros_gz_sim')

    # filepaths
    world_path = PathJoinSubstitution([ tangent_bug_pkg, 'worlds', world_name])

    # Start gazebo and spawn robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_pkg, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tangent_bug_pkg, 'launch', 'spawn_create3.launch.py'])
        )
    )

    ld = LaunchDescription()

    # Arguments
    ld.add_action(declare_world)

    # Nodes and launches
    ld.add_action(gazebo)
    ld.add_action(spawn)

    return ld
