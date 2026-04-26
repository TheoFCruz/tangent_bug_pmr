from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Packages
    pmr_tp1_pkg = FindPackageShare('pmr_tp1')

    # Filepaths
    rviz_config_path = PathJoinSubstitution([pmr_tp1_pkg, 'rviz', 'curve.rviz'])

    # Arguments
    world_name = LaunchConfiguration('world')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world name'
    )

    # Includes
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pmr_tp1_pkg, 'launch', 'include', 'sim_create3.launch.py'])
        ),
        launch_arguments={'world': world_name}.items()
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # Parametric Curve Node
    parametric_curve_node = Node(
        package='pmr_tp1',
        executable='parametric_curve',
        name='parametric_curve',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_world)

    # Add actions
    ld.add_action(sim_launch)
    ld.add_action(rviz)
    ld.add_action(parametric_curve_node)

    return ld
