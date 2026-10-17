import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('drone_pkg')
    model_file = os.path.join(pkg, 'models', 'quadrotor', 'model.sdf')

    # 1. Gazebo (physics server + GUI) with an empty world.
    #    The world must load gazebo_ros_init/state so ROS can talk to it;
    #    gazebo.launch.py already does that.
    world_file = os.path.join(pkg, 'worlds', 'drone.world')
    gui = LaunchConfiguration('gui')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'false',
                          'gui': gui}.items()
    )

    # 2. Spawn the quadrotor SDF into the running simulation.
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'quadrotor', '-file', model_file,
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # 3. Our altitude-hold controller.
    controller = Node(
        package='drone_pkg',
        executable='drone_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = headless (no Gazebo window)'),
        gazebo, spawn, controller,
    ])
