import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Package paths
    pkg_bringup = get_package_share_directory('cartpole_bringup')
    pkg_gazebo = get_package_share_directory('cartpole_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo world
    world_path = os.path.join(
        pkg_gazebo,
        'worlds',
        'world.sdf'
    )

    # Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_path} -v 4 -r'
        }.items(),
    )

    # ROS <-> Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                pkg_bringup,
                'config',
                'cartpole_bridge.yaml'
            )
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge
    ])
