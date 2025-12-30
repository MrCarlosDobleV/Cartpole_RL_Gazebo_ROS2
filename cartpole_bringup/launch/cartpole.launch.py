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
    pkg_description = get_package_share_directory('cartpole_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo world
    world_path = os.path.join(
        pkg_gazebo,
        'worlds',
        'world.sdf'
    )

    # Load model SDF for robot_state_publisher
    urdf_file = os.path.join(pkg_description, 'urdf', 'cartpole.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_path} -v 4 -r'
        }.items(),
    )

    # robot_state_publisher provides robot_description for ros2_control
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )


    # ROS 2 control spawners
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    cart_vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cart_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
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
        robot_state_publisher,
        joint_state_spawner,
        cart_vel_spawner,
        bridge
    ])
