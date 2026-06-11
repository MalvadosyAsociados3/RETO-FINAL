"""
Test Navegacion — point_generator + multi_point_nav + obstacle_avoidance.

NO lanza Gazebo ni EKF. Espera /odom y /scan de una fuente externa
(robot real, bag file, u otro launch).

Uso:
  # Terminal 1: fuente de datos (e.g., bag o test_ekf_launch)
  ros2 launch puzzlebot_sim test_ekf_launch.py

  # Terminal 2: navegacion
  ros2 launch puzzlebot_sim test_nav_launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('puzzlebot_sim')
    default_params = os.path.join(
        pkg_sim, 'config', 'final_challenge_params.yaml',
    )
    default_rviz = os.path.join(pkg_sim, 'rviz', 'final_challenge.rviz')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true si los datos vienen de Gazebo/bag',
    )

    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    point_gen = Node(
        package='puzzlebot_sim',
        executable='point_generator',
        name='point_generator',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    multi_nav = Node(
        package='puzzlebot_sim',
        executable='multi_point_nav',
        name='multi_point_nav',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    obs_avoid = Node(
        package='puzzlebot_sim',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        params_arg, use_rviz_arg, use_sim_time_arg,
        point_gen,
        multi_nav,
        obs_avoid,
        rviz,
    ])
