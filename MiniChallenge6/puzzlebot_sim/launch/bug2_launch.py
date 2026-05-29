"""
MINI CHALLENGE 6 - Launch para Bug 2.

ASUME que Gazebo (puzzlebot_gazebo de robotec_sim_ws) ya esta corriendo:

    ros2 launch puzzlebot_gazebo gazebo_example_launch.py

Identico a bug0_launch.py pero arranca el nodo `bug2` en lugar de `bug0`.

Topics y remapeos: ver docstring de bug0_launch.py.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_sim')
    rviz_file = os.path.join(pkg, 'rviz', 'puzzlebot_rviz.rviz')
    default_params = os.path.join(pkg, 'config', 'puzzlebot_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML con parametros (localisation, point_generator, bug2)'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Lanzar RViz (false para tests headless)'
    )
    params = LaunchConfiguration('params_file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock from Gazebo'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        params_arg,
        use_rviz_arg,
        use_sim_time_arg,

        # /joint_states -> /VelocityEncL,R (Float32) para localisation.
        Node(
            package='puzzlebot_sim',
            executable='joint_to_encoders',
            name='joint_to_encoders',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='localisation',
            name='localisation',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('wr', '/VelocityEncR'),
                ('wl', '/VelocityEncL'),
            ],
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='point_generator',
            name='point_generator',
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='bug2',
            name='bug2',
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
