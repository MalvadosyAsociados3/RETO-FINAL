"""
MINI CHALLENGE 6 - Launch para Bug 0.

ASUME que Gazebo (puzzlebot_gazebo de robotec_sim_ws) ya esta corriendo
en otra terminal:

    ros2 launch puzzlebot_gazebo gazebo_example_launch.py

Este launch arranca SOLO nuestros nodos:
  - localisation     (consume /VelocityEncL/R, publica /odom con covarianza, TF)
  - point_generator  (publica /current_goal con la trayectoria)
  - bug0             (la estrategia reactiva)
  - rviz2            (opcional con use_rviz:=false)

Topics de Gazebo (publicados por robotec_sim_ws):
  /cmd_vel        (Twist)         <- subscribe Gazebo
  /scan           (LaserScan)     <- publica  Gazebo
  /VelocityEncL   (Float32)       <- publica  Gazebo (encoder izq)
  /VelocityEncR   (Float32)       <- publica  Gazebo (encoder der)
  /odom           (Odometry)      <- publica  Gazebo (lo IGNORAMOS; usamos el de localisation)
  /ground_truth   (Odometry)      <- publica  Gazebo (referencia opcional)

Remapeos:
  localisation: wr <- /VelocityEncR, wl <- /VelocityEncL
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
        description='YAML con parametros (localisation, point_generator, bug0)'
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

        # Dead reckoning + covarianza
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

        # Genera secuencia de waypoints
        Node(
            package='puzzlebot_sim',
            executable='point_generator',
            name='point_generator',
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Bug 0 reactivo
        Node(
            package='puzzlebot_sim',
            executable='bug0',
            name='bug0',
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # RViz (opcional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
