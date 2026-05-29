"""
MINI CHALLENGE 5 — Launch para experimentos de covarianza (Task 1 y Task 2).

Arranca:
  - robot_state_publisher (URDF)
  - simulator (publish_tf=false; localisation es dueno del TF)
  - localisation (publica /odom con covarianza y TF odom->base_footprint)
  - RViz2 (opcional, controlado por use_rviz)

NO incluye point_generator ni control. Los experimentos de incertidumbre se
hacen enviando cmd_vel manualmente (ros2 topic pub o teleop).

Uso:
  ros2 launch puzzlebot_sim mc5_launch.py
  ros2 launch puzzlebot_sim mc5_launch.py params_file:=/ruta/a/mi.yaml
  ros2 launch puzzlebot_sim mc5_launch.py use_rviz:=false
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
    urdf_file = os.path.join(pkg, 'urdf', 'puzzlebot.urdf')
    rviz_file = os.path.join(pkg, 'rviz', 'puzzlebot_rviz.rviz')
    default_params = os.path.join(pkg, 'config', 'puzzlebot_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML con parametros de sim/localisation (kr, kl, etc.)'
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

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        params_arg,
        use_rviz_arg,
        use_sim_time_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),

        Node(
            package='puzzlebot_sim',
            executable='simulator',
            name='puzzlebot_sim',
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='localisation',
            name='localisation',
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
