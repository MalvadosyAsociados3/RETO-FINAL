"""
PARTE 2 - Launch con SIMULACION + LOCALISATION.

Arranca:
  - simulator (publish_tf=false; ahora el TF lo publica localisation)
  - localisation (publica /odom y TF odom->base_footprint)
  - robot_state_publisher (URDF con STL del puzzlebot)
  - RViz2


"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_sim')
    urdf_file = os.path.join(pkg, 'urdf', 'puzzlebot.urdf')
    rviz_file = os.path.join(pkg, 'rviz', 'puzzlebot_rviz.rviz')
    params_file = os.path.join(pkg, 'config', 'puzzlebot_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock from Gazebo'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
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
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='localisation',
            name='localisation',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
