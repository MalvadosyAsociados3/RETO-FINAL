"""
Final Challenge — Launch para el Puzzlebot REAL.

Esto NO inicia Gazebo. Solo lanza los nodos que viven en la PC:

  robot_state_publisher  (URDF para RViz)
  ekf_localisation       (predict encoders + correct ArUco)
  multi_point_nav        (go-to-goal -> /pre_cmd_vel)
  obstacle_avoidance     (/pre_cmd_vel -> /cmd_vel, reactivo con LiDAR)
  point_generator        (publica /current_goal y /planned_path)
  aruco_ros_bridge       (aruco_msgs/MarkerArray -> ArucoDetectionArray)
  covariance_visualizer  (elipse 2D para RViz)
  rviz2

Asume que en el robot ya estan corriendo:
  - aruco_jetson.launch.py        (ros_deep_learning + camera_info_publisher +
                                   aruco_ros marker_publisher)
  - Firmware de encoders, LiDAR y suscriptor de /cmd_vel
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('puzzlebot_sim')
    pkg_desc = get_package_share_directory('puzzlebot_description')

    robot_name = 'puzzlebot_jetson_lidar_ed'
    robot_xacro = os.path.join(
        pkg_desc, 'urdf', 'mcr2_robots', f'{robot_name}.xacro',
    )
    default_params = os.path.join(
        pkg_sim, 'config', 'real_robot_params.yaml',
    )
    default_rviz = os.path.join(pkg_sim, 'rviz', 'final_challenge.rviz')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML con parametros para la corrida en el robot real',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Lanzar RViz en la PC',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Falso en robot real (usamos el reloj del sistema)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
    )

    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot state publisher (URDF para RViz; el firmware del Jetson tambien
    # puede publicarlo, en cuyo caso podrias omitir este nodo).
    robot_description = Command(['xacro ', robot_xacro])
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    ekf = Node(
        package='puzzlebot_sim',
        executable='ekf_localisation',
        name='ekf_localisation',
        parameters=[params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('wr', '/VelocityEncR'),
            ('wl', '/VelocityEncL'),
        ],
        output='screen',
    )

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

    aruco_bridge = Node(
        package='puzzlebot_sim',
        executable='aruco_ros_bridge',
        name='aruco_ros_bridge',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    cov_viz = Node(
        package='puzzlebot_sim',
        executable='covariance_visualizer',
        name='covariance_visualizer',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        params_arg, use_rviz_arg, use_sim_time_arg, rviz_arg,
        robot_state_pub,
        ekf,
        point_gen,
        multi_nav,
        obs_avoid,
        aruco_bridge,
        cov_viz,
        rviz,
    ])
