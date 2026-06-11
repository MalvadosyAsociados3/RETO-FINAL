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
from launch.conditions import IfCondition, UnlessCondition
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
    default_map = os.path.join(pkg_sim, 'maps', 'map_maze_real.yaml')

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
    map_arg = DeclareLaunchArgument(
        'map_yaml', default_value=default_map,
        description='Mapa OccupancyGrid (.yaml) que publica /map para RViz',
    )
    enable_nav_arg = DeclareLaunchArgument(
        'enable_navigation', default_value='true',
        description='true = autonomous nav (point_gen+multi_point+obstacle_avoid). '
                    'false = solo EKF + ArUco bridge (para demo de teleop / Escena A).',
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

    # map_server: publica el mapa OccupancyGrid del laberinto en /map para
    # que RViz pueda mostrarlo y para que el profesor use "2D Pose Estimate"
    # y "2D Goal Pose" haciendo click sobre el mapa.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': LaunchConfiguration('map_yaml'),
        }],
        output='screen',
    )
    # nav2_map_server es un lifecycle node -> hay que activarlo. El
    # lifecycle_manager con autostart=True lo configura y lo activa solo.
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
        output='screen',
    )

    # joint_state_publisher: publica /joint_states con posiciones default
    # (cero) si el firmware del Jetson no lo hace. Sin esto las TFs de las
    # llantas no se resuelven y el RobotModel aparece "bugeado" en RViz
    # (llanta separada del chasis, etc.). Si el firmware ya publica
    # /joint_states, esto se ignora (gana el ultimo publisher).
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'rate': 30}],
        output='screen',
    )

    ekf = Node(
        package='puzzlebot_sim',
        executable='ekf_localisation',
        name='ekf_localisation',
        parameters=[params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('wr', '/VelocityEncL'),   # SWAP: firmware publica R/L invertidos
            ('wl', '/VelocityEncR'),
        ],
        output='screen',
    )

    nav_on = IfCondition(LaunchConfiguration('enable_navigation'))

    point_gen = Node(
        package='puzzlebot_sim',
        executable='point_generator',
        name='point_generator',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=nav_on,
    )

    bug0 = Node(
        package='puzzlebot_sim',
        executable='bug0',
        name='bug0',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=nav_on,
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

    # TF estatico: el URDF define 'laser_frame' pero rplidar publica en 'laser'
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'laser'],
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
        params_arg, use_rviz_arg, use_sim_time_arg, rviz_arg, map_arg, enable_nav_arg,
        map_server,
        map_lifecycle,
        robot_state_pub,
        joint_state_pub,
        laser_tf,
        ekf,
        point_gen,
        bug0,
        aruco_bridge,
        cov_viz,
        rviz,
    ])
