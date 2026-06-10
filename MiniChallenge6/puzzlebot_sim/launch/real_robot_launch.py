"""
Final Challenge — Launch para el Puzzlebot REAL (version minic3 portada).

Stack basado en el codigo "minic3Bueno" probado por el equipo. Nodos:

  robot_state_publisher  (URDF para RViz)
  mc3_puzzle_transforms  (TF estatica map->odom)
  mc3_joint_states       (encoders -> /joint_states)
  mc3_loc_node           (EKF: predict encoders + correct ArUco)
  mc3_aruco_node          (/marker_publisher/markers -> /aruco_measurement)
  mc3_bug2_node           (bug2 con wall-follow)
  point_generator        (publica /current_goal y /planned_path - opcional)
  covariance_visualizer  (elipse 2D para RViz)
  rviz2

Asume que en el robot ya estan corriendo:
  - aruco_jetson.launch.py  (camera + aruco_ros marker_publisher)
  - Firmware micro-ROS (encoders, /VelocityEncR, /VelocityEncL, /scan, /cmd_vel)
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

    # URDF de minic3Bueno (probado), instalado en este mismo paquete.
    robot_urdf = os.path.join(pkg_sim, 'urdf', 'puzzlebot.urdf')
    default_params = os.path.join(
        pkg_sim, 'config', 'real_robot_params.yaml',
    )
    default_rviz = os.path.join(pkg_sim, 'rviz', 'final_challenge.rviz')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML con parametros (waypoints, pose inicial, etc.)',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
    )
    enable_nav_arg = DeclareLaunchArgument(
        'enable_navigation', default_value='true',
        description='true = autonomous nav (bug2+point_gen). false = solo EKF.',
    )
    # Pose inicial del robot (medida fisicamente).
    x0_arg = DeclareLaunchArgument('x0', default_value='0.23')
    y0_arg = DeclareLaunchArgument('y0', default_value='-0.28')
    theta0_arg = DeclareLaunchArgument('theta0', default_value='0.0')

    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x0 = LaunchConfiguration('x0')
    y0 = LaunchConfiguration('y0')
    theta0 = LaunchConfiguration('theta0')

    # Robot description (URDF) — leido directamente del archivo.
    with open(robot_urdf, 'r') as f:
        robot_description = f.read()
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # TF estatica map->odom (necesaria para RViz Fixed Frame=map).
    transforms = Node(
        package='puzzlebot_sim',
        executable='mc3_puzzle_transforms',
        name='puzzle_transforms',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Joint states (encoders -> /joint_states para RViz robot model).
    joint_states = Node(
        package='puzzlebot_sim',
        executable='mc3_joint_states',
        name='joint_state_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # EKF de localizacion.
    loc = Node(
        package='puzzlebot_sim',
        executable='mc3_loc_node',
        name='loc_node',
        parameters=[
            {'use_sim_time': use_sim_time,
             'x0': x0, 'y0': y0, 'theta0': theta0},
        ],
        output='screen',
    )

    nav_on = IfCondition(LaunchConfiguration('enable_navigation'))

    # ArUco bridge (marker_publisher -> /aruco_measurement).
    aruco = Node(
        package='puzzlebot_sim',
        executable='mc3_aruco_node',
        name='aruco_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Generador de waypoints (publica /current_goal del YAML).
    point_gen = Node(
        package='puzzlebot_sim',
        executable='point_generator',
        name='point_generator',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=nav_on,
    )

    # Bug2 navegacion.
    bug2 = Node(
        package='puzzlebot_sim',
        executable='mc3_bug2_node',
        name='bug2_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=nav_on,
    )

    # Visualizacion de la covarianza de la EKF en RViz (opcional).
    cov_viz = Node(
        package='puzzlebot_sim',
        executable='covariance_visualizer',
        name='covariance_visualizer',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # TF estatico de compatibilidad: el URDF de minic3 ya define 'laser'.
    # Mantenemos un alias por si algun nodo legacy publica al frame antiguo.
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_alias',
        arguments=['0', '0', '0', '0', '0', '0', 'laser', 'laser_frame'],
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
        enable_nav_arg, x0_arg, y0_arg, theta0_arg,
        robot_state_pub,
        transforms,
        joint_states,
        laser_tf,
        loc,
        aruco,
        point_gen,
        bug2,
        cov_viz,
        rviz,
    ])
