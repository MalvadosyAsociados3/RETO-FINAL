"""
Final Challenge — All-in-one launch (Gazebo + Puzzlebot + EKF + ArUco +
multi-point navigation + obstacle avoidance + RViz).

Arquitectura (diagrama de la presentacion):

  point_generator                      LiDAR /scan
        |                                    |
        v                                    v
  multi_point_nav  --/pre_cmd_vel-->  obstacle_avoidance  --/cmd_vel--> robot
        ^
        | /odom (EKF)
        |
  ekf_localisation <-- /VelocityEncL,R, /aruco_detections
                            ^
                            |
                       aruco_detector
                            ^
                            |
                       /camera, /camera_info

Uso:
  ros2 launch puzzlebot_sim final_challenge_launch.py
  ros2 launch puzzlebot_sim final_challenge_launch.py world:=puzzlebot_aruco_markers.world
  ros2 launch puzzlebot_sim final_challenge_launch.py use_rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('puzzlebot_sim')
    pkg_gazebo = get_package_share_directory('puzzlebot_gazebo')
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')

    # Use local xacro (standard DiffDrive plugins, works on Fortress/Humble)
    robot_xacro = os.path.join(
        pkg_sim, 'urdf', 'mcr2_robots', 'puzzlebot_jetson_lidar_ed.xacro',
    )
    bridge_config = os.path.join(
        pkg_gazebo, 'config', 'puzzlebot_jetson_lidar_ed.yaml',
    )
    gazebo_models = os.path.join(pkg_gazebo, 'models')
    gazebo_plugins = os.path.join(pkg_gazebo, 'plugins')
    gazebo_media = os.path.join(gazebo_models, 'models', 'media', 'materials')

    default_params = os.path.join(
        pkg_sim, 'config', 'final_challenge_params.yaml',
    )
    default_rviz = os.path.join(pkg_sim, 'rviz', 'final_challenge.rviz')

    # ----------------------------------------------------- Launch args
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='puzzlebot_aruco_markers.world',
        description='Mundo Gazebo dentro de puzzlebot_gazebo/worlds/',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Usar /clock de Gazebo',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Lanzar RViz',
    )
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML con parametros de TODOS los nodos del challenge',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz,
        description='RViz config con markers, ellipsoide y debug image',
    )

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params_file')

    # ----------------------------------------------------- Gz env
    set_gz_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{gazebo_models}:{gazebo_media}',
    )
    set_gz_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=gazebo_plugins,
    )

    # ----------------------------------------------------- Gazebo
    world_path = PathJoinSubstitution([pkg_gazebo, 'worlds', world])
    gz_launch = PathJoinSubstitution([pkg_gz_sim, 'launch', 'gz_sim.launch.py'])
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ----------------------------------------------------- Robot
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
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'puzzlebot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-Y', '0.0',
        ],
        output='screen',
    )

    # ----------------------------------------------------- Bridges
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera'],
    )

    # ----------------------------------------------------- joint -> encoder shim
    joint_to_enc = Node(
        package='puzzlebot_sim',
        executable='joint_to_encoders',
        name='joint_to_encoders',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ----------------------------------------------------- EKF
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

    # ----------------------------------------------------- ArUco detector
    aruco = Node(
        package='puzzlebot_sim',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ----------------------------------------------------- Trayectoria
    point_gen = Node(
        package='puzzlebot_sim',
        executable='point_generator',
        name='point_generator',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ----------------------------------------------------- Multi-point nav
    multi_nav = Node(
        package='puzzlebot_sim',
        executable='multi_point_nav',
        name='multi_point_nav',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ----------------------------------------------------- Obstacle avoidance
    obs_avoid = Node(
        package='puzzlebot_sim',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ----------------------------------------------------- Visualizer
    cov_viz = Node(
        package='puzzlebot_sim',
        executable='covariance_visualizer',
        name='covariance_visualizer',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ----------------------------------------------------- RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        world_arg, use_sim_time_arg, use_rviz_arg, params_arg, rviz_arg,
        set_gz_resources, set_gz_plugins,
        robot_state_pub, start_gazebo, spawn_robot,
        gz_bridge, image_bridge,
        joint_to_enc,
        ekf,
        aruco,
        point_gen,
        multi_nav,
        obs_avoid,
        cov_viz,
        rviz,
    ])
