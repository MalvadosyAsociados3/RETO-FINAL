"""
Test ArUco — Solo deteccion de markers, sin EKF ni navegacion.

Lanza Gazebo + robot + camara + aruco_detector + RViz.
Sirve para verificar calibracion de camara y deteccion de markers.

Uso:
  ros2 launch puzzlebot_sim test_aruco_launch.py
  # Ver imagen con detecciones:
  ros2 run rqt_image_view rqt_image_view /aruco_image
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
    pkg_desc = get_package_share_directory('puzzlebot_description')

    robot_name = 'puzzlebot_jetson_lidar_ed'
    robot_xacro = os.path.join(
        pkg_desc, 'urdf', 'mcr2_robots', f'{robot_name}.xacro',
    )
    bridge_config = os.path.join(
        pkg_gazebo, 'config', f'{robot_name}.yaml',
    )
    gazebo_models = os.path.join(pkg_gazebo, 'models')
    gazebo_plugins = os.path.join(pkg_gazebo, 'plugins')
    gazebo_media = os.path.join(gazebo_models, 'models', 'media', 'materials')

    default_params = os.path.join(
        pkg_sim, 'config', 'final_challenge_params.yaml',
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='puzzlebot_aruco_markers.world',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
    )
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
    )

    use_sim_time = 'true'
    params = LaunchConfiguration('params_file')

    set_gz_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{gazebo_models}:{gazebo_media}',
    )
    set_gz_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=gazebo_plugins,
    )

    world_path = PathJoinSubstitution([pkg_gazebo, 'worlds', LaunchConfiguration('world')])
    gz_launch = PathJoinSubstitution([pkg_gz_sim, 'launch', 'gz_sim.launch.py'])
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

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

    aruco = Node(
        package='puzzlebot_sim',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_sim, 'rviz', 'final_challenge.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        world_arg, use_rviz_arg, params_arg,
        set_gz_resources, set_gz_plugins,
        robot_state_pub, start_gazebo, spawn_robot,
        gz_bridge, image_bridge,
        aruco,
        rviz,
    ])
