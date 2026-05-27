"""
MINI CHALLENGE 6 — Bug 2: Gazebo + reactive navigation (all-in-one).

Launches:
  1. Gazebo (Ignition Fortress) with the selected obstacle avoidance world
  2. Robot spawn (puzzlebot_jetson_lidar_ed via xacro)
  3. ros_gz_bridge (encoders, LiDAR, cmd_vel, clock, camera_info, etc.)
  4. bug2_launch.py (localisation + point_generator + bug2 + RViz)

Usage:
  ros2 launch puzzlebot_sim mc6_bug2_launch.py
  ros2 launch puzzlebot_sim mc6_bug2_launch.py world:=obstacle_avoidance_3.world
  ros2 launch puzzlebot_sim mc6_bug2_launch.py use_rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────────────────
    pkg_sim = get_package_share_directory('puzzlebot_sim')
    pkg_gazebo = get_package_share_directory('puzzlebot_gazebo')
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_desc = get_package_share_directory('puzzlebot_description')

    robot_name = 'puzzlebot_jetson_lidar_ed'
    robot_xacro = os.path.join(pkg_desc, 'urdf', 'mcr2_robots',
                               f'{robot_name}.xacro')
    bridge_config = os.path.join(pkg_gazebo, 'config',
                                 f'{robot_name}.yaml')
    gazebo_models = os.path.join(pkg_gazebo, 'models')
    gazebo_plugins = os.path.join(pkg_gazebo, 'plugins')
    gazebo_media = os.path.join(gazebo_models, 'models', 'media', 'materials')

    # ── Launch arguments ───────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='obstacle_avoidance_1.world',
        description='Gazebo world file inside puzzlebot_gazebo/worlds/'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock from Gazebo'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz (false for headless)'
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_sim, 'config', 'puzzlebot_params.yaml'),
        description='YAML with node parameters'
    )

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Environment for Gazebo model/plugin paths ──────────────────────
    set_gz_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{gazebo_models}:{gazebo_media}'
    )
    set_gz_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=gazebo_plugins
    )

    # ── Gazebo server ──────────────────────────────────────────────────
    world_path = PathJoinSubstitution([pkg_gazebo, 'worlds', world])
    gz_launch = PathJoinSubstitution([pkg_gz_sim, 'launch', 'gz_sim.launch.py'])

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Robot description (xacro → URDF string) ───────────────────────
    robot_description = Command(['xacro ', robot_xacro])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description,
                                                value_type=str),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ── Spawn robot in Gazebo ──────────────────────────────────────────
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

    # ── ros_gz_bridge (encoders, LiDAR, cmd_vel, clock, …) ────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # ── Image bridge (camera) ──────────────────────────────────────────
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera'],
    )

    # ── Bug 2 stack (localisation + point_generator + bug2 + rviz) ─────
    bug2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'bug2_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': LaunchConfiguration('use_rviz'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        use_rviz_arg,
        params_arg,
        set_gz_resources,
        set_gz_plugins,
        robot_state_pub,
        start_gazebo,
        spawn_robot,
        gz_bridge,
        image_bridge,
        bug2_launch,
    ])
