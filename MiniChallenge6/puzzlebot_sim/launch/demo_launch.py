"""
Launch UNIFICADO
Arranca:
  - robot_state_publisher (Parte 2: URDF + STL)
  - simulator             (Parte 1: modelo cinematico diferencial)
  - localisation          (Parte 2: integra wr/wl -> nav_msgs/Odometry + TF)
  - control               (Parte 3: setpoints -> /cmd_vel)
  - rviz2                 (visualizacion)
  - rqt_plot              (grafica pose y velocidades de ruedas)
  - rqt_graph             (grafo de nodos/topicos)



"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_sim')
    urdf_file = os.path.join(pkg, 'urdf', 'puzzlebot.urdf')
    rviz_file = os.path.join(pkg, 'rviz', 'puzzlebot_rviz.rviz')
    default_params = os.path.join(pkg, 'config', 'puzzlebot_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML con parametros (square/triangle/pentagon).'
    )
    plots_arg = DeclareLaunchArgument(
        'plots',
        default_value='true',
        description='Abrir rqt_plot y rqt_graph (true/false).'
    )

    params = LaunchConfiguration('params_file')
    plots = LaunchConfiguration('plots')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock from Gazebo'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    simulator = Node(
        package='puzzlebot_sim',
        executable='simulator',
        name='puzzlebot_sim',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    localisation = Node(
        package='puzzlebot_sim',
        executable='localisation',
        name='localisation',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    point_generator = Node(
        package='puzzlebot_sim',
        executable='point_generator',
        name='point_generator',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    control = Node(
        package='puzzlebot_sim',
        executable='control',
        name='control',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Esperamos ~3 s para que /odom, /wr, /wl existan antes de abrir rqt_plot;
    # si no, rqt_plot se queja y hay que recargar los topicos a mano.
    rqt_plot = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                condition=IfCondition(plots),
                cmd=[
                    'ros2', 'run', 'rqt_plot', 'rqt_plot',
                    '/odom/pose/pose/position/x',
                    '/odom/pose/pose/position/y',
                    '/wr/data',
                    '/wl/data',
                ],
                output='screen',
            ),
        ],
    )

    rqt_graph = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                condition=IfCondition(plots),
                cmd=['ros2', 'run', 'rqt_graph', 'rqt_graph'],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        params_arg,
        plots_arg,
        use_sim_time_arg,
        robot_state_pub,
        simulator,
        localisation,
        point_generator,
        control,
        rviz,
        rqt_plot,
        rqt_graph,
    ])
