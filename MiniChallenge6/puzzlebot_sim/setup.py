from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario Martinez',
    maintainer_email='mario.mtz@manchester-robotics.com',
    description='Puzzlebot Kinematic Sim, Localisation and Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = puzzlebot_sim.simulator:main',
            'localisation = puzzlebot_sim.localisation:main',
            'control = puzzlebot_sim.control:main',
            'point_generator = puzzlebot_sim.point_generator:main',
            'joint_state_publisher = puzzlebot_sim.joint_state_publisher:main',
            'experiment_runner = puzzlebot_sim.experiment_runner:main',
            'bug0 = puzzlebot_sim.bug0:main',
            'bug2 = puzzlebot_sim.bug2:main',
            'joint_to_encoders = puzzlebot_sim.joint_to_encoders:main',
            'ekf_localisation = puzzlebot_sim.ekf_localisation:main',
            'aruco_detector = puzzlebot_sim.aruco_detector:main',
            'multi_point_nav = puzzlebot_sim.multi_point_nav:main',
            'obstacle_avoidance = puzzlebot_sim.obstacle_avoidance:main',
            'covariance_visualizer = puzzlebot_sim.covariance_visualizer:main',
            'aruco_ros_bridge = puzzlebot_sim.aruco_ros_bridge:main',
            # Nodos del codigo "minic3Bueno" (probado por el equipo).
            'mc3_loc_node = puzzlebot_sim.mc3_loc_node:main',
            'mc3_bug2_node = puzzlebot_sim.mc3_bug2_node:main',
            'mc3_aruco_node = puzzlebot_sim.mc3_aruco_node:main',
            'mc3_joint_states = puzzlebot_sim.mc3_joint_states:main',
            'mc3_puzzle_transforms = puzzlebot_sim.mc3_puzzle_transforms:main',
        ],
    },
)
