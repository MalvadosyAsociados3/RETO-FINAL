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
            'ekf_localisation = puzzlebot_sim.ekf_localisation:main',
            'aruco_detector = puzzlebot_sim.aruco_detector:main',
            'aruco_ros_bridge = puzzlebot_sim.aruco_ros_bridge:main',
            'multi_point_nav = puzzlebot_sim.multi_point_nav:main',
            'obstacle_avoidance = puzzlebot_sim.obstacle_avoidance:main',
            'point_generator = puzzlebot_sim.point_generator:main',
            'joint_to_encoders = puzzlebot_sim.joint_to_encoders:main',
            'bug0 = puzzlebot_sim.bug0:main',
            'covariance_visualizer = puzzlebot_sim.covariance_visualizer:main',
        ],
    },
)
