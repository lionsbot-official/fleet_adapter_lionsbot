import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'fleet_adapter'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test', 'test.*', '*.test', '*.test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.xml')) +
            glob(os.path.join('launch', 'docker-compose.yaml'))),
        (os.path.join('share', package_name, 'configs', 'visualization'),
            glob(os.path.join('configs', 'visualization', '*.rviz'))),
    ],
    install_requires=[
        'setuptools',
        'requests',
        'PyYAML',
        'nudged>=0.3',
        'websocket-client',
    ],
    zip_safe=True,
    maintainer='Celes Chai Jia Xuan',
    maintainer_email='celes.chai@gmail.com',
    description='Fleet adapter for Lionsbot robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter.fleet_adapter:main',
            'building_3d_visualizer=fleet_adapter.visualization.building_3d_visualizer:main',
            'robot_marker_visualizer=fleet_adapter.visualization.robot_marker_visualizer:main',
        ],
    },
)
