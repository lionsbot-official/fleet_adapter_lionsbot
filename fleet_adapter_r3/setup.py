import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'fleet_adapter_r3'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'fastapi>=0.79.0', 'uvicorn>=0.18.2'],
    zip_safe=True,
    maintainer='Dani Purwadi',
    maintainer_email='dani.purwadi@lionsbot.com',
    description='Fleet adapter for R3 robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter_r3.fleet_adapter:main'
        ],
    },
)
