from setuptools import setup

package_name = 'fleet_adapter_lionsbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}/models'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name,['config.yaml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lum Kai Wen',
    maintainer_email='lum_kai_wen@artc.a-star.edu.sg',
    description='A template for an RMF fleet adapter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter_lionsbot.fleet_adapter:main'
        ],
    },
)
