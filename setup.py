from glob import glob

from setuptools import setup

package_name = 'tidy_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='VLM-driven tidying demo for CRANE+ V2 in Ignition Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tidy_node = tidy_robot.tidy_node:main',
            'world_gen = tidy_robot.world_gen:main',
        ],
    },
)
