from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ros2_python_test2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/ros2_python_test2_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhi',
    maintainer_email='abhi@todo.todo',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros2_python_test2.publisher:main',
            'listener = ros2_python_test2.subscriber:main',
        ],
    },
)
