from setuptools import setup

package_name = 'ros2_block_bt'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['py_trees', 'py_trees_ros', 'setuptools', 'message_filters', 'sensor_msgs'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 behavior tree implementation using py_trees_ros',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_bt0 = ros2_block_bt.behavior_tree:main',
        ],
    },
)

