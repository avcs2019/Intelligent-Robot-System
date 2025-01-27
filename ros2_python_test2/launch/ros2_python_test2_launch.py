from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_python_test2',  # Replace with your package name
            executable='talker',  # Replace with the executable name
            name='ros2_python_test2_talker',
            output='screen',
            parameters=[
                {
                    'image_folder': '/home/abhi/Pictures',  # Update with your image folder path
                    'publish_rate': 5.0  # Update the publishing rate as needed
                }
            ]
        ),
#        Node(
#            package='ros2_python_test2',  # Replace with your package name
#            executable='listener',  # Replace with the executable name
#            name='ros2_python_test2_listener',
#            output='screen',
#            parameters=[
#                {
#                    'sub_topic': '/image_topic'  # Update with your image folder path
#                }
#            ]
#        )
    ])

