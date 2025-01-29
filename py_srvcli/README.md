ROS NODES

bBased on ROS2 humble wiki example found at: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

Repo for ros2 service -  client service:

$ sudo apt install ros-humble-example_interfaces

example descrition: The client send two int64 variables, and the server returns the sum. 

$ ros2 run py_srvcli service     # to run the server

$ros2 run py_srvcli client 2 6   # run the client

Note: Services are only used for short, immediate tasks with a request-response structure.
