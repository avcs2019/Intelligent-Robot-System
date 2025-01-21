# Turtlebot3 Gazebo World
## Revised files
[Burger model](https://github.com/Ruiji-Liu/turtlebot3_simulations/blob/main/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf)

[Gazebo world](https://github.com/Ruiji-Liu/turtlebot3_simulations/blob/main/turtlebot3_gazebo/worlds/test.world)

[Launch file](https://github.com/Ruiji-Liu/turtlebot3_simulations/blob/main/turtlebot3_gazebo/launch/turtlebot3_test.launch.py)
## Required packages
Install all the ros2 humble turtlebot3 packages
```
 sudo apt install ros-humble-turtlebot3*
```
## Install the package
```
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/Ruiji-Liu/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build
```
## Launch the world
```
ros2 launch turtlebot3_gazebo turtlebot3_test.launch.py
```
![Screenshot 2024-12-30 20:53:36](https://github.com/user-attachments/assets/b80f3f77-5abc-4c96-9a24-f3032ed075e8)
