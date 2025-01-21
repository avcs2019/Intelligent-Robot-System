# Turtlebot3 Gazebo World
## Install gazebo and gazebo_ros packages
```
 sudo apt install gazebo
```

```
 sudo apt install ros-humble-gazebo-ros-pkgs
```
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
source install/setup.bash
```
## Launch the world
```
ros2 launch turtlebot3_gazebo turtlebot3_test.launch.py
```
## Gazebo simulation
![Screenshot 2024-12-30 20:53:36](https://github.com/user-attachments/assets/b80f3f77-5abc-4c96-9a24-f3032ed075e8)

## Rviz simulation
![image](https://github.com/user-attachments/assets/93ccc589-2524-4633-ae9f-59af23a2acd2)

### To visualize the camera image add the image topic from the add button on rviz

![image](https://github.com/user-attachments/assets/6f12e516-0dd2-4a6f-9fed-81bfd79ef860)


## To run the bot arround in a new terminal 
(remember to source the package again in ther new terminal or you can add it to your bashrc so you dont need to do it each time a new terminal instance is open, research about it !!)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
