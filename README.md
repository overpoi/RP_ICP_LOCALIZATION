This project demonstrates how to localize a robot in a known environment using 2D laser scans and the Iterative Closest Point (ICP) algorithm.
The system runs on ROS Noetic, integrates with Stage for simulation and uses RViz for visualization.

By aligning the online laser scan with a known environment map, the node estimates the robot’s pose and continuously updates the transform between the map frame and the robot’s laser frame.

# 📦 Dependencies

Make sure you have **ROS Noetic** installed and set up. Then install the following packages:

### Map Server
```sh
sudo apt install ros-noetic-map-server
```

### Stage-ROS and Teleop Twist Keyboard
```sh
sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
```

# 🚀 Launching the project

Clone the repo in a catkin_ws/src folder, then
AS
## Build the workspace
```sh
cd ~/catkin_ws
catkin_make
```
This compiles the code in /src and generated the executables and the libraries into /build and /devel

## Source the setup file
```sh
source devel/setup.bash
```
So ROS can find the new package

Or, add the source line to ~/.bashrc


## Run the launch file 
```sh
cd ~/catkin_ws/src
roslaunch RP_ICP_LOCALIZATION/launch_file.launch  
```

This will open Rviz and StageROS.
Set the 2D Pose Estimate on the robot starting position through Rviz.

## Control the robot
In a new terminal run 
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```
Use the IJKL, keys (like WASD) to move the robot around.

Enjoy!
