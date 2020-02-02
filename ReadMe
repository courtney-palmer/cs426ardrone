# Senior Capstone: Building an Autonomous Drone
<!-- comment format for markdown-->
## Group Members
Courtney Palmer<br/><br/>
Matthew Page<br/><br/>
Robert Lowry<br/><br/>
Gary Lougheed

## Project Description
<!--  Our project was initially inspired by the DARPA subterranean challenge, which aims to find an effective solution for mapping an underground environment. Our group is focusing on how to respond to underground-based emergency scenarios that initially may be unsafe for a person to traverse. We propose to utilize an autonomous drone system that could map an unknown environment. After mapping an environment, the drone will navigate to certain waypoints and scan the area for pre-defined objects. If any identifiable objects are detected, the drone alerts the user of its location. We believe that this will be a beneficial solution because the drone could provide crucial locational data and removes the need for human involvement.

## Workspace Environment Setup
Please note that our project is dependent on ROS Kinetic, which is only available for Ubuntu 16.04 or below. ROS Kinetic may be installed through the following guide: 

http://wiki.ros.org/kinetic/Installation

After installation, a ROS Workspace will need to be created. Follow this guide to create a workspace:

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment


## Setting up the Project
To run the program, clone this build and copy it to your catkin workspace folder. If you followed the ROS guide, the workspace will be named catkin_ws. 

Then, open a terminal, navigate to the workspace folder, and enter the following: 
```
source /opt/ros/kinetic/setup.bash && source devel/setup.bash

catkin_build

```
This should generate two new folders called build and devel. 

Finally, copy the contents of the Plugins folder and paste it into devel->lib. 

## Running the Project
Finally we are ready to run the project! Open a new terninal and enter the following to run: 

```
source /opt/ros/kinetic/setup.bash && source devel/setup.bash

roslaunch cvg_sim_gazebo ardrone_testworld.launch

```
## Controls
Drone movement is dependent on the ROS package, tum_simulator. Information about tum_simulator can be found here: 

http://wiki.ros.org/tum_simulator

Here are the manual controls taken from the above link:

```
# fly forward
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly backward
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly to left 
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly to right 
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly up 
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly down 
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: -1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# counterclockwise rotation
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 1.0}}'

# clockwise rotation
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: -1.0}}'

# stop
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```
