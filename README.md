# UR5e Item Sorter

# Table of Contents
-# Include a Table of Contents at the start of your README (this can be auto-generated). 

# Project Overview

## Customer Problem
A small-scale local manufacturing client requires a robotic solution to automate the sorting and organisation of small, packaged components along an assembly line. Currently, workers manually identify, pick, and place these components into designated trays based on visual characteristics such as colour and shape. The manual process of sorting is bottlenecking the assembly line with insufficient output and excessive errors, as well.

## Robot Functionality
Our product is an UR5e robotic system that autonomously identifies, picks, and sorts objects from a conveyor or workspace into correct storage bins, based on visual classification. This system aims to improve throughput, reduce labour fatigue, and increase sorting accuracy while maintaining safe operation within the defined workspace.

The system is designed to operate in a semi-structured environment with objects on a relatively flat surface. A fixed Intel Realsense (or alternative RGBD) camera will provide a real-time view of the workspace. 

The simplified workflow will consist of: 
- Using a trained machine learning model, the RGBD camera will detect, and classify objects and bins on the work surface. 
- Determining the 3D pose of each object detected using calibrated depth camera points
- Plan and execute pick-and-place trajectories using MoveIt
- Move to and grip each object using the custom servo-actuated gripper end-effector
- Move to and drop objects into the corresponding sorting bin

## Demo Video
(OneDrive Link?)

# System Architecture

## rqt Graph

## Closed-Loop System Behaviour

## Custom messages and services
### LabelledPose.msg and LabelledPoseArray.msg
### Move.srv
### TransformLookup.srv
### TransformLookupArray.srv

# Technical Components

## Computer Vision

## Custom End-Effector

## System Visualisation

## Closed-Loop Operation

# Installation and Setup

## Hardware setup

### UR5e 

### RealSense Camera

### Teensy & End Effector

## Dependencies

## System Variables and Calibration

# Running the System

## Launch commands

## Expected outputs

## Common Troubleshooting

# Results and Demonstration

## Iterations

## Final Result (inc. quantitative result)

## Compare against design goals

# Discussion and Future Work

## Development Challenges

## Novelty of Existing Solution

## Directions for Future Work

# Contributors and Roles
## Julian Britton
## Bryson Chen
## Matthew Viegas

# Repository Structure

## sorter_ws
### src
## unused_pkgs
### object_detect
## ur_gazebo
## ws_moveit2

# References and Acknowledgements

Item Picker for the UR5e specifically built for use in the MTRN4231 labs.

**Moveit Setup Instructions**
A slightly modified source install of moveit is provided for MTRN4231. To use it follow the build instructions:

1. Install Dependencies <pre>sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
sudo apt install python3-vcstool
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
</pre>
2. Build the workspace <pre>cd ws_moveit2
colcon build --mixin release</pre>

**UR Setup Instructions**
The ur_robot_driver needs to be installed to run the robot and the sim:

<pre>sudo apt install ros-humble-ur</pre>

Run robot calibration before running anything else:

<pre>ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=&lt;robot_ip&gt; target_filename:="${HOME}/my_robot_calibration.yaml"</pre>

**Running in ROS**
A setup script is provided in **setup.bash**. To use it run: <pre>source setup.bash</pre>


**Running in Simulation**
Simulation uses gazebo and is sourced from the directory https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation
To install dependencies: <pre>cd ur_gazebo/src
rosdep update && rosdep install --ignore-src --from-paths . -y</pre>
Then build:
<pre>colcon build --symlink-install</pre>
This allows us to launch with the provided launch file to test:
<pre>ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py</pre> or to launch without the moveit plugin
<pre>ros2 launch ur_simulation_gazebo ur_sim_control.launch.py</pre>
