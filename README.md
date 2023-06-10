# Hands-on-Planning Project


## Members of the goup:

This project has been carried out by:

* Ahmed Alghfeli
* Hassan Alhosani
* Reem Almeheri

## Aim of the package:

This package implements an exploration algorithm for the Kobuki robot, which is simulated on Stonefish.

## Prerequisites

Before using this package, make sure you have the following dependencies installed:

* Python 3
* ROS
* Rviz and its plugin
* Octmap servere and its plugins
* stonefish
* koubuki and its packages for stonefish
* numpy version 1.24.1
* scipy library


## Usage Instructions:

To use this package, you need to roslaunch a single file that contains all the necessary nodes for exploration. Make sure that this package is in the same workspace as the Stonefish ROS, Turtlebot, Kobuki, and SwiftPro packages.

For Turtlebot on Stonefish, follow these steps:

Launch the exploration nodes by executing the following command:

1) Launch the 

```bash
roslaunch hop_project explor.launch
```
## Troubleshooting
Sometimes, during the exploration process, the arm may hit a wall and get stuck. To resolve this issue, follow these steps:

1) If the arm is open, run the following command to close it:

```bash
rostopic pub /turtlebot/swiftpro/joint_velocity_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
 [0 , 0, -0.5, 0]"
 ```

2) Once the arm is closed, turn off the joint speed by running the following command:
   
 ```bash
rostopic pub /turtlebot/swiftpro/joint_velocity_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
 [0 , 0, 0, 0]"
 ```

 These steps should help resolve the issue for the arm to not gets stuck in a wall while exploring.