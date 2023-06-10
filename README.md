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
