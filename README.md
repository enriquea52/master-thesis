# Pose Based EKF SLAM (PEKSLAM) Using LiDAR Scans and ICP 


Prototype and main components:

<p align="center">
<img src="./docs/imgs/turtlebot_hardware.png" alt="drawing" width="267" title = "Robots DofF"/>
</p>
<figcaption align="center"><b>Figure 1. Turtlebot 2: Robotic Platform for Testing the Algorithm</b></figcaption>
</figure>

The present package provides an implementation of the Posed based Extended Kalman Filter SLAM algorithm for solving the localization adn Mapping project storing poses in thestate vector in order to correct the trayector of the robot and plot the map resulting from taking a snapshot of the envvironment with a sensor in a given pose. For this implementation the sensor to be used is a RPLiDAR A2 as shown in Figure 1. The correction step will be based on the transformation provided by the ICP from two overlapping scans in the given trayectory. the provposed algorithm's pseudocode is shown in Figure.


<p align="center">
<img src="./docs/imgs/pseudo.png" alt="drawing" width="300" title = "PEKSLAM pseudocode"/>
<img src="./docs/imgs/slam.png" alt="drawing" width="270" title = "PEKSLAM result"/>

</p>
<figcaption align="center"><b> Figure 2. PEKSLAM pseudocode and result after execting a loop closure</b></figcaption>
</figure>

## Team Members:

This project has been carried out by:

* [](https://www.github.com/enriquea52) [Enrique Aleman]
* [Mohammad Alshimmari]


## Code Structure
```bash
├── CMakeLists.txt
├── config
│   ├── localization.rviz
│   └── parameters.yaml
├── docs
│   └── imgs
│       ├── pseudo.png
│       ├── slam.png
│       ├── software.png
│       └── turtlebot_hardware.png
├── launch
│   ├── localization.launch
│   └── simulation.launch
├── libs
│   ├── icp_odom.cpp
│   ├── icp_odom.h
│   ├── slam.cpp
│   ├── slam.h
│   ├── vis_utils.cpp
│   └── vis_utils.h
├── package.xml
├── README.md
├── scripts
│   └── laser_scan_to_point_cloud.py
└── src
    └── pekslam.cpp

```

## Required Packages

To make use of the present package, it is necessary to have the following packages and dependencies installed or install them by running the following commands in the working ROS workspace.

For the real platform
```bash

# Clone required packages
cd ~/catkin_ws/src
git clone https://bitbucket.org/udg_cirs/turtlebot.git # This repository!
git clone https://bitbucket.org/udg_cirs/turtlebot_description.git

#The kobuki mobile base
git clone https://bitbucket.org/udg_cirs/kobuki.git
git clone https://bitbucket.org/udg_cirs/kobuki_description.git
git clone https://bitbucket.org/udg_cirs/yujin_ocs.git

# The manipulator
git clone https://bitbucket.org/udg_cirs/swiftpro.git
git clone https://bitbucket.org/udg_cirs/swiftpro_description.git

# The Lidar
git clone https://github.com/Slamtec/rplidar_ros.git
```

For the simulated environment

```bash
# Clone required packages
cd ~/catkin_ws/src
git clone https://bitbucket.org/udg_cirs/turtlebot_desktop.git 
git clone https://bitbucket.org/udg_cirs/turtlebot_description.git 

#The kobuki mobile base
git clone https://bitbucket.org/udg_cirs/kobuki_desktop.git
git clone https://bitbucket.org/udg_cirs/kobuki_description.git

# The manipulator
git clone https://bitbucket.org/udg_cirs/swiftpro_desktop.git
git clone https://bitbucket.org/udg_cirs/swiftpro_description.git

# The simulation world
git clone https://bitbucket.org/udg_cirs/small_house_world.git 

# Kobuki mobile base dependencies
git clone https://bitbucket.org/udg_cirs/yujin_ocs.git

# ROS controllers
sudo apt install ros-noetic-ros-control                     
sudo apt install ros-noetic-ros-controllers

# Install xterm terminal emulator
sudo apt-get install -y xterm 


```

## How to use it:

Go to your ROS_workspace/src directory

```bash
  $ cd ~/ROS_workspace/src
```

Clone the project or extact the compressed file in ROS_workspace/src directory 

```bash
  $ git clone https://github.com/enriquea52/Hands-On-Localization
```

Go back to ROS_workspace directory

```bash
  $ cd ../
```

Run catkin_make or catkin build

```bash
  $ catkin_make
  $ source ./devel/setup.bash 
```
The package can be run either for a real platform or for a simulated environment, the following are the comands for each case.

* Real Platform
```bash
  $ roslaunch localization localization.launch tbot_name:=turtlebotX
```
Where X can be either 1, 2 or 3
* Simulated Environment

```bash
  $ roslaunch localization simulation.launch
```

## Software Architecture

<p align="center">
<img src="./docs/imgs/software.png" alt="drawing" width="800" title = "Software Diagram"/>
</p>
<figcaption align="center"><b>Figure 3. ROS software architecture</b></figcaption>
</figure>

## Video demonstration (Click on video):

<p float="left">

Posed Based EKF SLAM  (Real System)

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/IRcGNB0fS8E/0.jpg)](https://youtu.be/IRcGNB0fS8E)

</p>


