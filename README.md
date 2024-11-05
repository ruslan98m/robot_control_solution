# Robot Control Solution

This repository contains a robot control solution built with ROS. This document outlines the steps to run the `task1.launch`, `task2.launch`, `task3.launch` files and the installation process for the necessary libraries.

## Prerequisites

Before running the launch file, ensure that you have the following prerequisites installed:

- ROS Noetic - full(with Gazebo simulator) installed and configured.
- CMake for building the project.
- A compatible compiler for building C++ files.

## Installing dependencies

### Eigen

Eigen is a C++ template library for linear algebra. You can install Eigen using the following methods:


On Ubuntu, you can install Eigen using the package manager:

```bash
sudo apt-get install libeigen3-dev
```

### Installing Ruckig 

```bash
git clone https://github.com/pantor/ruckig
```
build and install it

```bash
cd ruckig
mkdir build
cd build
cmake ..
make 
sudo make install
```

### Installing python3 dependencies  
dependencies must be installed
```bash
cd robot_control_solution
pip3 install -r requirements.txt
```


## Launching project

 **ur5_bringup.launch** launches the gazebo scene with the UR5 robot
```bash
roslaunch robot_control_solution ur5_bringup.launch
```
**task1.launch** launches the task 1 project (control of the robot's joints by sinus)
```bash
roslaunch robot_control_solution task1.launch
```
**task2.launch** launches a project with movePtP and moveLine motion functions in **C++**
```bash
roslaunch robot_control_solution task2.launch
```
**task3.launch** launches a project with movePtP and moveLine motion functions in **python3**
```bash
roslaunch robot_control_solution task3.launch
```
