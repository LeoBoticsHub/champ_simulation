
# champ [![Build Status](https://travis-ci.org/chvmp/champ.svg?branch=master)](https://travis-ci.org/chvmp/champ) 
ROS2 Package for CHAMP Quadruped Controller in ROS 2 Humble.

![champ](https://raw.githubusercontent.com/chvmp/champ/master/docs/images/robots.gif)

CHAMP is an open source development framework for building new quadrupedal robots and developing new control algorithms. The control framework is based on [*"Hierarchical controller for highly dynamic locomotion utilizing pattern modulation and impedance control : implementation on the MIT Cheetah robot"*](https://dspace.mit.edu/handle/1721.1/85490).

Core Features:

- Gazebo simulation environment.
- Ignition Fortrees.

Tested on:

- Ubuntu 22.04 (ROS2 Humble)


## 1. Installation

### 1.1 Clone and install all dependencies:

    sudo apt install -y python3-rosdep
    rosdep update

    cd <your_ws>/src
    git clone --recursive https://github.com/LeoBoticsHub/champ.git -b ros2_dev
    git clone https://github.com/LeoBoticsHub/champ_simulation.git
    git clone https://github.com/LeoBoticsHub/sensors_description.git -b 1-ros2_gazebo_simulation
    git clone https://github.com/LeoBoticsHub/b1_description.git -b ros2_control_plugin
    cd ..
    rosdep install --from-paths src --ignore-src -r -y

The Ignition Fortrees simulation is now available with Unitree B1 robot.

### 1.2 Build your workspace:

    cd <your_ws>
    colcon build
    . <your_ws>/install/setup.bash

## 2. Quick Start

### 2.1 Walking demo in Gazebo Classic:

#### 2.1.1 Run the simulation:

    ros2 launch champ_simulation robot_sim.launch.py 

#### 2.1.2 Run the teleop node:

    ros2 launch champ_simulation teleop.launch.py 

### 2.1 Walking demo in Ignition Fortrees:

#### 2.1.1 Run the simulation:

    ros2 launch champ_simulation robot_ign_sim.launch.py 

#### 2.1.2 Run the teleop node:

    ros2 launch champ_simulation teleop.launch.py

### 2.1 Sensors visualization in RVIZ 2:

#### 2.1.1 Run the simulation:

    ros2 launch champ_simulation robot_sim.launch.py SENSORS:=true
    
or
    
    ros2 launch champ_simulation robot_ign_sim.launch.py SENSORS:=true

#### 2.1.2 Run the teleop node:

    ros2 launch champ_simulation teleop.launch.py    

#### 2.1.2 Run RVIZ 2 visualization:

    ros2 launch b1_description b1_rviz_champ.launch.py  
