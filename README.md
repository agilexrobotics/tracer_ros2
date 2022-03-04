# ROS2 Packages for Tracer Mobile Robot

## Packages

This repository contains minimal packages to control the tracer robot using ROS. 

* tracer_base: a ROS wrapper around [ugv_sdk](https://github.com/westonrobot/ugv_sdk) to monitor and control the tracer robot
* tracer_msgs: tracer related message definitions

## Supported Hardware

* Tracer
* Tracer-mini

## Basic usage of the ROS packages

1. Clone the packages into your colcon workspace and compile

    (the following instructions assume your catkin workspace is at: ~/ros2_ws/src)

    ```
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/westonrobot/ugv_sdk.git
    $ git clone https://github.com/agilexrobotics/tracer_ros2.git
    $ cd ..
    $ colcon build
    ```
2. Setup CAN-To-USB adapter

* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
    
* first time use tracer-ros2 package
   ```
   $ cd ~/your_ws/src/ugv_sdk/scripts/
   $ bash setup_can2usb.bash
   ```
   
* if not the first time use tracer-ros2 package(Run this command every time you turn off the power) 
   ```
   $ cd ~/ros2_ws/src/ugv_sdk/scripts/
   $ bash bringup_can2usb_500k.bash
   ```
   
* Testing command
    ```
    # receiving data from can0
    $ candump can0
    ```
3. Launch ROS nodes
 
* Start the base node for the Tracer robot

    ```
    $ ros2 launch tracer_base tracer_base.launch.py
    ```
    or
     ```
    $ ros2 launch tracer_base tracer_mini_base.launch.py
    ```

* Then you can send command to the robot
    ```
    $ ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
    angular:
    x: 0.0
    y: 0.0
    z: 0.0" 

    ```
**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
