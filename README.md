![Altax](https://altax.net/images/altax.png "Altax")

# Robot 4wd dds115 base controller

This repository contains the Rover Base Controller, a ROS package for controlling a 4WD mobile robot equipped with DDSM115 motors. The package provides velocity control via /cmd_vel, reads motor feedback for RPM and current, and publishes odometry for localization.

Features
* Velocity Control: Subscribes to /cmd_vel and converts Twist messages into motor RPM commands.
* Motor Feedback: Reads RPM and current from each motor and publishes them on /motor_rpms and /motor_currents.
* Odometry Computation: Integrates wheel speeds to estimate the robot’s position and publishes it on /odom.
* ROS Compatibility: Designed for ROS Noetic on Ubuntu 20.04, compatible with teleop_twist_keyboard for manual control.
* Configurable Parameters: Supports YAML-based configuration for motor IDs, wheel directions, robot dimensions, and control loop frequency.