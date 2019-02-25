# Start Guide

To start SP module follow the [Install](./install/install.md) and [API Walkthrough](./user/api.md) procedures. In the following the list of ROS packages is given with the short explanation.

## Overview

In src folder there are four ROS packages:


## maptogridmap package

A ROS package for creating a gridmap with desired cell size from map_server (PNG or PGM file) and local map updates from an AGV, and a topology for Task Planner in the form of a graph with nodes and edges. This package is located in the Central SP.

## mapupdates package

A ROS package for creating local map updates from the laser readings. This package is located in the Locas SP on every AGV. Every AGV needs to start this package with the robot ID number set to 0, 1, etc., so that published topic has corresponding name as /robot_0/newObstacles, /robot_1/newObstacles, etc.

## maplistener package

A ROS package for testing subscribers to all created topics from mapupdates and maptogridmap packages and visualizing them in rviz.


## localization_and_mapping metapackage

A ROS metapackage containing packages for localization and mapping developed in the purposes of the L4MS project. 


### localization_and_mapping packages

#### lam_simulator

ROS package which demonstrates localization and mapping in Stage simulator. Relies on AMCL or gmapping algorithms started with the Stage simulator. Prepared demonstrations can also be used on the real robot.


#### andymark_driver

ROS drivers for omnidirectional andymark platform. Teleoperation and control relies on the nodes provided by the husky package.


#### husky

ROS package for interfacing with Clearpath Husky robot. It also includes nodes for teleoperation and control.


#### odometry_correction

ROS package which relies on robot_pose_ekf to fuse robot odometry with IMU data to improve odometry estimation.

#### sensing_and_perception

ROS package for publishing AGV's pose with covariance to be sent to context broker through firos. This package is located in the Local SP.

