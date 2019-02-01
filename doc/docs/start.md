# Start Guide

## Overview

In src folder there are four ROS packages:


## maptogridmap package

A ROS package for creating gridmap with desired cell size from map_server and publish custom created ROS messages to topics /map/topology, /map/nodes and /map/edges.

## mapupdates package

A ROS package for creating map local updates for the gridmap, nodes and edges from the laser readings to topics /robot_0/newObstacles, /robot_1/newObstacles, ...

## maplistener package

A ROS package for listening the topic realtopology and realmap sent through firos, and testing subscribers to all created topics and visualizing them in rviz.


## localization_and_mapping metapackage

A ROS metapackage containing packages for localization and mapping developed in the purposes of the L4MS project. 


### localization_and_mapping packages

#### lam_simulator

ROS package which demonstrantes localization and mapping in Stage simulator. Relies on AMCL, gmapping and Stage simulator.


#### andymark_driver

ROS drivers for omnidirectional andymark platform. Teleoperation and control relies on the nodes provided by the husky package


#### husky

ROS package for interfacing with Clearpath Husky robot. It also includes nodes for teleoperation and control.


#### odometry_correction

ROS package which relies on robot_pose_ekf to fuse robot odometry with IMU data to improve odometry estimation.

#### sensing_and_perception

ROS package for publishing AGV's pose with covariance to be sent to context brocker through firos. 

