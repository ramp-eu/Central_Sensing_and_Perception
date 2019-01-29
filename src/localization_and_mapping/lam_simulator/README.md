# Purpose

ROS pacakge


# Install requirements 

Install AMCL:
$ sudo apt-get install ros-$ROS_DISTRO-amcl 

Install Gmapping:
$ sudo apt-get install ros-$ROS_DISTRO-gmapping

Install ROS Stage:
$ sudo apt-get install ros-$ROS_DISTRO-stage-ros

Install ROS Map server:
$ sudo apt-get install ros-$ROS_DISTRO-map-server


# Usage

## launch folder
Contains launch files for demonstration of AMCL and Gmapping
	
### amcl_test_muraplast.launch
	- runs the map server which publishes a map of the environment saved in ../yaml/bitmaps folder 
	- runs the simulator with the map of the environment saved in ../world/elements folder
	- launches amcl_omnisim.launch 
	- opens rviz visualizer
	
### amcl_omnisim.launch
	- runs AMCL package
	
### gmapping_test_muraplast.launch
	- runs the simulator with the map of the environment saved in ../world/elements folder
	- launches gmapping_omnisim.launch 
	- opens rviz visualizer
	
### gmapping_omnisim.launch
	- runs Gmapping package
	
## rviz_cfg folder
Contains the configuration for rviz visualizer

## world folder
Contains files with the configuration of the simulated envirnoment in stage_stage

## yaml folder
Contains files with the configuration for the map loaded by map_server	
