# Purpose

ROS pacakge for interfacing with Andymark omnidirectional platform.  


# Install requirements 

Instalation of Sick Lidar ROS drivers
$ sudo apt-get install ros-$ROS_DISTRO-lms1xx

# Usage

Determine which USB device belongs to the platform (e.g. ttyUSB0) and allow access for the IMU with:
$ sudo chmod 777 /dev/ttyUSB0

Set up environmental varaibles, in our case:

$ export HUSKY_DESCRIPTION=~/catkin_ws/src/localization_and_mapping/husky/husky_description
$ export HUSKY_CONTROL=~/catkin_ws/src/localization_and_mapping/husky/husky_control

Launch the Lidar, IMU, odometry_correction and controllers with the following command

$ roslaunch andymark_driver andymark.launch
