# Purpose

ROS pacakge which corrects robot's odometry estimation by combining wheel odometry with IMU data. 
It relies on robot_pose_ekf pacakge ( http://wiki.ros.org/robot_pose_ekf ). 

Additionally, it contains two nodes: 
1. odometry_add_covar
--node that fills the covariance (high to rotation, low to translation) of the odometry published by the robot. 
  -- subscribed topic: odometry_input (type nav_msgs/Odometry)
  -- published topic: odometry_output (type nav_msgs/Odometry)

2. odometry_add_covar
--node that converts PoseWithCovariance msg to Odometry msg for the purposes of Rviz vizualization. 
  -- subscribed topic: pose_input (type geometry_msgs/PoseWithCovariance)
  -- published topic: odometry_output (type nav_msgs/Odometry)

# Install requirements 

$sudo apt-get install ros-$ROS_DISTRO-robot-pose-ekf

In case Xsense IMU is used:
$sudo apt-get install ros-$ROS_DISTRO-xsens-driver

# Usage

Determine which USB device belongs to IMU (e.g. ttyUSB0) and allow access for the IMU with:
$ sudo chmod 777 /dev/ttyUSB0

Then, set the parameters in the odometry_with_imu.launch file and then:
$roslaunch odometry_correction odometry_with_imu.launch
