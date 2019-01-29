# localization and mapping metapackage

A ROS metapackage containing packages for localization and mapping developed in the purposes of the L4MF project. 

For required install instructions, please refer to individual package READMEs.



# Packages

## lam_simulator

ROS package which demonstrantes localization and mapping in Stage simulator. Relies on AMCL, gmapping and Stage simulator.


## andymark_driver

ROS drivers for omnidirectional andymark platform. Teleoperation and control relies on the nodes provided by the husky package


## husky

ROS package for interfacing with Clearpath Husky robot. It also includes nodes for teleoperation and control.


## odometry_correction

ROS package which relies on robot_pose_ekf to fuse robot odometry with IMU data to improve odometry estimation.

## sensing_and_perception

ROS package for sending pose with covariance to context brocker (firos). 
First launch AMCL localization (run amcl_test_muraplast.launch in lam_simulator/launch folder).
Second use and adapt the config files in firos_config for cloned firos package.
Third start firos and pose publishing with send_posewithcovariance.launch.
terminal 1: roslaunch lam_simulator amcl_test_muraplast.launch
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 


