# Overview

The module Sensing and Perception (MOD.SW.SP) is a software module as part of OPIL (Open
Platform for
Innovation in
Logistics). It provides the pose of the AGV inside the built map of the environment in which the AGV is navigating and updates the map with the new sensor readings.
Additionally, it can built the map with SLAM (Simultaneous Localization And Mapping) if no map is given initially. It uses the laser scan data for map building and updating the map, and odometry sensors (encoders, IMU) together with lasers for localization.

Link to other pages:

opil-MODULENAME.l4ms.eu:

* [Robot Agent Node](http://opil-ran.l4ms.eu)
* [Human Agent Node](http://opil-han.l4ms.eu)
* [Sensor Agent Node](http://opil-san.l4ms.eu) 

## Localization

* Calculates a pose of the AGV inside the built map
* Calculates a pose of the AGV inside the incrementally built map of the SLAM process
* Sends a pose of the AGV to Task Planner, HMI, RAN

## Mapping

* Creates a map with SLAM
* Uses a map from CAD or as result of SLAM for localization
* Creates a topology for Task Planner
* Creates map updates from new laser reading of previously unoccupied areas


## Project layout

    test/	# The folder with testing files for sending the data with firos
    	config_files/    # The configuration files with firos json entities
    		machine_1/	 # The configuration files for machine_1 that sends the topology and pose
    			config.json	 # Change IP and interface for the machine_1 and machine_2
    			robotdescriptions.json	 # Always the same
    			robots.json	 # Defined what will be sent: topology and pose; and received: do_serve
    			whitelist.json	 # Also as above
    		machine_2/	 # The configuration files for machine_2 that receives the topology and pose
    			config.json	 # Change IP and interface for the machine_2
    			robotdescriptions.json	 # Always the same
    			robots.json	 # Defined what will be sent: do_serve; and received: topology and pose
    			whitelist.json	 # Also as above
    src/			# The source code that needs to be put inside the catkin workspace
        localization_and_mapping/  # The ROS metapackage for localization and SLAM
        	andymark_driver/	# Characheristic for Anda robot developed at ICENT
        	firos_config/	# Config files for sending the pose
        	husky/	# Drivers used by Anda robot
        	odometry_correction/	# Improvement of odometry with IMU sensor for Anda robot
        	lam_simulator/	# Simulations of world files in stage simulator
        		launch/	# Launch files for MURAPLAST floorplan and for ICENT floorplan
        	sensing_and_perception/	# The ROS package that creates a pose message to be send through firos
        maptogridmap/  # The ROS package for topology creation and map updates
        maplistener/  # The ROS package for visualization of ROS topics
