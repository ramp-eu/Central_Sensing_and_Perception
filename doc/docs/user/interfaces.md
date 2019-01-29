# Workflow
* Pose of the AGV is calculated inside the RAN module (level 1 of OPIL).
* Map updates are calculated at the level 1, where laser sensor data are received.
* Topology is calculated in the level 3 of OPIL from the map file.
* There is no service calls implemented yet in firos so map is not transmited through firos. Each module should have its own copy of map file (from CAD or as SLAM result).
* Only map updates are sent through firos, which are calculated from the new sensor readings that hit the free grid cells. 
* TODO: Map merging is done from map updates of more robots into one gridmap.
* Global map creation from initial map and map updates is not done in this version (OPIL v2). Since there is no replanning in TP in v2, the map updates are only used for visualization in HMI.
* HMI should have the map file as RAN, and ability to present map updates over the initial map. 

# RAN machine with SP as part of it
The main reason having SP as part of RAN is to have high-rate closed navigation loop when relying on laser data and odometry attached for pose calculation, with no delay introduced when using the large data flow (lasers) through wifi. 

## Pose with covariance of the AGV
To start SP inside RAN you should use _lam_simulator_ ROS package, which contains prepared launch files with AMCL localization and tests in the simulator Stage.
Afterwards, use _sensing_and_perception_ ROS package, which publish AGV's pose with covariance. 
AMCL is a standard ROS package that publishes the tf_tree transformations so any navigation package (like move_base) can be attached to this node in a standard way.
There are three example maps that can be started in the simulator Stage:

* Example with the MURAPLAST factory floorplan:
```
terminal 1: roslaunch lam_simulator amcl_test_muraplast.launch
terminal 2: rosrun sensing_and_perception pubPoseWithCovariance
```

* Example with the ICENT lab floorplan in Zagreb review meeting demo:
```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 2: rosrun sensing_and_perception pubPoseWithCovariance 
```

* Example with the IML lab floorplan:
```
terminal 1: roslaunch lam_simulator IMLamcltest.launch
terminal 2: rosrun sensing_and_perception pubPoseWithCovariance 
```


## Map updates at Level 1 (laser data)

TODO: explain how map updates are calculated from laser data

# SP machine

TODO: explain how topology is created

# HMI machine

TODO: explain how map file needs to exist there and how map updates will be received

#Interconnection between machines

## Towards Orion CB
To send the topics to Orion Context Broker json files needs to be set properly inside the firos/config folder and firos needs to be running.

## Towards Physical IO
TODO: this is sent directly from Level 1, not through OCB
It is not yet used that something is being sent to IO from OCB. There are two reasons: a) the map is too big; b) there is no service call so AMCL can not work.
To send the topics through firos to Physical IO json files need to be set properly inside the firos/config folder and firos needs to be running.

## firos config json files explained between machine 1 and machine 2
On machine 1 all topics that are being sent through context broker need to be "subscriber", and that are being received from the context broker need to be "publisher". Topics are listed under ids and here we have "map" id and "robot_opil_v1" id.

### robots.json
```
{
   "map":{
       "topics": {
                "realtopology": {
                    "msg": "maptogridmap.msg.Gridmap",
                    "type": "subscriber"
                },
            	"realmap": {
                	"msg": "nav_msgs.msg.OccupancyGrid",
                	"type": "subscriber"

            	},
            	"nodes": {
                	"msg": "maptogridmap.msg.Nodes",
                	"type": "subscriber"

            	},
            	"edges": {
                	"msg": "maptogridmap.msg.Edges",
                	"type": "subscriber"

            	},
                "do_serve": {
                    "msg": "std_msgs.msg.Bool",
                    "type": "publisher"
                }
       }
   },
   "robot_opil_v1":{
        "topics": {
	    "pose_channel": {
                "msg": "geometry_msgs.msg.PoseWithCovarianceStamped",
                "type": "subscriber"
 	       }
	    }
	}

}
```
### whitelist.json
```
{
    "map": {
        "subscriber": ["realtopology","nodes","realmap","edges"],
        "publisher": ["do_serve"]
    },
    "robot_opil_v1": {
        "subscriber": ["pose_channel"]
    }
}
```
On machine 2 there is exactly the opposite subscriber/publisher from machine 1, so simply just replace subscriber and publisher.






