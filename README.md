# mod.sw.sp

OPIL module: Sensing & Perception
Innovation Centre Nikola Tesla, written by Marija from May 6th 2018, contintributed by Goran, Jura, Petki and Ana

In src folder there are three packages:
* maptogridmap - creates gridmap with desired cell size from map_server and publish message to topic topology and also contains service getMap
* maplistener - test of listening of topic topology and calling the service getMap - everything is visualized in rviz
* localization_and_mapping - please read readme files inside it to simulate localization and mapping
some functionalities in maptogridmap package:
* mux_topics - sends the map on the whitelisted topic only when master requests
* show_map - shows map published on the whitelisted topic
config_files for firos:
* In test/config_files there are firos config jsons for machine_1 ubuntu and machine_2 ubuntu tested at ICENT that you need to copy to firos/config folder on each computer a different ones. Now machine_1 consists of nodes, edges and pose with covariance to be sent to machine_2 (which is in our test the opil server).

examples and prepared launch files:
* put these packages to your src folder of your catkin workspace or create a new one by typing catkin_init_workspace() in src folder
```
cd ..
catkin_make
```

reqired package map_server

# topology test - sending nodes and edges
machine 1:
(uncomment one of three maps in startmapserver.launch file - IML, MURAPLAST or Andamap)
```
terminal 1: roslaunch maptogridmap startmapserver.launch 
```
(change the parameter cell_size to desired value in startmaptogridmap.launch file)
```
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
```
(put in firos/config all json files from OPIL/config_files/machine_1)
```
terminal 3: rosrun firos core.py 
```
machine 2:
(put all json files in firos/config from OPIL/config_files/machine_2 and put package maptogridmap on machine 2 to have needed messages defined)
```
terminal 1: rosrun firos core.py
echo /map/nodes and /map/edges topics
```

# pose test - sending pose with covarianace
machine 1:
(put in firos/config all json files from OPIL/config_files/machine_1)
* Example with the MURAPLAST factory floorplan:
```
terminal 1: roslaunch lam_simulator amcl_test_muraplast.launch
terminal 2: rosrun sensing_and_perception pubPoseWithCovariance
terminal 3: rosrun firos core.py 
```

* Example with the ICENT lab floorplan in Zagreb review meeting demo:
```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 2: rosrun sensing_and_perception pubPoseWithCovariance 
terminal 3: rosrun firos core.py 
```

* Example with the IML lab floorplan:
```
terminal 1: roslaunch lam_simulator IMLamcltest.launch
terminal 2: rosrun sensing_and_perception pubPoseWithCovariance 
terminal 3: rosrun firos core.py 
```
machine 2:
(put in firos/config all json files from OPIL/config_files/machine_2)
```
terminal 1: roscore
terminal 2: rosrun firos core.py 
```

