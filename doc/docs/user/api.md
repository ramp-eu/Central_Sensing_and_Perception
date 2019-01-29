#Topology
The topology that needs to be send to Task Planner is composed of nodes and edges. In the future it will be a single graph message, but since firos is not supporting arrays of custom ROS messages it is divided into two ROS messages: nodes and edges.

Nodes.msg

	Header header	# standard ROS header
	nav_msgs/MapMetaData info	# number of cells in x and y directions of the gridmap, the size of the cell
	float64[] x	# x coordinate of the cell centre
	float64[] y	# y coordinate of the cell centre
	string[] name	# e.g. vertex_0
	string[] uuid	# unique id of a node
	
Edges.msg

	Header header	# standard ROS header 
	string[] uuid_src	# unique id of a source node of the edge
	string[] uuid_dest	# unique id of a destination node of the edge
	string[] name	# e.g. edge_0_1
	string[] uuid	# unique id of an edge
 
## Creation of Nodes in maptogridmap package
Nodes are all free cells of rectangular square size that does not contain any obstacle within it. The obstacles are readed from the map PNG or PGM file loaded by calling the map_server node. There are three maps prepared in startmapserver.launch, where MURAPLAST florplan is uncommented and IML lab and ICENT lab are commented for the later usage.

```
<launch>
<!--MURAPLAST floorplan-->
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/floorplanMP.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>


<!--IML lab floorplan	
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/IMLlab.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
-->	
		
<!--ICENT lab floorplan
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/Andamapa.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
-->

<node name="rviz" pkg="rviz" type="rviz" args="-d $(find maptogridmap)/singlerobot.rviz" /> 

</launch>
```

The size of the cell is given by the parameter in startmaptogridmap.launch:
```
<launch>

    <node name="map2gm" pkg="maptogridmap" type="map2gm" >
        <param name="cell_size" type="double" value="0.2" />
    </node>

</launch>
```

In this example it is set to 0.2m. Values that are presented in context broker are coordinates of the cell center (x,y), a name of the node in the form of "vertex_0" and the node's uuid. The message that is sent through firos can be found here: maptogridmap/msg/Nodes.msg

## Creation of Edges in maptogridmap package
Edges are pairs of neighbor nodes. Neighbors are defined between two nodes which have their centres' coordinates distanced for _cell_size_. The edges are bidirectional, meaning two neighbor nodes n and m forms the edge (n,m) and (m,n) which are identical.
Values of Edges are the source node's uuid, named as _uuid_src_, the destination node's uuid, named as _uuid_dest_, the name of the edge in the form of "edge_0_1" meaning that two nodes with names "vertex_0" and "vertex_1" are connected with the edge, and the edge's uuid. 
The message that is sent through firos can be found here: maptogridmap/msg/Edges.msg



# <a name="poswithcov">Pose with covariance</a>

A standard ROS message is used for sending the pose of the AGV:
	
	geometry_msgs.msg.PoseWithCovarianceStamped
	
To be able to send this message, two modules needs to be started: the AMCL package and the sensing_and_perception package. The AMCL package provides localization of the AGV inside a given map using the odometry and laser sensors. The sensing_and_perception package combines the covariance calculated by the AMCL package and the the global pose in the map frame as a result of a combination of odometry and AMCL localization with the laser.
First launch AMCL localization (run amcl_test_muraplast.launch in lam_simulator/launch folder).
Second use and adapt the config files in firos_config inside the localization_and_mapping metapackage.
Third start firos and pose publishing with send_posewithcovariance.launch.

Example with the MURAPLAST factory floorplan:
```
terminal 1: roslaunch lam_simulator amcl_test_muraplast.launch
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 
```

Example with the ICENT lab floorplan in Zagreb review meeting demo:
```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 
```

Example with the IML lab floorplan:
```
terminal 1: roslaunch lam_simulator IMLamcltest.launch
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 
```

#SLAM

When the map (or we also say the initial map) is not available, the SLAM process needs to be used to create it.

Inside the package lam_simulator there is a launch file (folder _localization_and_mapping/lam_simulator/launch/_). Run:

```
roslaunch roslaunch lam_simulator gmapping_test_muraplast.launch
```


The launch file starts gmappping SLAM algorithm, the simulation arena with the MURAPLAST factory floorplan and RVIZ.
The robot can be moved by moving the mouse while holding the pointer over the robot, or by using the ROS teleoperation package which is used for the real robot:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

The mapped environment is visualized in RVIZ.
After you are sattisfied with the built map presented in RVIZ, save it and use it later for AMCL localization by starting the map saver:

```
rosrun map_server map_saver -f mapfile
```

#Preparing the built map to be used for localization and navigation

As a result of SLAM you will obtain in the current folder where you called the map_saver command the mapfile.pgm and mapfile.yaml files.
This is an example:

```
image: mapfile.pgm
resolution: 0.068000
origin: [-25.024000, -25.024000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

To simulate this map in Stage you need to do the following. By default map_server and Stage have different reference global map coordinate systems. We choose to have the coordinate system at the lower left corner of the map. To have that first autocrop the mapfile.pgm so that borders are occupied, otherwise map_server and Stage will have different scales since Stage does the autocrop by itself. To do so open the mapfile.pgm in gimp, autocrop and put it in lam_simulator/worlds/elements folder for Stage world file and to yaml/bitmaps folder for map_server. Open the yaml file that is created by the map_saver, put origin to zero so that the origin will be the lower left corner and put it in the yaml folder. Do not forget to change the path to the mapfile.pgm (in this example bitmaps folder is added).

```
image: bitmaps/mapfile.pgm
resolution: 0.068000
origin: [0, 0, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

The Stage's world file should have defined size of the floorplan and pose of the floorplan which defines the coordinate frame. There is a parameter for map resolution but it is ignored. Put the right size of the floorplan by calculating resolution*numpixels and put the origin to half of it. 

Here is the example for the IML lab floorplan. The size of the map in pixels is 7057 x 2308 and the size of the pixel is 0.007 m (this is the resolution in IMLlab.yaml file) so the Stage world file looks as follows:

```
floorplan( 
bitmap "elements/smartface_topologie_entwurf.png"
#map_resolution     0.068 
  pose [24.6995 8.078 0.000 0.000] 
  size [49.399 16.156 0.800] 
  name "IMLlab"
)
```
Maybe you need to set the treshold of the image to have all the obstacle visible also in rviz, as was the case with this png file, since Stage has different tresholds for occupied pixels.


#Map updates

TODO: put text and examples

# Examples
## Testing if ROS topics for Nodes and Edges are sent to Orion Context Broker:
On opil server make a clean start of context broker:
```
sudo docker-compose down
sudo docker-compose up
```
Check in firefox if http://OPIL_SERVER_IP:1026/v2/entities is blank (replace OPIL_SERVER_IP with the correct IP address).

On machine 1 start:

* Uncomment the map you want to use (MP, IML or Anda) and comment the rest of maps in startmapserver.launch.
```
terminal 1: roslaunch maptogridmap startmapserver.launch
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
terminal 3: rosrun firos core.py (put in firos/config all json files from OPIL/config_files/machine_1)
```
Refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities. There should be under id "map" topics "nodes" and "edges" with their values.


## Testing sending pose with covariance on machine_1
Just start after previous three terminals. If you want to test only sending pose with covariance, repeat the commands in section [Pose with covariance](#poswithcov).
Start the simulation Stage with the amcl localization depending on the map you are using in startmapserver.launch. For example, if you are using MP map use amcl_test_muraplast.launch; if you are using IML map use IMLamcltest.launch; otherwise, if you are using ICENT map use the following:
```
terminal 4: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
```
This starts stage simulator and amcl localization. 
```
terminal 5: rosrun sensing_and_perception pubPoseWithCovariance
```
Now you can refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities.
There should be under id "robot_opil_v1" with topic "pose_channel".
Simply move the robot in stage by dragging it with the mouse and refresh the firefox to see the update of pose_channel.


## Testing if topics for Nodes, Edges, and PoseWithCovariance are received on machine_2 through firos
```
terminal 1: roscore
terminal 2: rosrun firos core.py (put in firos/config all json files from OPIL/config_files/machine_2)
```
_maptogridmap_ package needs to be on the machine_2 - it is not important that the source code is in there but only that msg files and CMakeLists.txt compiling them are there.
Now you are able to echo all ros topics:
```
rostopic echo /map/nodes
rostopic echo /map/edges
rostopic echo /robot_opil_v1/pose_channel
```
* Example output for the IML map - Nodes

```
$rostopic echo /map/nodes
header: 
  seq: 196
  stamp: 
    secs: 1548412279
    nsecs: 782414849
  frame_id: "map"
info: 
  map_load_time: 
    secs: 1548412279
    nsecs: 782414480
  resolution: 2.0
  width: 25
  height: 9
  origin: 
    position: 
      x: 0.0
      y: 0.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
x: [1.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 17.0, 17.0, 17.0, 17.0, 19.0, 19.0, 19.0, 19.0, 21.0, 21.0, 21.0, 21.0, 21.0, 21.0, 23.0, 23.0, 23.0, 23.0, 23.0, 23.0, 25.0, 25.0, 25.0, 25.0, 27.0, 27.0, 27.0, 27.0, 29.0, 29.0, 29.0, 29.0, 29.0, 29.0, 31.0, 31.0, 31.0, 31.0, 33.0, 33.0, 35.0, 35.0, 35.0, 35.0, 37.0, 37.0, 37.0, 37.0, 39.0, 39.0, 39.0, 39.0, 39.0, 39.0, 41.0, 41.0, 41.0, 41.0, 43.0, 43.0, 43.0, 43.0, 45.0, 45.0, 45.0, 45.0, 47.0, 47.0, 47.0, 47.0, 49.0]
y: [17.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 11.0, 13.0, 17.0, 9.0, 11.0, 13.0, 17.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 11.0, 13.0, 17.0, 9.0, 11.0, 13.0, 17.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 17.0, 9.0, 17.0, 9.0, 11.0, 13.0, 17.0, 5.0, 7.0, 9.0, 11.0, 3.0, 5.0, 7.0, 9.0, 11.0, 17.0, 3.0, 5.0, 7.0, 17.0, 3.0, 5.0, 7.0, 17.0, 3.0, 5.0, 7.0, 17.0, 3.0, 5.0, 7.0, 17.0, 17.0]
name: [vertex_8, vertex_10, vertex_11, vertex_12, vertex_13, vertex_14, vertex_15, vertex_19,
  vertex_20, vertex_21, vertex_22, vertex_23, vertex_24, vertex_26, vertex_29, vertex_30,
  vertex_31, vertex_32, vertex_33, vertex_35, vertex_38, vertex_39, vertex_40, vertex_41,
  vertex_42, vertex_44, vertex_47, vertex_48, vertex_49, vertex_50, vertex_51, vertex_53,
  vertex_56, vertex_57, vertex_58, vertex_59, vertex_60, vertex_62, vertex_65, vertex_66,
  vertex_67, vertex_68, vertex_69, vertex_71, vertex_76, vertex_77, vertex_78, vertex_80,
  vertex_85, vertex_86, vertex_87, vertex_89, vertex_92, vertex_93, vertex_94, vertex_95,
  vertex_96, vertex_98, vertex_101, vertex_102, vertex_103, vertex_104, vertex_105,
  vertex_107, vertex_112, vertex_113, vertex_114, vertex_116, vertex_121, vertex_122,
  vertex_123, vertex_125, vertex_128, vertex_129, vertex_130, vertex_131, vertex_132,
  vertex_134, vertex_137, vertex_138, vertex_139, vertex_143, vertex_148, vertex_152,
  vertex_157, vertex_158, vertex_159, vertex_161, vertex_164, vertex_165, vertex_166,
  vertex_167, vertex_172, vertex_173, vertex_174, vertex_175, vertex_176, vertex_179,
  vertex_181, vertex_182, vertex_183, vertex_188, vertex_190, vertex_191, vertex_192,
  vertex_197, vertex_199, vertex_200, vertex_201, vertex_206, vertex_208, vertex_209,
  vertex_210, vertex_215, vertex_224]
uuid: [931954c4-3d54-41fb-8a50-825336423614, 2c0b4f91-8780-45ba-93d5-dd60ab3ea61d, 706b0231-edbb-443e-b80b-330691cdba41,
  23f0fa05-6629-4f0c-a27f-9505006c5fe2, 98aad7a8-2820-4b16-b15d-603f83e1a5da, a0391f1b-8439-4dcd-b06b-3cbb3f2d7dfd,
  a378fbe4-504f-4132-9dd8-68507765de36, b0605c16-a416-4afc-b3c2-821e4fd9d7e5, 0d9f364e-901b-4845-8d6f-76c1122b97d1,
  4dadd7bd-9471-4c57-9b42-9fa89974de16, b51db75b-b22c-466f-8475-26c2ea99b2dc, 24b267fc-ab92-4a71-93c9-9a84785926cb,
  b11c9ae9-c10b-4aef-a299-121d58515840, dfd95cd4-3009-4d13-8587-cb2658db9888, 604067cb-d2f5-47b5-bfa5-8c1e6021b9c8,
  f4712b59-dd34-47ab-bc20-4965f9b36e2a, 8882f309-f497-40df-8cae-b358f53f1524, ce504224-af83-4d89-be6d-f59d6f114597,
  6028db74-70af-40de-b62b-fc0c113f07ba, f1b3ba92-3bba-47ea-9e8f-a248057bc296, c48e49be-1f97-4218-9870-3b73490945b0,
  ade7bff4-1c5f-4009-8f3c-9d35c57a9936, f2c90bdc-8325-446a-ae8a-94f1f0bce245, f63c03ca-f261-4c80-be3a-64f914e1e714,
  c10a9f4a-e6ae-4ae7-8eb5-7c2acb09dd89, 173d07bb-2512-4ec0-87e8-b4ba588b2781, 7dde7e83-e1a7-4e8b-9675-ef7577bdcb86,
  2f529d61-3452-4472-9d6a-4904f61aaea2, 39ff8173-1e10-488f-99f4-6bac128918b5, ef72e9b0-f87b-4ba8-af44-1c3ac8613258,
  44892486-e918-44c3-b42c-9771db99aa8c, 5abb38f3-a5ce-48c5-a1d2-609ffb791a69, c41a6308-4f6f-4073-871e-91b874d5c81f,
  49f57e6d-2474-4937-be38-f395aeaf21aa, e48dae03-49a4-4f7d-9ab8-1e6184f6a575, b26ededc-2ac5-47b3-928f-3f93f3ae3a4e,
  8364bb0f-00c3-40ed-855c-9033dc960649, c5bdffb1-2e50-42d0-a5be-3f3a4023e7c4, 42c3a87a-68d9-45df-8e4e-091a469b22cb,
  2f31e597-7c8a-4cdf-a4f6-113686a031df, a44f64b3-27c6-4436-a3a2-b49bb07cd974, 2fe86d4c-83a7-4180-9618-a6524ebd2511,
  91f1a197-f7b8-4653-9eb3-4c1a52ee21fb, 4733f7f7-08bc-4678-86f2-e750edc0a510, 4356e371-d349-479b-a9c1-abb22346a6fc,
  2189297d-90fa-435f-acd1-63f7af2b5f6b, 690f55e0-87f6-4c5d-8a56-b4fe367ca62b, 8bd310ba-9a12-4c5a-8f57-3bef729ed76e,
  c63be3e6-6a3f-4621-a118-0a216b7c0e0b, 44333768-5fa7-4dad-a870-f7be07f0afa0, 8ff3b6db-241b-4c9a-9b3f-d436fc8b79c5,
  eb5f0148-97ba-45b5-a7a9-d7d0cbdfae26, 766478dd-f712-404a-b374-f0f85796719b, 5c9e1983-243e-4624-9c3c-a0f52932f22f,
  f96bef75-78fb-41ba-8f75-f5f8f5db2adf, 91a2b1da-ec32-475e-b9b6-855972638751, 9024fce9-eb5d-48b4-9e70-16a693b8fd32,
  d7776ffe-ed0f-4368-9434-e7492b65d5b7, 6629a3d6-14a1-4ab7-a6a0-d3bd7f7303e1, 48ce49bb-1940-4380-b22a-9579e2cd5169,
  905e775f-8f6b-4303-84ef-f791ccecb448, 5db5eb57-df39-471a-a950-8b27b2b9963c, ced05dba-4c4d-430e-ae1e-4aa1d979843b,
  d012a1f1-5c07-41c8-a446-4a469898c6a3, 66fc4cee-84f2-42e4-aa7f-3986f49f2c60, 64d71e60-591c-4002-9cfa-53502082eaac,
  9716ef04-9d33-4874-adfd-fd6867ab6cfb, ea0728f5-e93c-48c1-976e-54c262d5d8fa, f8019862-df9e-492f-86cf-ab012cd47fa2,
  6f0c1703-4408-4ddd-9bd4-e74d0d57334c, 56cf9ba0-4cf3-4957-b84b-39637fd5e2ae, 9e7ace98-48ac-4da1-bdcb-311851d578a7,
  fb832fba-328c-4024-aa62-116b1cb6c6da, 03ea5b5d-ad04-4e9f-a50b-a722057eb7df, 9dc00c84-fcb1-499b-b734-91ee5a295973,
  e723997f-6aa3-4868-a71b-56aaed817bca, 93be216d-9b42-44b7-8103-5807bd34df42, 271bc16c-85c5-40b7-b4f6-1208d569b08d,
  cbf79ff4-d961-4379-9fc2-0d379b517bd1, e1874ac4-61b4-4238-b1da-6badb3b508ad, 05273595-b5b9-4bd8-8b72-8122d9b79ac8,
  b5ef1bf2-fcea-42d4-bc9e-9f6c6d921ab1, e279f55f-b8d1-4fca-9803-094972f97125, 6405de78-cddd-4fe7-aab1-df11059726e5,
  061eb5fc-c6a5-4575-98b8-15e0710af34d, 4f39ec35-9f6e-421b-94c9-91e2de32015d, 22b4c79a-27e3-4c8e-978c-c8f85c5dd695,
  69894ffd-c7cb-4e92-9423-8c9f66101741, 4c993324-f59f-4c1e-9e32-8e63b3325479, 18a90346-b758-492c-b0c2-921e0e5176e7,
  3fabf6f5-646c-4019-b7b8-043f2b6440f4, 3999dc32-0572-490e-9873-23259cf6a91e, cecce50b-ea2c-44da-bc7b-610cffa56172,
  59fb1959-d979-4821-a376-beedab111400, 42876f7b-5ab7-4809-92a8-95b1de18f99c, 3a464d12-91d7-4ed0-b88e-c556e088a821,
  d117f425-6c6e-478e-b9a0-a2c71754b2da, 7076e76d-fb57-4ab2-b0d7-81fff7e1b300, cb013a0a-92de-4741-8f2f-3b8a5fe87d9f,
  4a7ad181-4ec8-44aa-8e32-dce9784fb177, fdc3777a-0bc3-48ff-ba11-bdde1de03140, 3816520e-86b8-46b0-8507-a40f7ea81e5f,
  68ed02f3-fc91-4e2a-9ea6-33920cd6f6f4, ee309bc1-acf2-4cd7-ba4c-66918dcac234, 2ba32c76-4f47-4952-9932-f1174f305fd3,
  ef157fc4-fad2-461e-90a0-8bd11820a669, e078a718-52af-47b3-b9d8-9fc22fc0fae8, ded0cd7a-61e6-4d23-ac18-3f5f9d5e0a3d,
  5734df8b-09a6-4f7a-b0dd-db0fcf844e8c, 6110350e-34a6-4155-8bfa-ebdb10e18c33, 5759024e-ede9-4619-a43a-3aa722fac84b,
  0ada6574-13be-4b75-97ed-16feae4a911c, 45f31b84-a6a3-47d1-a675-99aa1530827c, 7de556b0-5050-4946-8811-a35eaf890c12,
  d6f4983d-6653-4709-a772-48f81a8bb1b3]
```
* Example output for the IML map - Edges
```
$rostopic echo /map/edges
header: 
  seq: 762
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: "map"
uuid_src: [2c0b4f91-8780-45ba-93d5-dd60ab3ea61d, 2c0b4f91-8780-45ba-93d5-dd60ab3ea61d, 706b0231-edbb-443e-b80b-330691cdba41,
  706b0231-edbb-443e-b80b-330691cdba41, 23f0fa05-6629-4f0c-a27f-9505006c5fe2, 23f0fa05-6629-4f0c-a27f-9505006c5fe2,
  98aad7a8-2820-4b16-b15d-603f83e1a5da, 98aad7a8-2820-4b16-b15d-603f83e1a5da, a0391f1b-8439-4dcd-b06b-3cbb3f2d7dfd,
  a0391f1b-8439-4dcd-b06b-3cbb3f2d7dfd, a378fbe4-504f-4132-9dd8-68507765de36, b0605c16-a416-4afc-b3c2-821e4fd9d7e5,
  0d9f364e-901b-4845-8d6f-76c1122b97d1, 0d9f364e-901b-4845-8d6f-76c1122b97d1, 4dadd7bd-9471-4c57-9b42-9fa89974de16,
  4dadd7bd-9471-4c57-9b42-9fa89974de16, b51db75b-b22c-466f-8475-26c2ea99b2dc, b51db75b-b22c-466f-8475-26c2ea99b2dc,
  24b267fc-ab92-4a71-93c9-9a84785926cb, 24b267fc-ab92-4a71-93c9-9a84785926cb, b11c9ae9-c10b-4aef-a299-121d58515840,
  dfd95cd4-3009-4d13-8587-cb2658db9888, 604067cb-d2f5-47b5-bfa5-8c1e6021b9c8, 604067cb-d2f5-47b5-bfa5-8c1e6021b9c8,
  f4712b59-dd34-47ab-bc20-4965f9b36e2a, f4712b59-dd34-47ab-bc20-4965f9b36e2a, 8882f309-f497-40df-8cae-b358f53f1524,
  8882f309-f497-40df-8cae-b358f53f1524, ce504224-af83-4d89-be6d-f59d6f114597, ce504224-af83-4d89-be6d-f59d6f114597,
  6028db74-70af-40de-b62b-fc0c113f07ba, f1b3ba92-3bba-47ea-9e8f-a248057bc296, c48e49be-1f97-4218-9870-3b73490945b0,
  c48e49be-1f97-4218-9870-3b73490945b0, ade7bff4-1c5f-4009-8f3c-9d35c57a9936, ade7bff4-1c5f-4009-8f3c-9d35c57a9936,
  f2c90bdc-8325-446a-ae8a-94f1f0bce245, f2c90bdc-8325-446a-ae8a-94f1f0bce245, f63c03ca-f261-4c80-be3a-64f914e1e714,
  f63c03ca-f261-4c80-be3a-64f914e1e714, c10a9f4a-e6ae-4ae7-8eb5-7c2acb09dd89, 173d07bb-2512-4ec0-87e8-b4ba588b2781,
  7dde7e83-e1a7-4e8b-9675-ef7577bdcb86, 7dde7e83-e1a7-4e8b-9675-ef7577bdcb86, 2f529d61-3452-4472-9d6a-4904f61aaea2,
  2f529d61-3452-4472-9d6a-4904f61aaea2, 39ff8173-1e10-488f-99f4-6bac128918b5, 39ff8173-1e10-488f-99f4-6bac128918b5,
  ef72e9b0-f87b-4ba8-af44-1c3ac8613258, ef72e9b0-f87b-4ba8-af44-1c3ac8613258, 44892486-e918-44c3-b42c-9771db99aa8c,
  5abb38f3-a5ce-48c5-a1d2-609ffb791a69, c41a6308-4f6f-4073-871e-91b874d5c81f, c41a6308-4f6f-4073-871e-91b874d5c81f,
  49f57e6d-2474-4937-be38-f395aeaf21aa, 49f57e6d-2474-4937-be38-f395aeaf21aa, e48dae03-49a4-4f7d-9ab8-1e6184f6a575,
  e48dae03-49a4-4f7d-9ab8-1e6184f6a575, b26ededc-2ac5-47b3-928f-3f93f3ae3a4e, b26ededc-2ac5-47b3-928f-3f93f3ae3a4e,
  8364bb0f-00c3-40ed-855c-9033dc960649, c5bdffb1-2e50-42d0-a5be-3f3a4023e7c4, 42c3a87a-68d9-45df-8e4e-091a469b22cb,
  2f31e597-7c8a-4cdf-a4f6-113686a031df, a44f64b3-27c6-4436-a3a2-b49bb07cd974, a44f64b3-27c6-4436-a3a2-b49bb07cd974,
  2fe86d4c-83a7-4180-9618-a6524ebd2511, 2fe86d4c-83a7-4180-9618-a6524ebd2511, 91f1a197-f7b8-4653-9eb3-4c1a52ee21fb,
  4733f7f7-08bc-4678-86f2-e750edc0a510, 4356e371-d349-479b-a9c1-abb22346a6fc, 4356e371-d349-479b-a9c1-abb22346a6fc,
  2189297d-90fa-435f-acd1-63f7af2b5f6b, 2189297d-90fa-435f-acd1-63f7af2b5f6b, 690f55e0-87f6-4c5d-8a56-b4fe367ca62b,
  8bd310ba-9a12-4c5a-8f57-3bef729ed76e, c63be3e6-6a3f-4621-a118-0a216b7c0e0b, c63be3e6-6a3f-4621-a118-0a216b7c0e0b,
  44333768-5fa7-4dad-a870-f7be07f0afa0, 44333768-5fa7-4dad-a870-f7be07f0afa0, 8ff3b6db-241b-4c9a-9b3f-d436fc8b79c5,
  eb5f0148-97ba-45b5-a7a9-d7d0cbdfae26, 766478dd-f712-404a-b374-f0f85796719b, 766478dd-f712-404a-b374-f0f85796719b,
  5c9e1983-243e-4624-9c3c-a0f52932f22f, 5c9e1983-243e-4624-9c3c-a0f52932f22f, f96bef75-78fb-41ba-8f75-f5f8f5db2adf,
  f96bef75-78fb-41ba-8f75-f5f8f5db2adf, 91a2b1da-ec32-475e-b9b6-855972638751, 91a2b1da-ec32-475e-b9b6-855972638751,
  9024fce9-eb5d-48b4-9e70-16a693b8fd32, d7776ffe-ed0f-4368-9434-e7492b65d5b7, 6629a3d6-14a1-4ab7-a6a0-d3bd7f7303e1,
  48ce49bb-1940-4380-b22a-9579e2cd5169, 905e775f-8f6b-4303-84ef-f791ccecb448, 905e775f-8f6b-4303-84ef-f791ccecb448,
  5db5eb57-df39-471a-a950-8b27b2b9963c, 5db5eb57-df39-471a-a950-8b27b2b9963c, ced05dba-4c4d-430e-ae1e-4aa1d979843b,
  d012a1f1-5c07-41c8-a446-4a469898c6a3, 66fc4cee-84f2-42e4-aa7f-3986f49f2c60, 66fc4cee-84f2-42e4-aa7f-3986f49f2c60,
  64d71e60-591c-4002-9cfa-53502082eaac, 64d71e60-591c-4002-9cfa-53502082eaac, 9716ef04-9d33-4874-adfd-fd6867ab6cfb,
  ea0728f5-e93c-48c1-976e-54c262d5d8fa, f8019862-df9e-492f-86cf-ab012cd47fa2, f8019862-df9e-492f-86cf-ab012cd47fa2,
  6f0c1703-4408-4ddd-9bd4-e74d0d57334c, 6f0c1703-4408-4ddd-9bd4-e74d0d57334c, 56cf9ba0-4cf3-4957-b84b-39637fd5e2ae,
  9e7ace98-48ac-4da1-bdcb-311851d578a7, fb832fba-328c-4024-aa62-116b1cb6c6da, fb832fba-328c-4024-aa62-116b1cb6c6da,
  03ea5b5d-ad04-4e9f-a50b-a722057eb7df, 03ea5b5d-ad04-4e9f-a50b-a722057eb7df, 9dc00c84-fcb1-499b-b734-91ee5a295973,
  9dc00c84-fcb1-499b-b734-91ee5a295973, e723997f-6aa3-4868-a71b-56aaed817bca, 271bc16c-85c5-40b7-b4f6-1208d569b08d,
  cbf79ff4-d961-4379-9fc2-0d379b517bd1, e1874ac4-61b4-4238-b1da-6badb3b508ad, 05273595-b5b9-4bd8-8b72-8122d9b79ac8,
  b5ef1bf2-fcea-42d4-bc9e-9f6c6d921ab1, e279f55f-b8d1-4fca-9803-094972f97125, 6405de78-cddd-4fe7-aab1-df11059726e5,
  061eb5fc-c6a5-4575-98b8-15e0710af34d, 061eb5fc-c6a5-4575-98b8-15e0710af34d, 4f39ec35-9f6e-421b-94c9-91e2de32015d,
  4f39ec35-9f6e-421b-94c9-91e2de32015d, 4c993324-f59f-4c1e-9e32-8e63b3325479, 4c993324-f59f-4c1e-9e32-8e63b3325479,
  18a90346-b758-492c-b0c2-921e0e5176e7, 18a90346-b758-492c-b0c2-921e0e5176e7, 3fabf6f5-646c-4019-b7b8-043f2b6440f4,
  3fabf6f5-646c-4019-b7b8-043f2b6440f4, 3999dc32-0572-490e-9873-23259cf6a91e, cecce50b-ea2c-44da-bc7b-610cffa56172,
  cecce50b-ea2c-44da-bc7b-610cffa56172, 59fb1959-d979-4821-a376-beedab111400, 59fb1959-d979-4821-a376-beedab111400,
  42876f7b-5ab7-4809-92a8-95b1de18f99c, 42876f7b-5ab7-4809-92a8-95b1de18f99c, 3a464d12-91d7-4ed0-b88e-c556e088a821,
  7076e76d-fb57-4ab2-b0d7-81fff7e1b300, cb013a0a-92de-4741-8f2f-3b8a5fe87d9f, cb013a0a-92de-4741-8f2f-3b8a5fe87d9f,
  4a7ad181-4ec8-44aa-8e32-dce9784fb177, 4a7ad181-4ec8-44aa-8e32-dce9784fb177, fdc3777a-0bc3-48ff-ba11-bdde1de03140,
  3816520e-86b8-46b0-8507-a40f7ea81e5f, 68ed02f3-fc91-4e2a-9ea6-33920cd6f6f4, 68ed02f3-fc91-4e2a-9ea6-33920cd6f6f4,
  ee309bc1-acf2-4cd7-ba4c-66918dcac234, ee309bc1-acf2-4cd7-ba4c-66918dcac234, 2ba32c76-4f47-4952-9932-f1174f305fd3,
  ef157fc4-fad2-461e-90a0-8bd11820a669, e078a718-52af-47b3-b9d8-9fc22fc0fae8, e078a718-52af-47b3-b9d8-9fc22fc0fae8,
  ded0cd7a-61e6-4d23-ac18-3f5f9d5e0a3d, ded0cd7a-61e6-4d23-ac18-3f5f9d5e0a3d, 5734df8b-09a6-4f7a-b0dd-db0fcf844e8c,
  6110350e-34a6-4155-8bfa-ebdb10e18c33, 5759024e-ede9-4619-a43a-3aa722fac84b, 0ada6574-13be-4b75-97ed-16feae4a911c,
  7de556b0-5050-4946-8811-a35eaf890c12]
uuid_dest: [b0605c16-a416-4afc-b3c2-821e4fd9d7e5, 706b0231-edbb-443e-b80b-330691cdba41, 0d9f364e-901b-4845-8d6f-76c1122b97d1,
  23f0fa05-6629-4f0c-a27f-9505006c5fe2, 4dadd7bd-9471-4c57-9b42-9fa89974de16, 98aad7a8-2820-4b16-b15d-603f83e1a5da,
  b51db75b-b22c-466f-8475-26c2ea99b2dc, a0391f1b-8439-4dcd-b06b-3cbb3f2d7dfd, 24b267fc-ab92-4a71-93c9-9a84785926cb,
  a378fbe4-504f-4132-9dd8-68507765de36, b11c9ae9-c10b-4aef-a299-121d58515840, 0d9f364e-901b-4845-8d6f-76c1122b97d1,
  604067cb-d2f5-47b5-bfa5-8c1e6021b9c8, 4dadd7bd-9471-4c57-9b42-9fa89974de16, f4712b59-dd34-47ab-bc20-4965f9b36e2a,
  b51db75b-b22c-466f-8475-26c2ea99b2dc, 8882f309-f497-40df-8cae-b358f53f1524, 24b267fc-ab92-4a71-93c9-9a84785926cb,
  ce504224-af83-4d89-be6d-f59d6f114597, b11c9ae9-c10b-4aef-a299-121d58515840, 6028db74-70af-40de-b62b-fc0c113f07ba,
  f1b3ba92-3bba-47ea-9e8f-a248057bc296, c48e49be-1f97-4218-9870-3b73490945b0, f4712b59-dd34-47ab-bc20-4965f9b36e2a,
  ade7bff4-1c5f-4009-8f3c-9d35c57a9936, 8882f309-f497-40df-8cae-b358f53f1524, f2c90bdc-8325-446a-ae8a-94f1f0bce245,
  ce504224-af83-4d89-be6d-f59d6f114597, f63c03ca-f261-4c80-be3a-64f914e1e714, 6028db74-70af-40de-b62b-fc0c113f07ba,
  c10a9f4a-e6ae-4ae7-8eb5-7c2acb09dd89, 173d07bb-2512-4ec0-87e8-b4ba588b2781, 7dde7e83-e1a7-4e8b-9675-ef7577bdcb86,
  ade7bff4-1c5f-4009-8f3c-9d35c57a9936, 2f529d61-3452-4472-9d6a-4904f61aaea2, f2c90bdc-8325-446a-ae8a-94f1f0bce245,
  39ff8173-1e10-488f-99f4-6bac128918b5, f63c03ca-f261-4c80-be3a-64f914e1e714, ef72e9b0-f87b-4ba8-af44-1c3ac8613258,
  c10a9f4a-e6ae-4ae7-8eb5-7c2acb09dd89, 44892486-e918-44c3-b42c-9771db99aa8c, 5abb38f3-a5ce-48c5-a1d2-609ffb791a69,
  c41a6308-4f6f-4073-871e-91b874d5c81f, 2f529d61-3452-4472-9d6a-4904f61aaea2, 49f57e6d-2474-4937-be38-f395aeaf21aa,
  39ff8173-1e10-488f-99f4-6bac128918b5, e48dae03-49a4-4f7d-9ab8-1e6184f6a575, ef72e9b0-f87b-4ba8-af44-1c3ac8613258,
  b26ededc-2ac5-47b3-928f-3f93f3ae3a4e, 44892486-e918-44c3-b42c-9771db99aa8c, 8364bb0f-00c3-40ed-855c-9033dc960649,
  c5bdffb1-2e50-42d0-a5be-3f3a4023e7c4, 42c3a87a-68d9-45df-8e4e-091a469b22cb, 49f57e6d-2474-4937-be38-f395aeaf21aa,
  2f31e597-7c8a-4cdf-a4f6-113686a031df, e48dae03-49a4-4f7d-9ab8-1e6184f6a575, a44f64b3-27c6-4436-a3a2-b49bb07cd974,
  b26ededc-2ac5-47b3-928f-3f93f3ae3a4e, 2fe86d4c-83a7-4180-9618-a6524ebd2511, 8364bb0f-00c3-40ed-855c-9033dc960649,
  91f1a197-f7b8-4653-9eb3-4c1a52ee21fb, 4733f7f7-08bc-4678-86f2-e750edc0a510, 2f31e597-7c8a-4cdf-a4f6-113686a031df,
  a44f64b3-27c6-4436-a3a2-b49bb07cd974, 4356e371-d349-479b-a9c1-abb22346a6fc, 2fe86d4c-83a7-4180-9618-a6524ebd2511,
  2189297d-90fa-435f-acd1-63f7af2b5f6b, 91f1a197-f7b8-4653-9eb3-4c1a52ee21fb, 690f55e0-87f6-4c5d-8a56-b4fe367ca62b,
  8bd310ba-9a12-4c5a-8f57-3bef729ed76e, c63be3e6-6a3f-4621-a118-0a216b7c0e0b, 2189297d-90fa-435f-acd1-63f7af2b5f6b,
  44333768-5fa7-4dad-a870-f7be07f0afa0, 690f55e0-87f6-4c5d-8a56-b4fe367ca62b, 8ff3b6db-241b-4c9a-9b3f-d436fc8b79c5,
  eb5f0148-97ba-45b5-a7a9-d7d0cbdfae26, f96bef75-78fb-41ba-8f75-f5f8f5db2adf, 44333768-5fa7-4dad-a870-f7be07f0afa0,
  91a2b1da-ec32-475e-b9b6-855972638751, 8ff3b6db-241b-4c9a-9b3f-d436fc8b79c5, 9024fce9-eb5d-48b4-9e70-16a693b8fd32,
  d7776ffe-ed0f-4368-9434-e7492b65d5b7, 6629a3d6-14a1-4ab7-a6a0-d3bd7f7303e1, 5c9e1983-243e-4624-9c3c-a0f52932f22f,
  48ce49bb-1940-4380-b22a-9579e2cd5169, f96bef75-78fb-41ba-8f75-f5f8f5db2adf, 905e775f-8f6b-4303-84ef-f791ccecb448,
  91a2b1da-ec32-475e-b9b6-855972638751, 5db5eb57-df39-471a-a950-8b27b2b9963c, 9024fce9-eb5d-48b4-9e70-16a693b8fd32,
  ced05dba-4c4d-430e-ae1e-4aa1d979843b, d012a1f1-5c07-41c8-a446-4a469898c6a3, 48ce49bb-1940-4380-b22a-9579e2cd5169,
  905e775f-8f6b-4303-84ef-f791ccecb448, 66fc4cee-84f2-42e4-aa7f-3986f49f2c60, 5db5eb57-df39-471a-a950-8b27b2b9963c,
  64d71e60-591c-4002-9cfa-53502082eaac, ced05dba-4c4d-430e-ae1e-4aa1d979843b, 9716ef04-9d33-4874-adfd-fd6867ab6cfb,
  ea0728f5-e93c-48c1-976e-54c262d5d8fa, f8019862-df9e-492f-86cf-ab012cd47fa2, 64d71e60-591c-4002-9cfa-53502082eaac,
  6f0c1703-4408-4ddd-9bd4-e74d0d57334c, 9716ef04-9d33-4874-adfd-fd6867ab6cfb, 56cf9ba0-4cf3-4957-b84b-39637fd5e2ae,
  9e7ace98-48ac-4da1-bdcb-311851d578a7, 9dc00c84-fcb1-499b-b734-91ee5a295973, 6f0c1703-4408-4ddd-9bd4-e74d0d57334c,
  e723997f-6aa3-4868-a71b-56aaed817bca, 56cf9ba0-4cf3-4957-b84b-39637fd5e2ae, 93be216d-9b42-44b7-8103-5807bd34df42,
  271bc16c-85c5-40b7-b4f6-1208d569b08d, cbf79ff4-d961-4379-9fc2-0d379b517bd1, 03ea5b5d-ad04-4e9f-a50b-a722057eb7df,
  e1874ac4-61b4-4238-b1da-6badb3b508ad, 9dc00c84-fcb1-499b-b734-91ee5a295973, 05273595-b5b9-4bd8-8b72-8122d9b79ac8,
  e723997f-6aa3-4868-a71b-56aaed817bca, 93be216d-9b42-44b7-8103-5807bd34df42, b5ef1bf2-fcea-42d4-bc9e-9f6c6d921ab1,
  e1874ac4-61b4-4238-b1da-6badb3b508ad, 05273595-b5b9-4bd8-8b72-8122d9b79ac8, e279f55f-b8d1-4fca-9803-094972f97125,
  6405de78-cddd-4fe7-aab1-df11059726e5, 061eb5fc-c6a5-4575-98b8-15e0710af34d, 69894ffd-c7cb-4e92-9423-8c9f66101741,
  3fabf6f5-646c-4019-b7b8-043f2b6440f4, 4f39ec35-9f6e-421b-94c9-91e2de32015d, 3999dc32-0572-490e-9873-23259cf6a91e,
  22b4c79a-27e3-4c8e-978c-c8f85c5dd695, 59fb1959-d979-4821-a376-beedab111400, 18a90346-b758-492c-b0c2-921e0e5176e7,
  42876f7b-5ab7-4809-92a8-95b1de18f99c, 3fabf6f5-646c-4019-b7b8-043f2b6440f4, 3a464d12-91d7-4ed0-b88e-c556e088a821,
  3999dc32-0572-490e-9873-23259cf6a91e, d117f425-6c6e-478e-b9a0-a2c71754b2da, cb013a0a-92de-4741-8f2f-3b8a5fe87d9f,
  59fb1959-d979-4821-a376-beedab111400, 4a7ad181-4ec8-44aa-8e32-dce9784fb177, 42876f7b-5ab7-4809-92a8-95b1de18f99c,
  fdc3777a-0bc3-48ff-ba11-bdde1de03140, 3a464d12-91d7-4ed0-b88e-c556e088a821, d117f425-6c6e-478e-b9a0-a2c71754b2da,
3816520e-86b8-46b0-8507-a40f7ea81e5f, 68ed02f3-fc91-4e2a-9ea6-33920cd6f6f4, 4a7ad181-4ec8-44aa-8e32-dce9784fb177,
  ee309bc1-acf2-4cd7-ba4c-66918dcac234, fdc3777a-0bc3-48ff-ba11-bdde1de03140, 2ba32c76-4f47-4952-9932-f1174f305fd3,
  ef157fc4-fad2-461e-90a0-8bd11820a669, e078a718-52af-47b3-b9d8-9fc22fc0fae8, ee309bc1-acf2-4cd7-ba4c-66918dcac234,
  ded0cd7a-61e6-4d23-ac18-3f5f9d5e0a3d, 2ba32c76-4f47-4952-9932-f1174f305fd3, 5734df8b-09a6-4f7a-b0dd-db0fcf844e8c,
  6110350e-34a6-4155-8bfa-ebdb10e18c33, 5759024e-ede9-4619-a43a-3aa722fac84b, ded0cd7a-61e6-4d23-ac18-3f5f9d5e0a3d,
  0ada6574-13be-4b75-97ed-16feae4a911c, 5734df8b-09a6-4f7a-b0dd-db0fcf844e8c, 45f31b84-a6a3-47d1-a675-99aa1530827c,
  7de556b0-5050-4946-8811-a35eaf890c12, 0ada6574-13be-4b75-97ed-16feae4a911c, 45f31b84-a6a3-47d1-a675-99aa1530827c,
  d6f4983d-6653-4709-a772-48f81a8bb1b3]
name: [edge_10_19, edge_10_11, edge_11_20, edge_11_12, edge_12_21, edge_12_13, edge_13_22,
  edge_13_14, edge_14_23, edge_14_15, edge_15_24, edge_19_20, edge_20_29, edge_20_21,
  edge_21_30, edge_21_22, edge_22_31, edge_22_23, edge_23_32, edge_23_24, edge_24_33,
  edge_26_35, edge_29_38, edge_29_30, edge_30_39, edge_30_31, edge_31_40, edge_31_32,
  edge_32_41, edge_32_33, edge_33_42, edge_35_44, edge_38_47, edge_38_39, edge_39_48,
  edge_39_40, edge_40_49, edge_40_41, edge_41_50, edge_41_42, edge_42_51, edge_44_53,
  edge_47_56, edge_47_48, edge_48_57, edge_48_49, edge_49_58, edge_49_50, edge_50_59,
  edge_50_51, edge_51_60, edge_53_62, edge_56_65, edge_56_57, edge_57_66, edge_57_58,
  edge_58_67, edge_58_59, edge_59_68, edge_59_60, edge_60_69, edge_62_71, edge_65_66,
  edge_66_67, edge_67_76, edge_67_68, edge_68_77, edge_68_69, edge_69_78, edge_71_80,
  edge_76_85, edge_76_77, edge_77_86, edge_77_78, edge_78_87, edge_80_89, edge_85_94,
  edge_85_86, edge_86_95, edge_86_87, edge_87_96, edge_89_98, edge_92_101, edge_92_93,
  edge_93_102, edge_93_94, edge_94_103, edge_94_95, edge_95_104, edge_95_96, edge_96_105,
  edge_98_107, edge_101_102, edge_102_103, edge_103_112, edge_103_104, edge_104_113,
  edge_104_105, edge_105_114, edge_107_116, edge_112_121, edge_112_113, edge_113_122,
  edge_113_114, edge_114_123, edge_116_125, edge_121_130, edge_121_122, edge_122_131,
  edge_122_123, edge_123_132, edge_125_134, edge_128_137, edge_128_129, edge_129_138,
  edge_129_130, edge_130_139, edge_130_131, edge_131_132, edge_134_143, edge_137_138,
  edge_138_139, edge_139_148, edge_143_152, edge_148_157, edge_152_161, edge_157_166,
  edge_157_158, edge_158_167, edge_158_159, edge_164_173, edge_164_165, edge_165_174,
  edge_165_166, edge_166_175, edge_166_167, edge_167_176, edge_172_181, edge_172_173,
  edge_173_182, edge_173_174, edge_174_183, edge_174_175, edge_175_176, edge_179_188,
  edge_181_190, edge_181_182, edge_182_191, edge_182_183, edge_183_192, edge_188_197,
  edge_190_199, edge_190_191, edge_191_200, edge_191_192, edge_192_201, edge_197_206,
  edge_199_208, edge_199_200, edge_200_209, edge_200_201, edge_201_210, edge_206_215,
  edge_208_209, edge_209_210, edge_215_224]
uuid: [35051bf1-1f7d-4748-b2c2-72002a9bcd0a, 81848666-9bdf-480d-a44c-a0219d231800, fd05cca2-6917-4593-9d20-74063ca68a08,
  676234dd-4941-4122-8b97-422a8696c880, 93c368d7-e655-4d99-8a43-7a6dcdecff4b, 90ae4997-5047-4242-991b-9eb8f4c9b5f1,
  ca6662ad-201f-4e58-94b3-63c3040be554, 8e38beb4-5421-42b0-a92e-be1eb3338f2e, 08cb4d85-8255-44bd-8f7a-06256dd0c294,
  95417ce5-b761-4490-8b00-7ccf1481228e, 89f0063d-0857-4462-85ea-e78def4084ca, 9f996f67-757a-4613-908e-86a43168b0ac,
  85586285-4a78-4339-b030-d444b6b27126, 39d3ad5e-bb58-487e-a1aa-26de9cf96b7f, 06a6c9ba-d7bd-45a1-b938-242a0fb69342,
  f467da5d-e77a-4710-ad62-68a8a3579e15, b21cb64c-ff33-4ab8-95e6-92f1e9aa81ba, 96ddb368-07ad-4f7c-ade0-c799b75f8762,
  0f6faaae-f045-49a3-ba3a-191bad253b4b, c8449bd0-85b3-4262-bb2f-a4187f31bf3e, f6c497e9-cd54-46bd-b5e9-f197f798e31a,
  3c79407a-d560-4f21-99dc-8b39ac8016e4, 03b3f962-6efd-40ff-b1ce-4732ffb8a832, 81f5ec28-7d42-4539-98d9-a54c105ff669,
  7a32de79-7485-4b6b-b044-09257989429c, 1262acda-54cc-4758-9dd5-335003609913, 59ad915b-c589-44ef-a155-4f9139f89da9,
  40e363ea-b1d5-4614-b0f6-7ec7d2493e93, 2b4c9fc5-af2d-4d05-94bd-548be28cd374, a63b0ed5-1ce8-468d-ba5a-b493b34177e5,
  cc3243d6-9070-4a77-a659-0c77964d991f, 29f138de-2d19-49d6-b1be-8c5b51736f0b, 9cbcaea3-e64c-4794-9196-443da2d03f4b,
  48de549b-1086-4388-ad03-ab8af152659b, 5b53f64f-ac07-4b48-a496-151e31f66beb, 89ea6041-bee8-4e48-ac8e-81d8b128695d,
  45d040d3-0159-4bd0-9285-f7e61546f7d9, 2ad365ab-268d-4533-b024-61620692dbea, 74ba667b-42aa-43c0-b290-d776b8799e1d,
  2e788a7a-feea-4980-8e0c-cfe2b98573f7, 430b3b6d-41fa-4586-b77d-31342782b02f, d15512c7-a3fd-4d10-bf32-9961bbd97653,
  5f80bb7a-cb7f-4db2-8b08-f48e5fa0aa20, 5ebd74b1-dd91-47d4-b244-542f9d873a92, a0b8649c-8971-4ab0-a711-a3143d3834d4,
  c1fdb94c-7cc6-409a-bc21-3a037c10eb84, ae3e1c40-cc5a-426d-b178-30c744d8f091, 10a8c1cc-9527-4339-b361-f0e499af6226,
  6ace602f-1de0-4578-a13e-21d20ec7baf4, 76873ef6-027f-474d-b636-d8f478d70450, b97dc9ed-21a1-40e8-8b76-b8a396269019,
  62bdfe06-dc9d-4c51-8527-0ad0d31b4d32, fd80a0be-6abb-4eac-8e4b-8cfe22399b93, e79f9b8e-1960-421f-9853-8a0c321bbfa1,
  77b26897-fee3-4844-a787-e3cce73766a2, 80258cd4-fb8e-49ac-a9a5-c1aa35e54ce6, f27fc665-d0c4-4649-a4c6-5678c2a4ce58,
  9aa27b7a-cef0-4ad9-8065-cfede9412829, a1dbde49-dcfa-43f9-8747-a28d474be2e7, ab252960-50b8-4f73-be3f-ff1d2321a9aa,
  ef22a78c-fba6-4f79-ac5a-b1c0a216b483, 2e0bf79f-44a5-4479-ae33-6adb671c97a5, 2e4e6845-15af-4e6c-bd78-74a090d1be68,
  16b41c09-6392-42d4-865c-dd15d7ad52cc, dc7a3155-41c5-4871-812f-e6c1ce577861, 67d9a5da-b80b-4b3b-a603-bb282e0190a8,
  292d922e-a3f8-4099-ab46-53a67375b4c8, f1fcc86f-e8b9-4368-a32d-a0841a48a350, d86666fa-fe32-475e-bc56-2b8c6aec1c1d,
  94033167-708f-41f9-8028-6c57b9433f58, 93e566b2-f4c1-43b1-83c1-ebe0e8b82edf, ec776f4f-d0de-40dc-9148-2043542569f4,
  1df25c52-acd3-47f8-91ae-cb18a8d51ac7, 4fb05070-07fe-4266-b68c-dcbb59a1cddb, b7542b90-60a8-4375-bf2d-f074dd67fd9a,
  a125cca3-58a9-47bc-8f98-6c782153a486, 5035cdc5-0c5e-4770-84db-c041f80c885f, 79891f2e-33f1-435c-836a-c2dd35ce959d,
  e6687c9f-dadd-429f-9b7d-374a0df2db78, ca5f1e34-2e10-4f39-b695-f617fc74ca8d, e70093c2-0ad7-4a5c-bf32-7cea7c71fc1a,
  c9d569fe-8ddd-4032-936b-7eb01d04ddc8, fa5bf47e-bbb0-43f9-908f-bfac700eb767, 9d481300-9b18-4a8d-bf79-3ae0c4ba548e,
  6c48af75-cf99-46b5-82bb-e3e4f52832cd, ddde6574-a9ad-48a4-8617-6c5eec95fd35, edebe688-a802-4e86-82ee-7699c5c7d0fb,
  4edc71d5-cdfe-4b63-9419-dc5a92e6a534, 1dddcda4-3e42-4bbd-a370-9ae59440c25a, f7a17366-9411-40f2-bcd9-5de406fb6c6d,
  80e6a11b-6897-418e-919d-93a24ba1fde9, d6cb5b16-6ab7-4500-b1e2-709d6adfbc9d, 1d3bcdc9-1da6-4a44-9a43-a5045e24587a,
  0b6f6ee2-5ea5-4901-9b38-c1ab49f035bf, 66f5bad9-440e-47db-8c6d-96b87845d7e7, 60d33f56-da95-4fc0-93b2-d8d9859e1501,
  87d9139a-19fb-4c75-83ac-3ed24aa3bff6, 9b00b1e8-9757-4ec4-8f3f-418dd9402d3e, fd07583d-988f-46b2-aa70-5f201a230da8,
  7b7d0ca3-8941-41d8-9c14-f0e56d8c453e, 9b3b194c-3797-409d-9fd0-118e47d3e60e, 66a04c77-7952-4b74-8487-23d10a0e56cb,
  179eedab-9c2f-40a4-8917-6804fc2e1e7f, 6d47b2cc-c9ed-4b04-8984-2c23652f2a9c, e46c6ac0-0a11-4c79-be08-86686a415131,
  fbcc2d91-9bf3-46db-81f1-63e7c9e2b791, 38627158-0a47-4fd2-b25c-1fa9f1a49744, 3855ccd6-346d-45bc-a203-3579c8fd036b,
  a0a3ab72-c9cd-4700-bf6a-8afd1b8f260f, ec28c427-938e-488e-91d7-0b2c13b57d2b, 47335975-03bc-44b0-bc96-32067032cff4,
  50732cf7-3372-46d9-b52b-a8796f959f81, 865f1113-eaa4-49e2-b486-6f21c0ef71a7, 63913c99-aad0-4e43-9bdc-a4dd4820f2ab,
  2172c31f-5612-4bfd-8268-83abf2f1d1b1, f29d5307-2438-4557-8bd6-9aae921a9478, 7a3be7ad-d64e-4fbc-a3b8-08b73865b6ce,
  b630d88b-6021-42e6-bdc9-247617b437e7, 8d1bd4dc-8eb3-4436-916e-02aa5c34f823, 8b04d68d-3f9d-40cf-a20d-20003b9c18f6,
  c246576c-e590-441c-9234-a3c1e487a7f7, ad08aae6-3724-4c6c-8d80-320d12be4793, 58f78ad0-250a-42fe-a1fe-f3b93a439e4b,
  48003598-8398-4a15-b9e9-af214313521a, 0ac92ab3-1f27-46ef-8f6b-cf3a75c63727, a759e8c9-9e84-455c-9c00-a16824682121,
  c27f364f-3b07-44ec-9b6d-d4507ff6bfa7, 230843b2-ec0a-4725-819a-1472f4d0ec6a, 95fd3338-4284-496b-bd29-2c42b9aafe93,
  532cc225-b925-4a91-b929-88ab7fe726c5, 30b21723-99bb-4bb5-8b3c-819e3c4cf9de, 00597eeb-fd2c-4a59-ac16-85ab66a69321,
  c5b7d170-e9a8-4dd4-a34d-166478e64632, 89f5dc92-09b1-4887-a39d-f21a73e357b7, 48925c29-848e-4adf-82e2-6031821772e7,
  be782dfb-23d1-40a2-8bb9-82a7d5f038cc, 0cb89563-6b4d-4c48-8737-a37537142f07, 19782fd2-9f28-48fe-8dbf-fbdfc050454c,
  e5d07871-e5cd-4057-88a0-ab4c80ba956c, 2fbea29c-50d2-4adc-8c88-c22983e84d9d, 7c8ba745-8bc2-4a2c-8b57-7e20d85cdd15,
  ee1eb5e5-19a5-4a36-9866-ec1e4b4e1462, c8966e76-67c1-4b97-a58d-4d73bc0b6713, 350ba8e7-172e-44dc-b2c8-888c3d0354c4,
  29853318-8134-442b-b45d-e4b013bfc2a8, 16b638eb-7b23-4bcf-b31c-312ca5b0ead1, 09359e40-3a7b-4401-b525-de7f4f2c5db2,
  76e9c026-0232-4790-a2e3-bea08e204903, d4c3e35f-c82a-416f-a0d0-5b602545a09a, 05f51ce9-d666-482f-a55e-c4079725a23b,
  064dd852-c597-47b8-91de-e2c0d67635c7, 8253ffb6-79f7-4784-8a94-741939c17ae8, ca4bcd6b-462c-4ae7-9520-9450fc8537b7,
  bde9c5db-d46e-4b9c-b2a4-03bca96fe24f, ebce7947-71ff-49ee-a77c-d9c7d70b924a, b7b1233a-348d-4bed-ae5d-17bf54bb573a,
  72ade897-d587-4f5c-b92d-25981fe26f65, 7cf757c0-69f3-408f-9089-8e33c8de6991, 40bea417-8610-4ed9-af7a-fe46b0da2731,
  35cddfc8-dfff-405b-b84b-6733ae162ea6, 19d6474c-a9a2-465e-bef7-2331f4c5ac30, 25a6af8b-2171-4490-97db-97a10b7ded0a,
  ffad7313-8d70-4b63-a0b4-4d16d53a8631, 8019a4f2-b816-4b14-a7c1-c9e7cb6e936b, 85a5d25d-1172-471d-97d9-c7c45e5ce0eb,
  0a89a43e-75db-4e1f-9a36-3d6d4829c51e]
```
* Example output for the IML map - Pose with covariance
```
$rostopic echo /robot_opil_v1/pose_channel 
header: 
  seq: 76
  stamp: 
    secs: 36
    nsecs: 200000000
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: 15.9636536984
      y: 13.3448596722
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.718399874794
      w: 0.695630375915
  covariance: [0.23681326809196435, -0.0016609806341989497, 0.0, 0.0, 0.0, 0.0, -0.0016609806341989497, 0.23012575436405314, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06503792236623226]
```



