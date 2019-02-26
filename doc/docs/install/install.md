# From Scratch

If you are using older version of Firos (v1) use this:

```git clone https://github.com/Ikergune/firos.git```

Right now, all prepared config files are for Firos v1. Update to Firos v2 will come soon.
To install Firos v2:

```git clone --recursive https://github.com/iml130/firos.git```

To change all config files to work with Firos v2 you need to replace some variables in config.json to numbers (check the folowing lines):

```
          "throttling": 3, 
          "subscription_length": 300,
          "subscription_refresh_delay": 0.9
```

Install ROS packages:

```sudo apt-get install ros-kinetic-navigation```

```sudo apt-get install ros-kinetic-gmapping```

```sudo apt-get install ros-kinetic-teleop-twist-keyboard```

Install from SourceCode:

* put everything to your src folder of your catkin workspace or create a new one by typing catkin_init_workspace in src folder. Then compile it with catkin_make in one folder up.
```
cd ..
catkin_make
```

# Docker

There are two docker containers - the Central SP and the Local SP.

## The Central SP docker

The docker container is located at 
<https://hub.docker.com/r/l4ms/opil.sw.sp>

To start a docker file prepare a docker-compose.yml following this example:

```
version: "3"
services:      
    #Context Broker
    orion:        
        image: fiware/orion
        ports:
            - 1026:1026
        command: 
            -dbhost mongo
    mongo:
        restart: always
        image: mongo:3.4
        command: --nojournal    
#S&P
    sp:
        restart: always
        image: l4ms/opil.sw.sp:2.0
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
#            - ./annotations.ini:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/annotations.ini:ro
#            - ./map.yaml:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.yaml:ro
#            - ./map.png:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.png:ro
#            - ./topology.launch:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/topology.launch:ro
        environment:
            - FIWAREHOST=orion
            - HOST=sp
            - NETINTERFACE=eth0
            - DISPLAY=$DISPLAY
```
This example uses the version 3 and it does not need links to enable services to communicate. To update the docker-compose to the working version (1.22) type:
```
sudo curl -L https://github.com/docker/compose/releases/download/1.22.0/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
```
To check the version type:
```
sudo docker-compose --version
```
You should see something similar to:
```
docker-compose version 1.22.0, build f46880fe
```
Then, start it from the folder where you put your docker-compose.yml file:
```
xhost local:root (call this only once - this is for display)
sudo docker-compose up
```
You should see the entities in, e.g. firefox, at the address <http://localhost:1026/v2/entities>.

This docker container starts the creation of topology (nodes, edges) from the given map file and with included annotations from the annotation file. As a default, IML lab will be started with 4 annotations, as is the case in this example, where usage of the files is commented.

![IML topology](../img/IMLtopology.png)

To use arbitrary annotations file and map file you should create these files and put it in the same folder next to the docker-compose.yml and uncomment the lines containing these files. These example files can be found in the folder test/docker_compose_files/Central_SP_docker.

Here is a quick guide how to create these files:

* To use arbitrary annotations create the annotations.ini file:
```
#annotations.ini
[P1]
# coordinates
point_x = 17.96
point_y = 6.57
theta = -90
distance = 1.8
```
where point_x, point_y are coordinates of the annotation, and theta and distance determine the topology node where AGV will stop distanced from the annotation coordinates and with orientation pointing to the annotation coordinates.

* Uncomment the line in the docker-compose.yml:
```
            - ./annotations.ini:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/annotations.ini:ro
```
After restarting docker-compose.yml this is what should be the result:

![IML topology](../img/IMLtopology.png)

* To use arbitrary map file you should prepare map.png (export as png in any graphical software) and map.yaml. Here is an example of the CHEMI factory floorplan saved as png file:

![IML topology](../img/IMLtopology.png)

It should be a greyscaled image, where dark grey will be considered as obstacle, while light grey as free area.

* To set the dimensions of the map file you need to prepare parameter file map.yaml, where you need to set the resolution of the map, and thresholds for defining what will be considered as obstacle, and what as free area. Here is an example for the CHEMI factory:
```
image: map.png
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.165
free_thresh: 0.001
```
Here, the line ```image: map.png``` should not be changed ever because on the docker side the name map.png will always be used, while the file names can be arbitrary. The parameter ```resolution: 0.05``` means the size of the pixel of the png file is 0.05 m wide. Adjust the parameters occupied_thresh and free_thresh to different values, depending on which shade of grey should be considered as occupied. In this example, thresholds are quite low, which means almost only white is considered as free area.

* Uncomment the lines in the docker-compose.yml:
```
            - ./map.yaml:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.yaml:ro
            - ./map.png:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.png:ro
```
After restarting docker-compose.yml this is what should be the result:

![IML topology](../img/IMLtopology.png)

It can be seen that here the topology nodes are too rare and we are missing some of them in narrow passages. In the following we will change the size of the grid cell to 0.5 m and have dense topology nodes:
 
* To change the size of the grid cell for calculating the topology prepare topology.launch as follows:
```
<launch>
<node name="map_server" pkg="map_server" type="map_server" args="$(find maptogridmap)/launch/map.yaml" respawn="false" >
<param name="frame_id" value="/map" />
</node>
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find maptogridmap)/singlerobot.rviz" /> 
<node name="map2gm" pkg="maptogridmap" type="map2gm" output="screen">
        <param name="cell_size" type="double" value="0.5" />
        <param name="annotation_file" textfile="$(find maptogridmap)/launch/annotations.ini" />
</node>
    <!-- Run FIROS -->
    <node name="firos" pkg="firos" type="core.py" />
</launch>
```
where we put the parameter cell_size to 0.5 m.

* Uncomment the line in the docker-compose.yml:
```
            - ./topology.launch:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/topology.launch:ro
```
After restarting docker-compose.yml this is what should be the result:

![IML topology](../img/IMLtopology.png)


## The Local SP docker

is not yet ready. Will come soon!

