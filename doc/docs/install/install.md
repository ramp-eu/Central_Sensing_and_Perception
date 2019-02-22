# From Scratch

Install Firos:

```git clone https://github.com/Ikergune/firos.git```

Install ROS packages:

```sudo apt-get install ros-kinetic-navigation```

```sudo apt-get install ros-kinetic-gmapping```

```sudo apt-get install ros-kinetic-teleop-twist-keyboard```

Install from SourceCode:

* put all from src folder to your src folder of your catkin workspace or create a new one by typing catkin_init_workspace in src folder. Then compile it with catkin_make in one folder up.
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
#S&P
    sp:
        restart: always
        image: l4ms/opil.sw.sp:2.0
#        links:
#            - orion
        volumes:
            #- path on the host : path inside the container
            - /tmp/.X11-unix:/tmp/.X11-unix:rw
#            - ./annotations.ini:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/annotations.ini:ro
#            - ./CHEMI.yaml:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.yaml:ro
#            - ./map.png:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/map.png:ro
#            - ./topology.launch:/root/catkin_ws/src/mod.sw.sp/src/maptogridmap/launch/topology.launch:ro
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
You should see:
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
To use arbitrary annotations file and map file, go to the folder test/docker_compose_files/Central_SP_docker, where you can find an example of docker-compose.yml that starts everything on a local machine, the file with annotations - annotations.ini, the map files - map.png and CHEMI.yaml, and also a topology.launch if you want to change the size of the grid cell for calculating the topology. To use these files, first uncomment the lines in the docker-compose.yml with these files, and put the files in the same folder where you have your docker-compose.yml file. The files CHEMI.yaml and map.png can have arbitrary names, but do not change the image line in yaml file:
```
#CHEMI.yaml
image: map.png
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.165
free_thresh: 0.001
```
The map file is treated as a grayscaled image, and you can adjust occupied_tresh and free_tresh to different values, depending from which shade of gray you want it to be considered as obstacle.

