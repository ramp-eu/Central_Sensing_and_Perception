# mod.sw.sp

OPIL module: Central Sensing & Perception (Central SP)
Innovation Centre Nikola Tesla, written by Marija from May 6th 2018, contintributed by Goran, Jura, Petki and Ana

# Creating the docker image

This is the Central SP. The docker image is created by executing in the root folder of this package:

```
docker build -t "centralsp:test" -f docker/centralSPdocker/Dockerfile .
```

Test built docker container by starting `docker-compose up` from the folder `test/docker_compose_files/Central_SP_docker.

# Quick start from source

Documentation about the Central SP module can be found here: <https://opil-documentation.readthedocs.io/en/latest/SP/Central_SP_Getting_Started.html>.

Type `catkin_init_workspace` in the src folder of this repository. Then, compile it with `catkin_make` in one folder up.
```
cd ..
catkin_make
```
Put the `setup.bash` script from the newly created `devel` folder to your `.bashrc` file:

```
source (your path to the repo folder)/mod.sw.sp/devel/setup.bash
```

Open the new terminal tab and command `roscd` should navigate you to the repo folder.

* Start the map by launching the map_server package:
```
terminal 1: roslaunch maptogridmap startmapserver.launch
```

* Start the topology calculation:
```
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
```

