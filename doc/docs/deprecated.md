Deprecated features are features that SP stil supports but that are not maintained or evolved any longer, or will be used in the future. In particular:


# Sending larger data on demand - a service mockup
Since there is no service call supported yet in firos, topic _do_serve_ is used as a service mockup. On another machine that wants to obtain the data, on topic _do_serve_ needs to be sent value "true" or 1. Large data are a) map topic created with _map_server_ from PNG or PGM file and b) gridmap topic created from the map by resampling to cells of size given by the parameter _cell_size_.

## Testing sending map topic on request on machine_1:
Remark: only a small map can be tested here around 10m x 7m - Andamapa.yaml.
To test the sending of map topic the _mux_topicsmap.py_ program needs to be started which sends the map on the whitelisted topic only when master requests.
Start all these steps:
```
terminal 1: roslaunch maptogridmap startmapserver.launch
terminal 2: roslaunch maptogridmap startmaptogridmap.launch
terminal 3: rosrun firos core.py (put in firos/config all json files from OPIL/config_files/machine_1)
```
(the next two steps are optional, e.g., it will work without terminal 4 and 5) 
```
terminal 4: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
terminal 5: rosrun sensing_and_perception pubPoseWithCovariance
terminal 6: rosrun maptogridmap mux_topicsmap.py
terminal 7: rostopic pub /map/do_serve std_msgs/Bool '{data: 1}'
```
Refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities.
There should be under id "map" the topic "realmap".

## Testing sending gridmap topic on request:
To test the sending of gridmap topic the _mux_topics.py_ program needs to be started which sends the gridmap on the whitelisted topic only when master requests.
Start all previous steps (terminal 1, ..., terminal 7), but it works with starting only terminals 1,2,3.
```
terminal 8: rosrun maptogridmap mux_topics.py
terminal 9: rostopic pub /map/do_serve std_msgs/Bool '{data: 1}'
```
Refresh firefox on http://OPIL_SERVER_IP:1026/v2/entities.
There should be under id "map" the topic "realtopology".


## Testing if topics are received on machine 2 through firos
```
terminal 1: roscore
terminal 2: rosrun firos core.py (put in firos/config all json files from OPIL/config_files/machine_2)
```
_maptogridmap_ package needs to be on the machine_2 - it is not important that the source code is in there but only that msg files and CMakeLists.txt compiling them are there.
Now you are able to echo these ros topics:
```
rostopic echo /map/realtopology (if test sending gridmap is on)
rostopic echo /map/realmap (if test sending map is on)
```

