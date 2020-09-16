#!/bin/bash
set -e

echo SP Central : OPIL version 3 is now running
# firos setup
sed -e "s/LOCALHOST/$HOST/g" -e "s/FIWAREHOST/$FIWAREHOST/g" /catkin_ws/src/firos/config/config.json.template > /catkin_ws/src/firos/config/config.json

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_ws/devel/setup.sh"

FILEANNOT=/annotations.ini
if test -f "$FILEANNOT"; then
	cp annotations.ini /catkin_ws/src/maptogridmap/launch/
fi
FILETOP=/topology.launch
if test -f "$FILETOP"; then
	cp topology.launch /catkin_ws/src/maptogridmap/launch/
fi
FILEPNG=/map.png
FILEYML=/map.yaml
if test -f "$FILEPNG"; then
	cp map.png /catkin_ws/src/maptogridmap/launch/
	if test -f "$FILEYML"; then
		cp map.yaml /catkin_ws/src/maptogridmap/launch/
		exec roslaunch maptogridmap topology.launch
	fi
fi

echo "please insert a floorplan.png and floorplan.yaml file to begin!"
echo "in your docker-compose.yml put under volumes:"
echo "            - ./floorplan.yaml:/map.yaml:ro"
echo "            - ./floorplan.png:/map.png:ro"

exec  "$@"
