#!/bin/bash
set -e

echo SP Central : OPIL version 2 is now running
# firos setup
sed -e "s/LOCALHOST/$HOST/g" -e "s/FIWAREHOST/$FIWAREHOST/g" -e "s/NETINTERFACE/$NETINTERFACE/g" /root/catkin_ws/src/firos/config/config.json.template > /root/catkin_ws/src/firos/config/config.json

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin_ws/devel/setup.sh"

exec roslaunch maptogridmap topology.launch


exec  "$@"
