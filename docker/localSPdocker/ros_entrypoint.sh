#!/bin/bash
set -e

echo SP Local : OPIL version 2 is now running
# firos setup
sed -e "s/LOCALHOST/$HOST/g" -e "s/FIWAREHOST/$FIWAREHOST/g" -e "s/NETINTERFACE/$NETINTERFACE/g" /root/catkin_ws/src/firos/config/config.json.template > /root/catkin_ws/src/firos/config/config.json

if [ $SIMULATION == true ]
        then 
                echo "without RAN but with simulator Stage"
				echo "sleeping"
				sleep 5
				echo "continuing"
        else
				echo "sleeping"
				sleep 5
				echo "continuing"
				export ROS_MASTER_URI=http://ran:11311
				export ROS_IP=splocal
				echo $ROS_MASTER_URI
fi

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin_ws/devel/setup.sh"

if [ $SIMULATION == true ]
        then 
                echo "with simulator Stage"
                exec roslaunch sensing_and_perception local_robot_sim.launch
        else
                echo "without simulator"
                exec roslaunch sensing_and_perception local_robot.launch
fi


exec  "$@"
