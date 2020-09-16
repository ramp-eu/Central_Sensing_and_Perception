#FROM osrf/ros:kinetic-desktop-full
#FROM fhgiml130/firos:latest
FROM osrf/ros:melodic-desktop

RUN apt-get update && apt-get install -y \    
    ros-melodic-rviz \
    ros-melodic-navigation \
    ros-melodic-industrial-msgs \
	    git\
            python-requests			

RUN apt update && apt upgrade -y

COPY ./src /catkin_ws/src
#COPY ./docker/centralSPdocker/src /catkin_ws/src
RUN cd /catkin_ws/src && \
	git clone -b v0.2.0 --recursive https://github.com/iml130/firos.git

# Build
WORKDIR /catkin_ws

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; . /catkin_ws/devel/setup.bash; catkin_make'

# Add ROS setup bash to .bashrc
RUN echo 'source /opt/ros/melodic/setup.bash \n source /catkin_ws/devel/setup.bash' >> /root/.bashrc

# Clean up
RUN rm -rf /var/lib/apt/lists/*

COPY ./docker/centralSPdocker/FIROS_config_centralSPgraph /catkin_ws/src/firos/config

COPY ./docker/centralSPdocker/ros_entrypoint.sh /

WORKDIR /

RUN chmod +x /ros_entrypoint.sh
