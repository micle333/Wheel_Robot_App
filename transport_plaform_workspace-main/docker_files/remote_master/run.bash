#!/bin/bash

SIM_ROOT="$( cd ../../"$(dirname "${BASH_SOURCE[0]}" )" && pwd )"

xhost +local:docker > /dev/null || true

IMG_NAME="kortium/ros_dev_sc_2024"

### DOCKER RUN ----------------------------------------------------------- #

docker run  -d -ti --rm \
            -e "DISPLAY" \
            -e "QT_X11_NO_MITSHM=1" \
            -e "ROS_HOSTNAME=192.168.88.254" \
            -e "ROS_IP=192.168.88.254" \
            -e "ROS_MASTER_URI=http://192.168.88.254:11311/" \
            -e XAUTHORITY \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v /etc/localtime:/etc/localtime:ro \
            -v ${SIM_ROOT}/workspace:/workspace \
            --net=host \
            --privileged \
            --name "dev_sc_2024_remote_master" ${IMG_NAME} \
            > /dev/null
