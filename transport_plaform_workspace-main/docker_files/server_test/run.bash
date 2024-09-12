#!/bin/bash

# Настройка директории симуляции
SIM_ROOT="$( cd ../../"$(dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Разрешение доступа к X11 для Docker
xhost +local:docker > /dev/null || true

IMG_NAME="kortium/ros_server_sc_2024"

# Запуск Docker контейнера
docker run -d -ti \
    -e "DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "ROS_HOSTNAME=localhost" \
    -e XAUTHORITY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /etc/localtime:/etc/localtime:ro \
    -v ${SIM_ROOT}/workspace:/workspace \
    -p 8080:8080 \
    --privileged \
    --name "server_sc_2024" ${IMG_NAME}
