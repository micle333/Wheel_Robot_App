@echo off
SET CONTAINER_NAME=ros_server_sc_2024
SET IMAGE_NAME=kortium/ros_server_sc_2024:latest
SET WORKSPACE=.\workspace

echo Checking container...
docker ps -a --filter "name=%CONTAINER_NAME%" --format "{{.Names}}" | findstr /i %CONTAINER_NAME%
IF %ERRORLEVEL% EQU 0 (
    echo Removeing old container %CONTAINER_NAME%...
    docker stop %CONTAINER_NAME%
    docker rm %CONTAINER_NAME%
)

echo Creating new container %IMAGE_NAME%...
docker pull %IMAGE_NAME%

echo Running %CONTAINER_NAME%...
docker run -d -ti^
    -e "DISPLAY"^
    -e "QT_X11_NO_MITSHM=1"^
    -e "ROS_HOSTNAME=localhost"^
    -e "XAUTHORITY"^
    -v %WORKSPACE%:/workspace^
    -p 8080:8080^
    --privileged^
    --name %CONTAINER_NAME% %IMAGE_NAME%

echo Контейнер %CONTAINER_NAME% запущен.

