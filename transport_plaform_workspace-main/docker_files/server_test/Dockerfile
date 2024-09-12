# Используйте базовый образ ROS Noetic
FROM osrf/ros:noetic-desktop-full

# Установите переменную окружения для ROS Noetic
ENV ROS_DISTRO=noetic

# Обновите список пакетов и установите необходимые зависимости
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    tmux \
    gazebo11 \
    python3 \
    iputils-ping \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control

# Установка дополнительных пакетов Python
RUN apt-get install -y python3-pip && pip install pyserial crc8

# Инициализация среды ROS
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/devel/setup.bash" >> ~/.bashrc

# Команда для запуска с учётом инициализации среды
CMD ["bash", "-c", "source /etc/bash.bashrc && source /workspace/devel/setup.bash && roslaunch platform_controller_comm server.launch"]
