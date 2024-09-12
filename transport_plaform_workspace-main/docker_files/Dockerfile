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

RUN apt-get update && \
    apt-get install -y python3 python3-pip


# Настройка tmux
RUN touch ~/.tmux.conf && \
    echo "set -g mouse on" >> ~/.tmux.conf

# Установка необходимых бибилиотек
RUN pip install pyserial crc8


# Инициализация среды ROS
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/devel/setup.bash" >> ~/.bashrc

# Стандартная команда или точка входа
CMD ["bash"]
