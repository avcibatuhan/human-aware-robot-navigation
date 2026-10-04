FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-cv-bridge \
    ros-jazzy-message-filters \
    ros-jazzy-tf2-ros \
    ros-jazzy-rosbag2-storage-mcap \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    clang-format \
    && rm -rf /var/lib/apt/lists/*

# ultralytics and deep-sort-realtime are not ROS packages: install them in a
# venv that still sees the system rclpy / cv_bridge.
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt

ENV TURTLEBOT3_MODEL=waffle
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc \
    && echo '[ -f /ws/ros2_ws/install/setup.bash ] && source /ws/ros2_ws/install/setup.bash' >> /etc/bash.bashrc

WORKDIR /ws
CMD ["bash"]
