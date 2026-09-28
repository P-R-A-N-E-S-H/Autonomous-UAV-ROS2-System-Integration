# ==============================================================================
# Dockerfile: Autonomous UAV System Integration Environment (ROS 2 Jazzy)
# Author: Member 3 (ROS 2 & System Integration Engineer)
# ==============================================================================
FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

# Install Core Robotics & Integration Tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    git \
    tmux \
    htop \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib Datasets for MAVROS
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh || true

# Set up Workspace
WORKDIR /ros2_ws

# Copy workspace source
COPY drone_ws/src /ros2_ws/src

# Install ROS dependencies
RUN source /opt/ros/jazzy/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build Workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source environment automatically in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
