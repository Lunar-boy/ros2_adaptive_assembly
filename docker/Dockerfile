FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WS=/workspaces/ros2_adaptive_assembly_ws



# System and ROS development dependencies.
#
# This image is intentionally headless:
# - no RViz2 GUI setup
# - no Gazebo GUI setup
# - no NVIDIA runtime setup
# - no camera or real robot device passthrough
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    curl \
    git \
    sudo \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep cache. Ignore failure because some official images may
# already have rosdep initialized.
RUN rosdep init 2>/dev/null || true && \
    rosdep update --rosdistro ${ROS_DISTRO} || true

RUN mkdir -p ${WS}/src

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


WORKDIR ${WS}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
