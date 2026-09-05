# ROS 2 Humble development/runtime image for the Tactile-UMI stack.
# Build:  docker compose -f docker/docker-compose.yml build
# Shell:  docker compose -f docker/docker-compose.yml run --rm ros
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=humble \
    PIP_ROOT_USER_ACTION=ignore \
    PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rosbag2 \
      ros-humble-rosbag2-storage-mcap \
      ros-humble-cv-bridge \
      ros-humble-diagnostic-msgs \
      ros-humble-launch-ros \
      ros-humble-ament-flake8 \
      ros-humble-ament-copyright \
      python3-opencv \
      python3-serial \
      python3-numpy \
      python3-pytest \
      python3-pip \
      python3-colcon-common-extensions \
      v4l-utils \
      cmake \
      build-essential \
    && rm -rf /var/lib/apt/lists/*

# Offline MCAP tooling (also used by tools/sync_check.py).
RUN pip3 install --no-cache-dir "mcap>=1.1" "mcap-ros2-support>=0.5"

WORKDIR /ws
RUN mkdir -p /data/sessions

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
