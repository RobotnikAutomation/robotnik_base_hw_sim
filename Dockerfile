ARG base_image="robotnik/ros"
ARG ros_distro="jazzy"
ARG image_base_version="0.6.1"
ARG ros_mirror="ros.mirror.robotnik.ws"

FROM ${base_image}:${ros_distro}-builder-${image_base_version} AS builder

ENV DEBIAN_FRONTEND=noninteractive

USER root

# Install compiled packages
RUN --mount=type=bind,\
target=/tmp/requirements.txt,\
source=dependencies/requirements/builder/packages.txt \
    true \
    && if \
        timeout 2 curl -IsS http://${ros_mirror} &>/dev/null; \
        then \
        sed -i \
            "s#packages.ros.org#${ros_mirror}#" \
            /etc/apt/sources.list.d/ros-latest.list ;\
        fi \
    && apt-fast update \
    && apt-fast install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && true

USER ${USER_NAME}

RUN --mount=type=bind,\
source=./dependencies/repos/common.repo.yml,\
target=/tmp/common.repo.yml,ro \
        vcs import \
        --input /tmp/common.repo.yml  \
        --shallow

COPY robotnik_base_hw_sim src/robotnik_base_hw_sim

# Generate deb packages
RUN generate_debs.sh

WORKDIR /home/robot/robot_ws/debs
# Generate Packages.gz
RUN dpkg-scanpackages . | gzip -9c > Packages.gz

FROM ${base_image}:${ros_distro}-base-${image_base_version} AS base

USER root
# Install compiled packages and dependencies
RUN \
    --mount=\
type=bind,\
from=builder,\
source=/home/robot/robot_ws/debs,\
target=/tmp/debs \
    --mount=\
type=bind,\
target=/tmp/requirements.txt,\
source=dependencies/requirements/base/packages.txt \
    true \
    && if \
        timeout 2 curl -IsS http://${ros_mirror} &>/dev/null; \
        then \
        sed -i \
            "s#packages.ros.org#${ros_mirror}#" \
            /etc/apt/sources.list.d/ros-latest.list ;\
        fi \
    && echo "deb [trusted=yes] file:///tmp/debs/ ./" | tee /etc/apt/sources.list.d/debs.list \
    && apt-get update \
    && apt-fast install -q -y \
        --no-install-recommends \
        $(eval "echo $(cat /tmp/requirements.txt | xargs)") \
    && apt-get clean -q -y \
    && apt-get autoremove -q -y \
    && rm -rf /var/lib/apt/lists/* \
    && rm /etc/apt/sources.list.d/debs.list \
    && true

USER ${USER_NAME}

# The image is built to run gazebo ignition by default if no other setup is provided.
ENV STARTUP_TYPE="launch"
ENV ROS_BU_PKG="robotnik_base_hw_sim"
ENV ROS_BU_LAUNCH="robotnik_io_controller_sim.launch.py"
