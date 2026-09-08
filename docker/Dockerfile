FROM ros:noetic-ros-base AS messages

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3-catkin-pkg \
    python3-empy \
    python3-numpy \
    ros-noetic-actionlib-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/ros_ws/src \
    && git clone --depth 1 --branch release-5.2.0 \
        https://github.com/RethinkRobotics/intera_common.git /opt/ros_ws/src/intera_common \
    && /bin/bash -lc 'source /opt/ros/noetic/setup.bash && cd /opt/ros_ws && catkin_make'

FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/sawyer-control
COPY pyproject.toml README.md /opt/sawyer-control/
COPY src /opt/sawyer-control/src
RUN python3 -m pip install --no-cache-dir ".[bridge]"

COPY --from=messages /opt/ros_ws/devel /opt/ros_ws/devel
COPY bridge /opt/sawyer-control/bridge
COPY docker/entrypoint.sh /opt/sawyer-control/docker/entrypoint.sh
RUN chmod 0755 /opt/sawyer-control/docker/entrypoint.sh

ENTRYPOINT ["/opt/sawyer-control/docker/entrypoint.sh"]
