#!/usr/bin/env bash
set -Eeuo pipefail

: "${ROBOT_IP:?ROBOT_IP is required}"
: "${HOST_IP:?HOST_IP is required}"

export ROS_IP="$HOST_IP"
export ROS_MASTER_URI="http://${ROBOT_IP}:11311"
source /opt/ros/noetic/setup.bash
source /opt/ros_ws/devel/setup.bash
export PYTHONPATH=/opt/sawyer-control/bridge:${PYTHONPATH:-}

exec python3 -m sawyer_control_bridge.server
