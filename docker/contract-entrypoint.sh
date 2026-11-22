#!/usr/bin/env bash
# The Ouster driver image's environment contract (METROLLA_DRIVER_CONTRACT=1;
# docs/METROLLA_DRIVER_CONTRACT.md).
#
# /entrypoint.sh execs this script when METROLLA_DRIVER_CONTRACT is set and not "0". It sources
# ROS 2 Jazzy and the driver workspace, then hands over to ouster_env.py, which checks the
# environment (refusing to start on any value it does not understand), writes the driver's
# parameter file under /tmp and execs the os_driver node with it, so the node is the process that
# receives SIGTERM.
set -eo pipefail

# setup.bash reads unset variables; it must not run under `set -u`.
# shellcheck source=/dev/null
source "/opt/ros/jazzy/setup.bash"
# shellcheck source=/dev/null
source "/opt/ros2_ws/install/setup.bash"

exec python3 -I /opt/metrolla/ouster/ouster_env.py "$@"
