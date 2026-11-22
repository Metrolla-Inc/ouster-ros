#!/bin/bash
# Metrolla driver contract, opt-in (docs/METROLLA_DRIVER_CONTRACT.md). Only when
# METROLLA_DRIVER_CONTRACT is set and not "0" does the image switch to the environment contract;
# otherwise everything below this block is the v1.0.0 entrypoint, byte for byte (tests/legacy pins it).
if [ -n "${METROLLA_DRIVER_CONTRACT:-}" ] && [ "${METROLLA_DRIVER_CONTRACT}" != "0" ]; then
    exec /opt/metrolla/ouster/contract-entrypoint.sh "$@"
fi
# --- end of the Metrolla contract block; the v1.0.0 entrypoint follows ---
set -e

# Source the global ROS 2 environment
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Source the local workspace environment
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    source "$ROS2_WS/install/setup.bash"
fi

# Execute the launch command with any passed arguments
exec "$@"
