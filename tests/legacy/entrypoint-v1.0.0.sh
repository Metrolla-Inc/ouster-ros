#!/bin/bash
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
