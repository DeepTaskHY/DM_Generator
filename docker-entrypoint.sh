#!/bin/bash
set -e

# Setup catkin workspace
source "/opt/ros/$ROS_DISTRO/setup.sh" && catkin_make
source devel/setup.sh

# Setup ROS environment
echo "source \"/opt/ros/$ROS_DISTRO/setup.sh\"" >> /etc/bash.bashrc
echo "source /workspace/devel/setup.sh" >> /etc/bash.bashrc

# Setup executable files
chmod +x src/scripts/launcher.py

# Execute CMD
exec "$@"
