#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
# Build against the system ROS installation, independent of older workspaces.
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
if [[ -f /opt/ros/melodic/setup.bash ]]; then
    source /opt/ros/melodic/setup.bash
else
    source /opt/ros/noetic/setup.bash
fi
catkin_make -j2 -l2 -DCATKIN_WHITELIST_PACKAGES= -DCATKIN_ENABLE_TESTING=OFF \
    -DPYTHON_EXECUTABLE="$(command -v "python${ROS_PYTHON_VERSION}")" "$@"
