#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
# Use Ubuntu's Python and tools rather than an active Conda environment.
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
colcon build --symlink-install "$@" --cmake-args \
  -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
