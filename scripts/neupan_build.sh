#!/bin/bash

# Activate NeuPAN virtual environment and set PYTHONPATH
source ~/temporary/ros2_ws/neupan_env/bin/activate
export PYTHONPATH=$PYTHONPATH:~/temporary/ros2_ws/neupan_env/lib/python3.10/site-packages

# Build only the AI packages
colcon build \
  --packages-select neupan_nav2_controller \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Deactivate the environment
deactivate 2>/dev/null || true

# Clean PYTHONPATH
PYTHONPATH=$(echo $PYTHONPATH | tr ':' '\n' | grep -v "neupan_env" | tr '\n' ':')