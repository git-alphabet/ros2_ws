#!/bin/bash

# Build the ROS workspace skipping NeuPAN and neupan_nav2_controller
colcon build --parallel-workers 1 --packages-skip neupan_nav2_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Activate NeuPAN virtual environment and set PYTHONPATH
source ~/temporary/ros2_ws/neupan_env/bin/activate
python3 -m pip install -q "numpy<2" || true
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