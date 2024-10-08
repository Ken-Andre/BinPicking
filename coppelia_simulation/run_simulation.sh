#!/bin/bash
# Source ROS2 setup
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source ~/ros2_ws/install/setup.bash  # Only if you have a custom workspace

# Run the Python script
python3 /home/ken/Documents/BinPicking/coppelia_simulation/simulation.py
