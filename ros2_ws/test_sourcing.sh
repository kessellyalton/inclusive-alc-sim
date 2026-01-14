#!/bin/bash
cd ~/dev/inclusive-alc-sim/ros2_ws
source /opt/ros/jazzy/setup.bash 2>&1 | grep -v "no such file" | grep -v "can't open file" || true
source install/local_setup.bash 2>&1 | grep -v "no such file" | grep -v "can't open file" || true
echo "Testing packages..."
ros2 pkg list | grep alc
