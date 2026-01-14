# inclusive-alc-sim

inclusive-alc-sim is a ROS 2 Jazzy–based simulation framework for designing and evaluating an inclusive, adaptive robotic tutoring system for low-resource educational contexts.

The project integrates learner modelling, disability-aware adaptivity, and reinforcement-learning-based instructional pacing within a Gazebo simulation environment. It adopts a simulation-first methodology to support ethical, scalable, and reproducible educational robotics research.

## Requirements
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo (Harmonic)
- ros_gz

## Build Instructions
```bash
cd ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash

