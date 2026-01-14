# inclusive-alc-sim

inclusive-alc-sim is a ROS 2 Jazzy-based simulation framework for designing and evaluating an inclusive, adaptive robotic tutoring system for low-resource educational contexts.

The project integrates learner modelling, disability-aware adaptivity, and reinforcement-learning-based instructional pacing within a Gazebo simulation environment. It adopts a simulation-first methodology to support ethical, scalable, and reproducible educational robotics research.

## Requirements
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo (Harmonic)
- ros_gz

## Layout
- `ros2_ws/src`: ROS 2 packages
- `docs`: documentation
- `scripts`: utilities and experiments

## Setup and Build

### Initial Setup
```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

### Source the Workspace

**Option 1: Use the helper script (Recommended)**
```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
source source_workspace.sh
```

**Option 2: Manual sourcing**
```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
source /opt/ros/jazzy/setup.bash  # Source ROS 2 base first
source install/local_setup.bash  # Then source workspace overlay
```

**Option 3: Using setup.bash (if ROS 2 base is already sourced)**
```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
source install/setup.bash
```

### Verify Installation
```bash
ros2 pkg list | grep alc
# Should show: alc_bringup, alc_core, alc_description, alc_interfaces, alc_simulation
```

## Running the System

```bash
# Launch the full system
ros2 launch alc_bringup full_system.launch.py

# Or with custom disability profile
ros2 launch alc_bringup full_system.launch.py disability_profile:=adhd

# Check topics
ros2 topic list
ros2 topic echo /learner/state
```
