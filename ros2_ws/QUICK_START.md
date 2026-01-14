# Quick Start Guide

## Sourcing the Workspace

**IMPORTANT:** You must source the workspace in your **current shell**, not in a script or subshell.

### Method 1: Direct Sourcing (Recommended)

```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
```

Then verify:
```bash
ros2 pkg list | grep alc
```

### Method 2: Using the Helper Script

If you're using **bash**:
```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
source source_workspace.sh
```

If you're using **zsh**:
```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
source source_workspace.zsh
```

### Troubleshooting

If packages are not found after sourcing:

1. **Check ROS 2 is installed:**
   ```bash
   echo $ROS_DISTRO
   # Should output: jazzy
   ```

2. **Check AMENT_PREFIX_PATH:**
   ```bash
   echo $AMENT_PREFIX_PATH | grep inclusive-alc-sim
   # Should show your workspace path
   ```

3. **Manual verification:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/dev/inclusive-alc-sim/ros2_ws/install/local_setup.bash
   ros2 pkg list | grep alc
   ```

4. **If still not working, rebuild:**
   ```bash
   cd ~/dev/inclusive-alc-sim/ros2_ws
   colcon build --symlink-install
   source install/local_setup.bash
   ```

## Launching the System

After sourcing:
```bash
ros2 launch alc_bringup full_system.launch.py
```
