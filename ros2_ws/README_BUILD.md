# Build Instructions

## Standard Build

```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
colcon build --symlink-install
```

## Suppressing Harmless Warnings

The build may show stderr warnings about `pytest-repeat` having an "Unbuilt egg". This is a **harmless system-level warning** that doesn't affect functionality. It's caused by corrupted metadata in the system's pytest-repeat installation.

### Option 1: Use the Quiet Build Script

```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
./build_quiet.sh
```

### Option 2: Suppress Warnings Manually

```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
PYTHONWARNINGS="ignore::UserWarning:setuptools.command.easy_install" colcon build --symlink-install
```

### Option 3: Add to Shell Config (Permanent)

Add this to your `~/.zshrc` or `~/.bashrc`:

```bash
# Suppress harmless setuptools warnings during ROS 2 builds
export PYTHONWARNINGS="ignore::UserWarning:setuptools.command.easy_install"
```

Then rebuild:
```bash
cd ~/dev/inclusive-alc-sim/ros2_ws
colcon build --symlink-install
```

## Note

These warnings are **completely harmless** and don't affect:
- Package building
- Functionality
- Runtime behavior
- Testing

You can safely ignore them if you prefer not to suppress them.
