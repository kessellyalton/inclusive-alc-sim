#!/bin/zsh
# Source script for inclusive-alc-sim ROS 2 workspace (zsh version)
# Usage: source source_workspace.zsh

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${(%):-%x}")" && pwd)"

# Source ROS 2 base installation first
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    # Change to ROS 2 directory to avoid setup.sh lookup errors
    OLD_PWD="$(pwd)"
    cd /opt/ros/jazzy 2>/dev/null || true
    source setup.bash 2>/dev/null || {
        if [ -f "setup.sh" ]; then
            source setup.sh 2>/dev/null
        fi
    }
    cd "$OLD_PWD" 2>/dev/null || cd "$SCRIPT_DIR" || true
else
    echo "Warning: ROS 2 installation not found at /opt/ros/jazzy"
    return 1
fi

# Source the workspace overlay
if [ -f "$SCRIPT_DIR/install/local_setup.bash" ]; then
    source "$SCRIPT_DIR/install/local_setup.bash"
    echo "Workspace sourced successfully!"
    
    # Verify packages are available
    if command -v ros2 >/dev/null 2>&1; then
        PKG_COUNT=$(ros2 pkg list 2>/dev/null | grep -c "^alc_" || echo "0")
        if [ "$PKG_COUNT" -gt 0 ]; then
            echo "Found $PKG_COUNT ALC package(s)"
        else
            echo "Warning: Packages not found. Try: source /opt/ros/jazzy/setup.bash && source install/local_setup.bash"
        fi
    fi
else
    echo "Error: install/local_setup.bash not found. Run 'colcon build' first."
    return 1
fi
