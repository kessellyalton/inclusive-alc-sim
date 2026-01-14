#!/bin/bash
# Source script for inclusive-alc-sim ROS 2 workspace
# Usage: source source_workspace.sh
# Works with both bash and zsh

# Detect shell and get script directory accordingly
if [ -n "$ZSH_VERSION" ]; then
    # zsh
    SCRIPT_DIR="$(cd "$(dirname "${(%):-%x}")" && pwd)"
elif [ -n "$BASH_VERSION" ]; then
    # bash
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    # fallback
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

# Source ROS 2 base installation first
# Use absolute path to avoid current directory issues
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    # Save current directory
    OLD_PWD="$(pwd)"
    # Change to ROS 2 directory to avoid setup.sh lookup errors in workspace
    cd /opt/ros/jazzy 2>/dev/null || true
    # Source ROS 2 setup (this sets environment in current shell)
    source setup.bash 2>/dev/null || {
        # If that fails, try setup.sh directly
        if [ -f "setup.sh" ]; then
            source setup.sh 2>/dev/null
        fi
    }
    # Return to original directory
    cd "$OLD_PWD" 2>/dev/null || cd "$SCRIPT_DIR" || true
else
    echo "Warning: ROS 2 installation not found at /opt/ros/jazzy"
    echo "Make sure ROS 2 Jazzy is installed."
    return 1
fi

# Source the workspace overlay
if [ -f "$SCRIPT_DIR/install/local_setup.bash" ]; then
    source "$SCRIPT_DIR/install/local_setup.bash"
    echo "Workspace sourced successfully!"
    echo ""
    echo "Verifying packages..."
    # Wait a moment for environment to settle, then check
    sleep 0.1
    if command -v ros2 >/dev/null 2>&1; then
        PKG_COUNT=$(ros2 pkg list 2>/dev/null | grep -c "^alc_" || echo "0")
        if [ "$PKG_COUNT" -gt 0 ]; then
            echo "Found $PKG_COUNT ALC package(s):"
            ros2 pkg list 2>/dev/null | grep "^alc_"
        else
            echo "  Warning: No ALC packages found in ros2 pkg list"
            echo "  This might indicate a sourcing issue."
            echo "  Try: source /opt/ros/jazzy/setup.bash && source install/local_setup.bash"
        fi
    else
        echo "  Error: ros2 command not found. Make sure ROS 2 is properly installed."
        return 1
    fi
else
    echo "Error: install/local_setup.bash not found."
    echo "Run 'colcon build --symlink-install' first."
    return 1
fi
