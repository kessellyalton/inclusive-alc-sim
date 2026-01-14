#!/bin/bash
# Script to fix setup.bash after colcon build
# Run this after each 'colcon build' if you want to use setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="$SCRIPT_DIR/install"

if [ ! -f "$INSTALL_DIR/setup.bash" ]; then
    echo "Error: install/setup.bash not found. Run 'colcon build' first."
    exit 1
fi

# Backup original
cp "$INSTALL_DIR/setup.bash" "$INSTALL_DIR/setup.bash.backup"

# Create fixed version
cat > "$INSTALL_DIR/setup.bash" << 'SETUP_EOF'
#!/bin/bash
# Fixed version of setup.bash that handles path resolution correctly
# This file sources ROS 2 base and workspace overlay properly

# Get the directory where this script is located
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 base first (change to ROS 2 directory to avoid path issues)
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    (
        cd /opt/ros/jazzy
        source setup.bash
    )
fi

# Source the workspace overlay
if [ -f "$_SCRIPT_DIR/local_setup.bash" ]; then
    source "$_SCRIPT_DIR/local_setup.bash"
else
    echo "Error: local_setup.bash not found in $_SCRIPT_DIR" >&2
    return 1
fi

unset _SCRIPT_DIR
SETUP_EOF

chmod +x "$INSTALL_DIR/setup.bash"
echo "Fixed setup.bash - you can now use 'source install/setup.bash'"
