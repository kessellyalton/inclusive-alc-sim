#!/bin/bash
# Build script that suppresses harmless setuptools warnings
# Usage: ./build_quiet.sh [colcon build arguments...]

# Suppress the pytest-repeat unbuilt egg warning
export PYTHONWARNINGS="ignore::UserWarning:setuptools.command.easy_install"

# Run colcon build with all arguments passed, filtering out the warning
colcon build --symlink-install "$@" 2>&1 | grep -v "UserWarning: Unbuilt egg" | grep -v "self.local_index = Environment" | grep -v "easy_install.py:363"
