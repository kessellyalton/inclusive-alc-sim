#!/usr/bin/env python3
"""
Generate Figure 4.2: Simplified Gazebo classroom environment showing the stationary robotic tutor.

This script:
1. Launches Gazebo with the minimal classroom world
2. Spawns the robot
3. Provides instructions for manual screenshot capture

Figure 4.2 is a literal snapshot of the Gazebo world defined in:
ros2_ws/src/alc_simulation/worlds/classroom.sdf

The world contains only:
- Ground plane
- Sun / uniform lighting
- No furniture, walls, or textures

This minimalism is intentional and defensible, not incomplete.
"""

import subprocess
import sys
import os
import time
import signal
from pathlib import Path


def check_dependencies():
    """Check if required ROS 2 and Gazebo tools are available."""
    required = ["ros2", "gz", "colcon"]
    missing = []
    
    for cmd in required:
        result = subprocess.run(
            ["which", cmd],
            capture_output=True,
            text=True
        )
        if result.returncode != 0:
            missing.append(cmd)
    
    if missing:
        print(f"❌ Missing required commands: {', '.join(missing)}")
        print("Please install ROS 2 Jazzy, Gazebo Harmonic, and colcon.")
        return False
    
    return True


def build_workspace():
    """Build the ROS 2 workspace."""
    workspace = Path(__file__).parent.parent / "ros2_ws"
    print(f"📦 Building workspace: {workspace}")
    
    result = subprocess.run(
        ["colcon", "build", "--symlink-install"],
        cwd=workspace,
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        print(f"❌ Build failed:\n{result.stderr}")
        return False
    
    print("✅ Workspace built successfully")
    return True


def take_screenshot(output_path):
    """Take a screenshot using available tools."""
    # Try gnome-screenshot first (most common on Ubuntu)
    result = subprocess.run(
        ["gnome-screenshot", "-f", str(output_path)],
        capture_output=True,
        text=True
    )
    
    if result.returncode == 0:
        return True
    
    # Fallback to ImageMagick import
    result = subprocess.run(
        ["import", "-window", "root", str(output_path)],
        capture_output=True,
        text=True
    )
    
    return result.returncode == 0


def launch_gazebo_for_screenshot():
    """
    Launch Gazebo with minimal setup for Figure 4.2.
    
    This launches:
    - Gazebo with classroom.sdf world
    - Robot spawn (via robot_state_publisher and ros_gz_sim create)
    - No learner nodes, no policy nodes (minimal for screenshot)
    """
    workspace = Path(__file__).parent.parent / "ros2_ws"
    source_script = workspace / "source_workspace.sh"
    figures_dir = Path(__file__).parent.parent / "figures"
    figures_dir.mkdir(exist_ok=True)
    output_path = figures_dir / "figure_4_2_gazebo_environment.png"
    
    print("\n" + "="*70)
    print("🚀 Launching Gazebo for Figure 4.2")
    print("="*70)
    print("\nThis will:")
    print("  1. Launch Gazebo with the minimal classroom world")
    print("  2. Spawn the stationary robotic tutor")
    print("  3. Wait 10 seconds for you to position the camera")
    print("  4. Automatically capture a screenshot")
    print("\n📸 Screenshot Instructions:")
    print("  - In Gazebo GUI, rotate camera slightly downward")
    print("  - Ensure robot is centered")
    print("  - Ensure empty ground plane is visible")
    print("  - Disable physics/contact visuals if needed")
    print(f"  - Screenshot will be saved to: {output_path}")
    print("\n⏹️  Press Ctrl+C to stop Gazebo when done")
    print("="*70 + "\n")
    
    # Launch command
    launch_cmd = [
        "bash", "-c",
        f"source {source_script} && "
        "ros2 launch alc_simulation sim.launch.py"
    ]
    
    try:
        process = subprocess.Popen(
            launch_cmd,
            cwd=workspace,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        
        # Print output in real-time and wait for Gazebo to be ready
        print("Gazebo is starting...\n")
        gazebo_ready = False
        for line in process.stdout:
            print(line, end='')
            # Check for common Gazebo ready signals
            if ("Gazebo Sim" in line or "Simulation running" in line or 
                "robot_state_publisher" in line.lower()):
                if not gazebo_ready:
                    gazebo_ready = True
                    print("\n✅ Gazebo is ready!")
                    print("   You have 10 seconds to position the camera...")
                    print("   (Make sure Gazebo window is visible and focused)")
                    time.sleep(10)
                    print("\n📸 Taking screenshot now...")
                    if take_screenshot(output_path):
                        print(f"✅ Screenshot saved to: {output_path}")
                    else:
                        print("⚠️  Automatic screenshot failed. Please take manually.")
                        print(f"   Save as: {output_path}")
                    print("\n⏹️  Press Ctrl+C to stop Gazebo when done viewing")
        
        process.wait()
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Stopping Gazebo...")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
        print("✅ Gazebo stopped")
        return True
    
    return process.returncode == 0


def main():
    """Main function to generate Figure 4.2."""
    print("="*70)
    print("Figure 4.2 Generator: Gazebo Classroom Environment")
    print("="*70)
    print("\nThis script helps you generate Figure 4.2:")
    print("  Simplified Gazebo classroom environment showing the stationary robotic tutor")
    print("\nThe figure represents:")
    print("  - Minimal Gazebo world (ground plane + sun only)")
    print("  - Stationary robotic tutor (fixed, non-locomotive)")
    print("  - Intentional environmental minimalism")
    print()
    
    if not check_dependencies():
        sys.exit(1)
    
    # Ask user if they want to rebuild
    rebuild = input("Rebuild workspace? (y/N): ").strip().lower()
    if rebuild == 'y':
        if not build_workspace():
            sys.exit(1)
    
    # Launch Gazebo
    if not launch_gazebo_for_screenshot():
        print("\n❌ Failed to launch Gazebo")
        sys.exit(1)
    
    figures_dir = Path(__file__).parent.parent / "figures"
    output_path = figures_dir / "figure_4_2_gazebo_environment.png"
    
    print("\n" + "="*70)
    print("✅ Figure 4.2 generation complete!")
    print("="*70)
    print("\nYour screenshot should show:")
    print("  ✓ Robot centered in frame")
    print("  ✓ Empty ground plane visible")
    print("  ✓ Minimal environment (no furniture/walls)")
    print(f"\n📁 Screenshot location: {output_path}")
    if output_path.exists():
        print(f"   File size: {output_path.stat().st_size / 1024:.1f} KB")


if __name__ == "__main__":
    main()
