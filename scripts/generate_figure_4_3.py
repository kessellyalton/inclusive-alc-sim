#!/usr/bin/env python3
"""
Generate Figure 4.3: Abstract learner representation as ROS 2 cognitive nodes.

⚠️ Important distinction:
Figure 4.3 is NOT a Gazebo screenshot.
It is a ROS 2 computation figure showing the node graph.

What Figure 4.3 Actually Shows:
- learner_model_node
- disability_sim_node
- tutor_policy_node (or policy_node)
- Internal mathematical state (knowledge, load, error)
- Topic connections between nodes

There is no learner model in Gazebo, by design.
"""

import subprocess
import sys
import os
import time
import signal
from pathlib import Path
import json
import re


def check_dependencies():
    """Check if required ROS 2 tools are available."""
    required = ["ros2", "rqt_graph"]
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
        print(f"⚠️  Missing optional commands: {', '.join(missing)}")
        print("   (rqt_graph is optional - we can generate graph programmatically)")
    
    return True


def source_workspace():
    """Source the ROS 2 workspace."""
    workspace = Path(__file__).parent.parent / "ros2_ws"
    source_script = workspace / "source_workspace.sh"
    
    # Return the command prefix to source workspace
    return f"source {source_script} && "


def get_ros2_nodes():
    """Get list of ROS 2 nodes."""
    source_cmd = source_workspace()
    cmd = f"{source_cmd}ros2 node list"
    
    result = subprocess.run(
        ["bash", "-c", cmd],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        print(f"❌ Failed to get nodes: {result.stderr}")
        return []
    
    nodes = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
    return nodes


def get_ros2_topics():
    """Get list of ROS 2 topics."""
    source_cmd = source_workspace()
    cmd = f"{source_cmd}ros2 topic list"
    
    result = subprocess.run(
        ["bash", "-c", cmd],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        print(f"❌ Failed to get topics: {result.stderr}")
        return []
    
    topics = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
    return topics


def get_topic_info(topic):
    """Get publisher and subscriber info for a topic."""
    source_cmd = source_workspace()
    cmd = f"{source_cmd}ros2 topic info {topic}"
    
    result = subprocess.run(
        ["bash", "-c", cmd],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0:
        return {"publishers": [], "subscribers": []}
    
    publishers = []
    subscribers = []
    current_section = None
    
    for line in result.stdout.split('\n'):
        line = line.strip()
        if 'Publisher count:' in line or 'Publishers:' in line:
            current_section = 'publishers'
        elif 'Subscriber count:' in line or 'Subscribers:' in line:
            current_section = 'subscribers'
        elif line.startswith('-'):
            node_name = line.replace('-', '').strip()
            if current_section == 'publishers':
                publishers.append(node_name)
            elif current_section == 'subscribers':
                subscribers.append(node_name)
    
    return {"publishers": publishers, "subscribers": subscribers}


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


def generate_graph_with_rqt():
    """
    Generate node graph using rqt_graph (interactive GUI).
    
    This is the recommended method for Figure 4.3.
    """
    source_cmd = source_workspace()
    cmd = f"{source_cmd}rqt_graph"
    
    figures_dir = Path(__file__).parent.parent / "figures"
    figures_dir.mkdir(exist_ok=True)
    output_path = figures_dir / "figure_4_3_ros2_node_graph.png"
    
    print("\n" + "="*70)
    print("📊 Launching rqt_graph for Figure 4.3")
    print("="*70)
    print("\nThis will open an interactive node graph visualization.")
    print("\n📸 Screenshot Instructions:")
    print("  1. Wait for the graph to fully load")
    print("  2. Arrange nodes for clarity (drag to reposition)")
    print("  3. Focus on learner-related nodes:")
    print("     - learner_model_node")
    print("     - disability_sim_node")
    print("     - tutor_policy_node (or policy_node)")
    print("  4. Hide system nodes if needed (right-click → hide)")
    print("  5. When ready, press Enter to take automatic screenshot")
    print(f"  6. Screenshot will be saved to: {output_path}")
    print("\n⏹️  Close rqt_graph when done")
    print("="*70 + "\n")
    
    try:
        process = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        print("✅ rqt_graph launched.")
        print("   Arrange the graph, then press Enter to take screenshot...")
        input()
        
        print("\n📸 Taking screenshot now...")
        if take_screenshot(output_path):
            print(f"✅ Screenshot saved to: {output_path}")
        else:
            print("⚠️  Automatic screenshot failed. Please take manually.")
            print(f"   Save as: {output_path}")
        
        print("\n   Press Enter again when done viewing...")
        input()
        
        process.terminate()
        process.wait(timeout=5)
        
    except KeyboardInterrupt:
        print("\n⏹️  Stopping rqt_graph...")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
    
    return True


def generate_text_graph():
    """
    Generate a text-based representation of the node graph.
    
    This is a fallback if rqt_graph is not available.
    """
    print("\n" + "="*70)
    print("📊 Generating Text-Based Node Graph")
    print("="*70)
    
    nodes = get_ros2_nodes()
    topics = get_ros2_topics()
    
    if not nodes:
        print("❌ No nodes found. Is the system running?")
        return False
    
    print(f"\nFound {len(nodes)} nodes:")
    for node in nodes:
        print(f"  - {node}")
    
    print(f"\nFound {len(topics)} topics")
    
    # Focus on learner-related topics
    learner_topics = [
        "/learner/state",
        "/learner/outcome",
        "/tutor/action",
        "/disability/profile"
    ]
    
    print("\n" + "="*70)
    print("Node-Topic Connections (Learner System):")
    print("="*70)
    
    for topic in learner_topics:
        if topic in topics:
            info = get_topic_info(topic)
            print(f"\n📡 {topic}")
            if info["publishers"]:
                print(f"   Publishers: {', '.join(info['publishers'])}")
            if info["subscribers"]:
                print(f"   Subscribers: {', '.join(info['subscribers'])}")
    
    print("\n" + "="*70)
    print("💡 Recommendation:")
    print("   Use rqt_graph for a visual representation suitable for Figure 4.3")
    print("   Run: ros2 run rqt_graph rqt_graph")
    print("="*70)
    
    return True


def launch_system_for_graph():
    """
    Launch the full system in background, then open rqt_graph and capture screenshot.
    """
    workspace = Path(__file__).parent.parent / "ros2_ws"
    source_script = workspace / "source_workspace.sh"
    figures_dir = Path(__file__).parent.parent / "figures"
    figures_dir.mkdir(exist_ok=True)
    output_path = figures_dir / "figure_4_3_ros2_node_graph.png"
    
    print("\n" + "="*70)
    print("🚀 Launching System + rqt_graph for Figure 4.3")
    print("="*70)
    print("\nThis will:")
    print("  1. Launch the full ROS 2 system in the background")
    print("  2. Wait for nodes to start")
    print("  3. Open rqt_graph for visualization")
    print("  4. Wait for you to arrange the graph")
    print("  5. Capture a screenshot automatically")
    print(f"  6. Save to: {output_path}")
    print("\n📸 Screenshot Instructions:")
    print("  - Wait for the graph to fully load")
    print("  - Arrange nodes for clarity (drag to reposition)")
    print("  - Focus on learner-related nodes:")
    print("    - learner_model_node")
    print("    - disability_sim_node")
    print("    - tutor_policy_node")
    print("  - Hide system nodes if needed (right-click → hide)")
    print("  - When ready, press Enter to take screenshot")
    print("\n⏹️  Press Ctrl+C to stop everything when done")
    print("="*70 + "\n")
    
    launch_cmd = [
        "bash", "-c",
        f"source {source_script} && "
        "ros2 launch alc_bringup full_system.launch.py"
    ]
    
    system_process = None
    rqt_process = None
    
    try:
        # Launch system in background
        print("🚀 Launching ROS 2 system...")
        system_process = subprocess.Popen(
            launch_cmd,
            cwd=workspace,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid  # Create new process group for clean shutdown
        )
        
        # Wait for nodes to start
        print("⏳ Waiting for nodes to start (10 seconds)...")
        time.sleep(10)
        
        # Check if nodes are available
        nodes = get_ros2_nodes()
        learner_nodes = [n for n in nodes if "learner" in n.lower() or "tutor" in n.lower() or "disability" in n.lower()]
        if learner_nodes:
            print(f"✅ Found learner nodes: {', '.join(learner_nodes)}")
        else:
            print("⚠️  Learner nodes not yet visible, but continuing...")
        
        # Launch rqt_graph
        print("\n📊 Launching rqt_graph...")
        source_cmd = source_workspace()
        rqt_cmd = f"{source_cmd}rqt_graph"
        rqt_process = subprocess.Popen(
            ["bash", "-c", rqt_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        print("✅ rqt_graph launched.")
        print("   Arrange the graph, then press Enter to take screenshot...")
        input()
        
        print("\n📸 Taking screenshot now...")
        if take_screenshot(output_path):
            print(f"✅ Screenshot saved to: {output_path}")
        else:
            print("⚠️  Automatic screenshot failed. Please take manually.")
            print(f"   Save as: {output_path}")
        
        print("\n   Press Enter again when done viewing...")
        input()
        
        return True
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Stopping processes...")
        if rqt_process:
            rqt_process.terminate()
            try:
                rqt_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                rqt_process.kill()
        
        if system_process:
            try:
                os.killpg(os.getpgid(system_process.pid), signal.SIGTERM)
                system_process.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(os.getpgid(system_process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        
        print("✅ All processes stopped")
        return True
    
    finally:
        # Cleanup
        if rqt_process:
            rqt_process.terminate()
            try:
                rqt_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                rqt_process.kill()
        
        if system_process:
            try:
                os.killpg(os.getpgid(system_process.pid), signal.SIGTERM)
                system_process.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(os.getpgid(system_process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass


def main():
    """Main function to generate Figure 4.3."""
    print("="*70)
    print("Figure 4.3 Generator: ROS 2 Node Graph")
    print("="*70)
    print("\nThis script helps you generate Figure 4.3:")
    print("  Abstract learner representation as ROS 2 cognitive nodes")
    print("\n⚠️  Important:")
    print("  - Figure 4.3 is NOT a Gazebo screenshot")
    print("  - It shows the ROS 2 computation graph")
    print("  - Focus on learner_model_node, disability_sim_node, tutor_policy_node")
    print()
    
    if not check_dependencies():
        sys.exit(1)
    
    print("\nOptions:")
    print("  1. Launch system + use rqt_graph (recommended)")
    print("  2. Use rqt_graph only (if system already running)")
    print("  3. Generate text-based graph (fallback)")
    
    choice = input("\nSelect option (1-3): ").strip()
    
    if choice == "1":
        # Launch system in background, then open rqt_graph and capture screenshot
        launch_system_for_graph()
    
    elif choice == "2":
        # Just open rqt_graph
        generate_graph_with_rqt()
    
    elif choice == "3":
        # Text-based fallback
        generate_text_graph()
    
    else:
        print("❌ Invalid choice")
        sys.exit(1)
    
    figures_dir = Path(__file__).parent.parent / "figures"
    output_path = figures_dir / "figure_4_3_ros2_node_graph.png"
    
    print("\n" + "="*70)
    print("✅ Figure 4.3 generation complete!")
    print("="*70)
    print("\nYour graph should show:")
    print("  ✓ learner_model_node")
    print("  ✓ disability_sim_node")
    print("  ✓ tutor_policy_node (or policy_node)")
    print("  ✓ Topic connections between nodes")
    print(f"\n📁 Screenshot location: {output_path}")
    if output_path.exists():
        print(f"   File size: {output_path.stat().st_size / 1024:.1f} KB")


if __name__ == "__main__":
    main()