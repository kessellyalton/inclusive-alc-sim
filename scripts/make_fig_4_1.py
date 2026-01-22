#!/usr/bin/env python3
import os
import signal
import subprocess
import time
from pathlib import Path

# Adjust these paths if your repo is elsewhere
PROJECT_ROOT = Path.home() / "dev" / "inclusive-alc-sim"
ROS_WS = PROJECT_ROOT / "ros2_ws"
FIG_DIR = PROJECT_ROOT / "figures"
OUT_FILE = FIG_DIR / "figure_4_1_gazebo_environment.png"

# Window title match (works on most systems)
GAZEBO_WINDOW_MATCH = "Gazebo Sim"


def run(cmd: str, *, check=True, env=None) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        shell=True,
        check=check,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def source_env() -> dict:
    """
    Source ROS 2 + workspace and return an environment dict for subprocesses.
    """
    source_script = ROS_WS / "source_workspace.sh"
    if source_script.exists():
        cmd = f"bash -lc 'source {source_script} && env'"
    else:
        cmd = f"bash -lc 'source /opt/ros/jazzy/setup.bash && source {ROS_WS}/install/setup.bash && env'"

    p = subprocess.run(cmd, shell=True, check=True, stdout=subprocess.PIPE, text=True)
    env = dict(line.split("=", 1) for line in p.stdout.splitlines() if "=" in line)
    return env


def wait_for_window(timeout_s: int = 45) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        out = run("wmctrl -l", check=False).stdout.lower()
        if GAZEBO_WINDOW_MATCH.lower() in out:
            return True
        time.sleep(0.5)
    return False


def focus_window() -> None:
    run(f"wmctrl -a '{GAZEBO_WINDOW_MATCH}'", check=False)
    time.sleep(0.7)


def screenshot_root(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    # Captures the full desktop; focusing Gazebo ensures it is in front
    run(f"import -window root '{path}'", check=True)


def main() -> int:
    print("======================================================================")
    print("Figure 4.1 Generator: Minimal Classroom World (No Robot)")
    print("======================================================================")
    print(f"Output will be saved to:\n  {OUT_FILE}\n")
    print("This will capture:")
    print("  - Minimal classroom world (ground plane + sun)")
    print("  - Uniform lighting")
    print("  - Spatial reference")
    print("  - NO robot (environment-focused framing)\n")

    env = source_env()

    # Launch Gazebo directly with world file (no robot spawn)
    world_file = ROS_WS / "src" / "alc_simulation" / "worlds" / "classroom.sdf"
    if not world_file.exists():
        print(f"ERROR: World file not found: {world_file}")
        return 1

    launch_cmd = f"gz sim -r {world_file}"
    print(f"Launching Gazebo (world only, no robot):\n  {launch_cmd}\n")

    proc = subprocess.Popen(
        ["bash", "-lc", launch_cmd],
        cwd=str(ROS_WS),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,  # enables clean group shutdown
    )

    try:
        print("Waiting for Gazebo window...")
        if not wait_for_window():
            print("ERROR: Gazebo window not detected.")
            print("Tip: run `wmctrl -l` and confirm the window title includes 'Gazebo Sim'.")
            return 1

        print("Bringing Gazebo to the front...")
        focus_window()

        print("\n📌 FIGURE 4.1 framing instructions:")
        print("  - Adjust camera to show the minimal classroom environment")
        print("  - Focus on ground plane and uniform lighting")
        print("  - Ensure spatial reference is visible")
        print("  - No robot will appear (environment only)")
        print("  - Hide the tab bar if needed")
        print(f"\n⏸️  When ready, press Enter to capture the screenshot...")
        input()  # Wait for user to press Enter

        print("\nCapturing screenshot now...")
        screenshot_root(OUT_FILE)

        if OUT_FILE.exists() and OUT_FILE.stat().st_size > 0:
            print(f"✅ Saved Figure 4.1 to: {OUT_FILE}")
            return 0

        print("⚠️ Screenshot file missing or empty.")
        return 2

    finally:
        # Stop Gazebo cleanly
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            time.sleep(1.5)
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
