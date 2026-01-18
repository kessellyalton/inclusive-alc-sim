#!/usr/bin/env python3
import os
import re
import json
import signal
import subprocess
import time
from pathlib import Path
from typing import Set, Tuple

import select  # non-blocking stdout monitoring

# ============================================================
# CONFIGURATION
# ============================================================

PROFILES = ["none", "dyslexia", "hearing_impairment", "low_vision"]
SEEDS = [1, 2, 3, 4, 5]

POLICY_MODE = "inclusive_adaptive"

TARGET_ROWS = 300          # Number of logged steps (rows) per run
MAX_WALL_SECS = 420        # Hard timeout per run (seconds)
NO_PROGRESS_SECS = 45      # Abort if CSV stops growing

LOG_DIR = Path("/home/alton/alc_logs")

ROS2_WS = Path.home() / "dev" / "inclusive-alc-sim" / "ros2_ws"
SOURCE_CMD = f"cd '{ROS2_WS}' && source source_workspace.sh"

CSV_RE = re.compile(r"Logging to\s+(?P<csv>/\S+\.csv)")

# ============================================================
# UTILITIES
# ============================================================

def count_data_rows(csv_path: Path) -> int:
    """Count data rows in CSV (excluding header)."""
    try:
        with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
            return max(0, sum(1 for _ in f) - 1)
    except FileNotFoundError:
        return 0

def existing_runs_set(log_dir: Path) -> Set[Tuple[str, str, int]]:
    """Return set of (condition, disability_profile, seed) present in *_metadata.json."""
    done: Set[Tuple[str, str, int]] = set()
    for meta in log_dir.glob("*_metadata.json"):
        try:
            with meta.open("r", encoding="utf-8") as f:
                md = json.load(f)
            cond = md.get("condition")
            prof = md.get("disability_profile")
            seed = md.get("seed")
            if isinstance(cond, str) and isinstance(prof, str) and isinstance(seed, int):
                done.add((cond, prof, seed))
        except Exception:
            continue
    return done

def stop_process_group(proc: subprocess.Popen) -> None:
    """Send SIGINT then SIGKILL (if needed) to the process group."""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except Exception:
        return
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass

# ============================================================
# CORE RUNNER
# ============================================================

def run_one(profile: str, seed: int) -> None:
    cmd = (
        f"{SOURCE_CMD} && "
        f"ros2 launch alc_bringup full_system.launch.py "
        f"disability_profile:={profile} "
        f"policy_mode:={POLICY_MODE} "
        f"seed:={seed}"
    )

    print("\n" + "=" * 80)
    print(f"RUN: profile={profile} seed={seed} mode={POLICY_MODE} target_rows={TARGET_ROWS}")
    print("=" * 80)

    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,  # new process group
        bufsize=1,
    )

    csv_path: Path | None = None
    start = time.time()
    last_progress_time = start
    last_rows = 0

    try:
        while True:
            now = time.time()

            if now - start > MAX_WALL_SECS:
                print(f"\n⏳ Timeout reached ({MAX_WALL_SECS}s). Stopping run.")
                break

            # Non-blocking read of stdout (if any available)
            if proc.stdout is not None:
                rlist, _, _ = select.select([proc.stdout], [], [], 0.1)
                if rlist:
                    line = proc.stdout.readline()
                    if line:
                        print(line, end="")
                        if csv_path is None:
                            m = CSV_RE.search(line)
                            if m:
                                csv_path = Path(m.group("csv"))
                                print(f"📄 Detected CSV: {csv_path}")

            # Always monitor CSV growth (even if stdout is quiet)
            if csv_path is not None:
                rows = count_data_rows(csv_path)

                if rows > last_rows:
                    last_progress_time = now

                # Print progress only when crossing multiples of 50
                if rows // 50 > last_rows // 50:
                    print(f"✅ Progress: {rows}/{TARGET_ROWS} rows")

                last_rows = rows

                if rows >= TARGET_ROWS:
                    print(f"\n✅ Target reached: {rows} rows. Stopping run.")
                    break

                if now - last_progress_time > NO_PROGRESS_SECS:
                    print(f"\n⚠️ No CSV growth for {NO_PROGRESS_SECS}s. Stopping run as likely stalled.")
                    break

            if proc.poll() is not None:
                print("\nℹ️ Launch process ended on its own.")
                break

    finally:
        stop_process_group(proc)
        print("🛑 Run stopped.\n")

def main():
    print("\n==============================")
    print("INCLUSIVE ADAPTIVE AUTO-RUNNER")
    print("==============================\n")

    completed = existing_runs_set(LOG_DIR)

    for profile in PROFILES:
        for seed in SEEDS:
            key = (POLICY_MODE, profile, seed)
            if key in completed:
                print(f"↩️  Skipping existing run: {key}")
                continue
            run_one(profile, seed)
            completed.add(key)

    print("\n🎉 All requested inclusive_adaptive runs completed.")

if __name__ == "__main__":
    main()

