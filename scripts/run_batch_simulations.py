#!/usr/bin/env python3
"""
Batch simulation runner for generating Table 5.1 data.

Runs simulations for all disability profile × condition combinations
with multiple random seeds for statistical validity.
"""
import argparse
import os
import subprocess
import time
from pathlib import Path


# Available disability profiles from DISABILITY_LIBRARY
DISABILITY_PROFILES = ["none", "dyslexia", "hearing_impairment", "low_vision"]

# Conditions to test
CONDITIONS = {
    "baseline": "fixed",  # fixed policy mode maps to baseline condition
    "adaptive": "adaptive"  # adaptive policy mode
}

# Default seeds to use (can be overridden)
DEFAULT_SEEDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]


def run_simulation(profile: str, condition: str, seed: int, workspace_dir: str, timeout: int = 300) -> bool:
    """
    Run a single simulation.
    
    Args:
        profile: Disability profile name
        condition: Condition name (baseline or adaptive)
        seed: Random seed
        workspace_dir: Path to ROS2 workspace
        timeout: Maximum time to wait for simulation (seconds)
    
    Returns:
        True if successful, False otherwise
    """
    policy_mode = CONDITIONS[condition]
    
    print(f"\n{'='*80}")
    print(f"Running: profile={profile}, condition={condition} (policy={policy_mode}), seed={seed}")
    print(f"{'='*80}")
    
    # Change to workspace directory
    os.chdir(workspace_dir)
    
    # Source ROS2 workspace
    source_cmd = f"source {workspace_dir}/install/setup.bash"
    
    # Launch command
    launch_cmd = [
        "ros2", "launch", "alc_bringup", "full_system.launch.py",
        f"disability_profile:={profile}",
        f"policy_mode:={policy_mode}",
        f"seed:={seed}",
    ]
    
    # Full command with sourcing
    full_cmd = f"{source_cmd} && {' '.join(launch_cmd)}"
    
    try:
        # Run simulation (will run until Ctrl+C or completion)
        # For headless mode, we might want to add a timeout or run in background
        process = subprocess.Popen(
            full_cmd,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Wait for process with timeout
        try:
            stdout, stderr = process.communicate(timeout=timeout)
            if process.returncode == 0:
                print(f"✓ Success: {profile}/{condition}/seed_{seed}")
                return True
            else:
                print(f"✗ Failed: {profile}/{condition}/seed_{seed}")
                print(f"  stderr: {stderr[:200]}")
                return False
        except subprocess.TimeoutExpired:
            process.kill()
            print(f"✗ Timeout: {profile}/{condition}/seed_{seed} (exceeded {timeout}s)")
            return False
            
    except Exception as e:
        print(f"✗ Error running {profile}/{condition}/seed_{seed}: {e}")
        return False


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run batch simulations for Table 5.1 data generation"
    )
    parser.add_argument(
        "--workspace",
        default=os.path.expanduser("~/dev/inclusive-alc-sim/ros2_ws"),
        help="Path to ROS2 workspace"
    )
    parser.add_argument(
        "--profiles",
        nargs="+",
        default=DISABILITY_PROFILES,
        help="Disability profiles to simulate (default: all)"
    )
    parser.add_argument(
        "--conditions",
        nargs="+",
        default=list(CONDITIONS.keys()),
        help="Conditions to simulate (default: baseline adaptive)"
    )
    parser.add_argument(
        "--seeds",
        type=int,
        nargs="+",
        default=DEFAULT_SEEDS,
        help="Random seeds to use (default: 1-10)"
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=300,
        help="Timeout per simulation in seconds (default: 300)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be run without actually running"
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip runs that already have log files"
    )
    args = parser.parse_args()

    workspace_dir = os.path.abspath(args.workspace)
    if not os.path.exists(workspace_dir):
        raise SystemExit(f"Error: Workspace not found: {workspace_dir}")

    # Check if workspace is built
    install_dir = os.path.join(workspace_dir, "install")
    if not os.path.exists(install_dir):
        raise SystemExit(f"Error: Workspace not built. Run: cd {workspace_dir} && colcon build")

    print(f"Workspace: {workspace_dir}")
    print(f"Profiles: {args.profiles}")
    print(f"Conditions: {args.conditions}")
    print(f"Seeds: {args.seeds}")
    print(f"Total simulations: {len(args.profiles) * len(args.conditions) * len(args.seeds)}")

    if args.dry_run:
        print("\n=== DRY RUN MODE ===")
        for profile in args.profiles:
            for condition in args.conditions:
                for seed in args.seeds:
                    policy_mode = CONDITIONS[condition]
                    print(f"Would run: profile={profile}, policy_mode={policy_mode}, seed={seed}")
        return

    # Check for existing logs if skip-existing is enabled
    log_dir = os.path.expanduser("~/alc_logs")
    existing_runs = set()
    if args.skip_existing:
        print(f"\nChecking for existing runs in {log_dir}...")
        # This is a simplified check - you might want to check actual CSV files
        if os.path.exists(log_dir):
            for root, dirs, files in os.walk(log_dir):
                for file in files:
                    if file.endswith(".csv") and not any(x in file for x in ["summary", "table_", "metrics"]):
                        # Try to extract profile/condition/seed from filename
                        # This is approximate - actual extraction would need CSV parsing
                        existing_runs.add(file)
        print(f"Found {len(existing_runs)} existing log files")

    # Run simulations
    results = {
        "success": 0,
        "failed": 0,
        "skipped": 0
    }

    for profile in args.profiles:
        for condition in args.conditions:
            for seed in args.seeds:
                # Check if we should skip
                if args.skip_existing:
                    # Simple check - could be improved
                    expected_pattern = f"{condition}_{profile}_seed_{seed}"
                    if any(expected_pattern in f for f in existing_runs):
                        print(f"⊘ Skipping existing: {profile}/{condition}/seed_{seed}")
                        results["skipped"] += 1
                        continue

                success = run_simulation(profile, condition, seed, workspace_dir, args.timeout)
                if success:
                    results["success"] += 1
                else:
                    results["failed"] += 1

                # Small delay between runs
                time.sleep(2)

    # Print summary
    print(f"\n{'='*80}")
    print("BATCH SIMULATION SUMMARY")
    print(f"{'='*80}")
    print(f"Success: {results['success']}")
    print(f"Failed: {results['failed']}")
    print(f"Skipped: {results['skipped']}")
    print(f"Total: {results['success'] + results['failed'] + results['skipped']}")


if __name__ == "__main__":
    main()
