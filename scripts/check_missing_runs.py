#!/usr/bin/env python3
"""
Check which profile × condition × seed combinations are missing from logs.

Helps identify what simulations need to be run to complete Table 5.1 data.
"""
import argparse
import os
import pandas as pd
from pathlib import Path
from typing import Set, Tuple


# Available disability profiles
DISABILITY_PROFILES = ["none", "dyslexia", "hearing_impairment", "low_vision"]

# Conditions
CONDITIONS = ["baseline", "adaptive"]

# Default seeds to check
DEFAULT_SEEDS = list(range(1, 11))  # 1-10


def find_existing_runs(search_dirs: list) -> Set[Tuple[str, str, int]]:
    """Find existing runs and return set of (profile, condition, seed) tuples."""
    existing = set()
    
    for search_dir in search_dirs:
        if not os.path.exists(search_dir):
            continue
        
        for root, dirs, files in os.walk(search_dir):
            for file in files:
                if not file.endswith(".csv"):
                    continue
                
                # Skip derived files
                if any(x in file.lower() for x in ["summary", "run_metrics", "condition_summary", "table_", "appendix"]):
                    continue
                
                csv_path = os.path.join(root, file)
                try:
                    df = pd.read_csv(csv_path, nrows=1)  # Just read header + first row
                    
                    # Extract profile
                    profile = None
                    if "disability_profile_param" in df.columns:
                        profile = str(df["disability_profile_param"].iloc[0]).strip()
                    elif "disability_profile_state" in df.columns:
                        profile = str(df["disability_profile_state"].iloc[0]).strip()
                    
                    # Extract condition
                    condition = None
                    if "condition" in df.columns:
                        cond_raw = str(df["condition"].iloc[0]).lower().strip()
                        if cond_raw in ["fixed", "baseline"]:
                            condition = "baseline"
                        elif cond_raw == "adaptive":
                            condition = "adaptive"
                    
                    # Try to extract seed from filename or run_name
                    seed = None
                    if "run_name" in df.columns:
                        run_name = str(df["run_name"].iloc[0])
                        # Try to extract seed from run_name
                        import re
                        seed_match = re.search(r'seed[_\s]*(\d+)', run_name, re.IGNORECASE)
                        if seed_match:
                            seed = int(seed_match.group(1))
                    
                    # Also try filename
                    if seed is None:
                        import re
                        seed_match = re.search(r'seed[_\s]*(\d+)', file, re.IGNORECASE)
                        if seed_match:
                            seed = int(seed_match.group(1))
                    
                    if profile and condition and seed:
                        existing.add((profile, condition, seed))
                        
                except Exception:
                    pass
    
    return existing


def generate_launch_commands(missing: Set[Tuple[str, str, int]], workspace_dir: str) -> list:
    """Generate ROS2 launch commands for missing runs."""
    commands = []
    
    condition_to_policy = {
        "baseline": "fixed",
        "adaptive": "adaptive"
    }
    
    for profile, condition, seed in sorted(missing):
        policy_mode = condition_to_policy[condition]
        cmd = (
            f"cd {workspace_dir} && "
            f"source install/setup.bash && "
            f"ros2 launch alc_bringup full_system.launch.py "
            f"disability_profile:={profile} "
            f"policy_mode:={policy_mode} "
            f"seed:={seed}"
        )
        commands.append((profile, condition, seed, cmd))
    
    return commands


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check which profile × condition × seed combinations are missing"
    )
    parser.add_argument(
        "--search-dirs",
        nargs="+",
        default=["/home/alton/alc_logs/final", "/home/alton/alc_logs"],
        help="Directories to search for existing logs"
    )
    parser.add_argument(
        "--profiles",
        nargs="+",
        default=DISABILITY_PROFILES,
        help="Disability profiles to check"
    )
    parser.add_argument(
        "--conditions",
        nargs="+",
        default=CONDITIONS,
        help="Conditions to check"
    )
    parser.add_argument(
        "--seeds",
        type=int,
        nargs="+",
        default=DEFAULT_SEEDS,
        help="Seeds to check"
    )
    parser.add_argument(
        "--workspace",
        default=os.path.expanduser("~/dev/inclusive-alc-sim/ros2_ws"),
        help="ROS2 workspace directory"
    )
    parser.add_argument(
        "--generate-commands",
        action="store_true",
        help="Generate launch commands for missing runs"
    )
    args = parser.parse_args()

    # Find existing runs
    print("Scanning for existing runs...")
    existing = find_existing_runs(args.search_dirs)
    print(f"Found {len(existing)} existing runs")

    # Generate all required combinations
    required = set()
    for profile in args.profiles:
        for condition in args.conditions:
            for seed in args.seeds:
                required.add((profile, condition, seed))

    # Find missing
    missing = required - existing

    # Print summary
    print(f"\n{'='*80}")
    print("COVERAGE SUMMARY")
    print(f"{'='*80}")
    print(f"Required combinations: {len(required)}")
    print(f"Existing runs: {len(existing)}")
    print(f"Missing runs: {len(missing)}")
    print(f"Coverage: {100 * len(existing) / len(required):.1f}%")

    # Print missing by profile × condition
    print(f"\n{'='*80}")
    print("MISSING RUNS BY PROFILE × CONDITION")
    print(f"{'='*80}")
    
    missing_by_combo = {}
    for profile, condition, seed in missing:
        key = (profile, condition)
        if key not in missing_by_combo:
            missing_by_combo[key] = []
        missing_by_combo[key].append(seed)
    
    for (profile, condition), seeds in sorted(missing_by_combo.items()):
        print(f"\n{profile} × {condition}: {len(seeds)} missing runs")
        print(f"  Missing seeds: {sorted(seeds)}")

    # Generate commands if requested
    if args.generate_commands and missing:
        print(f"\n{'='*80}")
        print("LAUNCH COMMANDS FOR MISSING RUNS")
        print(f"{'='*80}")
        commands = generate_launch_commands(missing, args.workspace)
        
        # Save to file
        script_path = os.path.join(args.workspace, "run_missing_simulations.sh")
        with open(script_path, "w") as f:
            f.write("#!/bin/bash\n")
            f.write("# Generated launch commands for missing runs\n")
            f.write("# Run this script or execute commands individually\n\n")
            
            for profile, condition, seed, cmd in commands:
                f.write(f"# {profile} × {condition} × seed_{seed}\n")
                f.write(f"{cmd}\n")
                f.write("sleep 5  # Brief pause between runs\n\n")
        
        os.chmod(script_path, 0o755)
        print(f"\n✓ Saved {len(commands)} commands to: {script_path}")
        print(f"\nTo run all missing simulations:")
        print(f"  bash {script_path}")
        print(f"\nOr run individually (first 5 commands):")
        for profile, condition, seed, cmd in commands[:5]:
            print(f"\n  # {profile} × {condition} × seed_{seed}")
            print(f"  {cmd}")


if __name__ == "__main__":
    main()
