#!/usr/bin/env python3
"""
Generate Table 5.1 from raw run CSV logs.

Scans log directories, extracts final knowledge per run, and creates a formatted table
comparing Baseline (fixed/baseline) vs Adaptive tutors across learner profiles.
"""
import argparse
import json
import os
import pandas as pd
import re
from pathlib import Path
from typing import Dict, List, Tuple, Optional


def is_derived_file(filename: str) -> bool:
    """Check if file should be excluded (derived/summary files)."""
    exclude_keywords = ["summary", "run_metrics", "condition_summary", "table_", "appendix"]
    filename_lower = filename.lower()
    return any(keyword in filename_lower for keyword in exclude_keywords)


def find_run_csvs(search_dirs: List[str]) -> List[str]:
    """Find all run CSV files, excluding derived files."""
    csv_files = []
    for search_dir in search_dirs:
        if not os.path.exists(search_dir):
            continue
        for root, dirs, files in os.walk(search_dir):
            for file in files:
                if file.endswith(".csv") and not is_derived_file(file):
                    csv_files.append(os.path.join(root, file))
    return sorted(csv_files)


def normalize_condition(condition: str) -> str:
    """Normalize condition names: fixed/baseline → baseline."""
    condition_lower = str(condition).lower().strip()
    if condition_lower in ["fixed", "baseline"]:
        return "baseline"
    return condition_lower


def extract_profile_from_path(csv_path: str) -> Optional[str]:
    """Extract disability profile from folder path or filename."""
    # Known profile names
    profiles = ["none", "dyslexia", "hearing_impairment", "low_vision", "autism", "asd", "motor_difficulty"]
    
    # Check folder names (e.g., "inclusive_dyslexia" → "dyslexia")
    path_lower = csv_path.lower()
    for profile in profiles:
        if profile in path_lower:
            return profile
    
    # Check for "inclusive_<profile>" pattern
    match = re.search(r'inclusive[_-](\w+)', path_lower)
    if match:
        potential_profile = match.group(1)
        if potential_profile in profiles:
            return potential_profile
    
    return None


def extract_profile_from_metadata(csv_path: str) -> Optional[str]:
    """Try to extract disability_profile from metadata JSON file."""
    # Look for metadata JSON in same directory
    csv_dir = os.path.dirname(csv_path)
    csv_basename = os.path.splitext(os.path.basename(csv_path))[0]
    
    # Try common metadata filename patterns
    metadata_patterns = [
        f"{csv_basename}_metadata.json",
        f"{csv_basename}.metadata.json",
        os.path.join(csv_dir, "metadata.json"),
    ]
    
    for metadata_path in metadata_patterns:
        if os.path.exists(metadata_path):
            try:
                with open(metadata_path, 'r') as f:
                    metadata = json.load(f)
                    if "disability_profile" in metadata:
                        return str(metadata["disability_profile"]).strip()
            except Exception:
                pass
    
    return None


def extract_final_knowledge(csv_path: str) -> Dict:
    """
    Extract final knowledge and metadata from a run CSV.
    
    Returns dict with:
    - condition (normalized)
    - disability_profile
    - final_knowledge
    - run_name
    - csv_path
    """
    try:
        df = pd.read_csv(csv_path)
        
        # Check required columns
        if "knowledge" not in df.columns:
            return None
        
        # Get condition (normalize fixed → baseline)
        if "condition" in df.columns:
            condition_raw = df["condition"].iloc[0]
            condition = normalize_condition(condition_raw)
        else:
            return None
        
        # Filter to baseline and adaptive only (also allow inclusive_adaptive)
        if condition not in ["baseline", "adaptive", "inclusive_adaptive"]:
            return None
        
        # Normalize inclusive_adaptive to adaptive for table
        if condition == "inclusive_adaptive":
            condition = "adaptive"
        
        # Get disability profile (try multiple sources)
        profile = None
        
        # 1. Try CSV columns first (most reliable)
        if "disability_profile_param" in df.columns:
            profile_val = str(df["disability_profile_param"].iloc[0]).strip()
            if profile_val and profile_val != "nan":
                profile = profile_val
        
        if not profile and "disability_profile_state" in df.columns:
            profile_val = str(df["disability_profile_state"].iloc[0]).strip()
            if profile_val and profile_val != "nan":
                profile = profile_val
        
        # 2. Try metadata JSON file
        if not profile:
            profile = extract_profile_from_metadata(csv_path)
        
        # 3. Try extracting from path/filename
        if not profile:
            profile = extract_profile_from_path(csv_path)
        
        if not profile:
            return None
        
        # Get final knowledge (last value in knowledge column)
        df_sorted = df.sort_values("step" if "step" in df.columns else df.index.name or "index").reset_index(drop=True)
        final_knowledge = float(df_sorted["knowledge"].iloc[-1])
        
        # Get run name
        run_name = df["run_name"].iloc[0] if "run_name" in df.columns else os.path.basename(csv_path)
        
        return {
            "condition": condition,
            "disability_profile": profile,
            "final_knowledge": final_knowledge,
            "run_name": run_name,
            "csv_path": csv_path,
        }
    except Exception as e:
        print(f"Warning: Failed to process {csv_path}: {e}")
        return None


def format_mean_std(mean: float, std: float, count: int, include_n: bool = True, decimals: int = 2) -> str:
    """Format mean and standard deviation as "mean ± SD (n=...)" or "mean ± SD"."""
    if pd.isna(std):
        std = 0.0
    if include_n:
        return f"{mean:.{decimals}f} ± {std:.{decimals}f} (n={count})"
    else:
        return f"{mean:.{decimals}f} ± {std:.{decimals}f}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate Table 5.1 from raw run CSV logs"
    )
    parser.add_argument(
        "--search-dirs",
        nargs="+",
        default=["/home/alton/alc_logs/final", "/home/alton/alc_logs"],
        help="Directories to search for CSV files (default: /home/alton/alc_logs/final /home/alton/alc_logs)"
    )
    parser.add_argument(
        "--out",
        default="/home/alton/alc_logs/final/table_5_1_final_knowledge_all_profiles.csv",
        help="Output CSV path"
    )
    parser.add_argument(
        "--out-clean",
        default="/home/alton/alc_logs/final/table_5_1_final_knowledge_all_profiles_clean.csv",
        help="Output CSV path for clean version (no n counts)"
    )
    parser.add_argument(
        "--decimals",
        type=int,
        default=2,
        help="Number of decimal places (default: 2)"
    )
    args = parser.parse_args()

    # Find all run CSV files
    print(f"Scanning for run CSV files in: {args.search_dirs}")
    csv_files = find_run_csvs(args.search_dirs)
    print(f"Found {len(csv_files)} potential run CSV files (excluding derived files)")

    if not csv_files:
        raise SystemExit("Error: No run CSV files found. Check search directories.")

    # Extract final knowledge from each run
    print("\nExtracting final knowledge from runs...")
    rows = []
    for csv_file in csv_files:
        result = extract_final_knowledge(csv_file)
        if result:
            rows.append(result)

    if not rows:
        raise SystemExit("Error: No valid runs found with required columns (condition, knowledge, disability_profile).")

    df = pd.DataFrame(rows)
    print(f"\n✓ Processed {len(df)} runs")

    # Validation report
    print("\n=== Validation Report ===")
    print(f"\nProfiles found: {sorted(df['disability_profile'].unique())}")
    print(f"\nConditions found: {sorted(df['condition'].unique())}")
    
    print("\nRun counts by profile × condition:")
    counts = df.groupby(["disability_profile", "condition"]).size().reset_index(name="count")
    print(counts.to_string(index=False))
    
    # Check for missing combinations
    print("\n⚠️  Missing combinations (profile × condition):")
    profiles = sorted(df["disability_profile"].unique())
    conditions = ["baseline", "adaptive"]
    missing = []
    for profile in profiles:
        for condition in conditions:
            count = len(df[(df["disability_profile"] == profile) & (df["condition"] == condition)])
            if count == 0:
                missing.append((profile, condition))
                print(f"  - {profile} × {condition}: 0 runs")
    
    if not missing:
        print("  None - all combinations have data!")

    # Group by profile × condition and compute statistics
    print("\n=== Computing Statistics ===")
    grouped = df.groupby(["disability_profile", "condition"])["final_knowledge"].agg([
        ("mean", "mean"),
        ("std", "std"),
        ("count", "count")
    ]).reset_index()

    # Create pivot table with formatted values
    print("\n=== Generating Table 5.1 ===")
    
    # Full version with n counts
    grouped["formatted"] = grouped.apply(
        lambda row: format_mean_std(row["mean"], row["std"], int(row["count"]), include_n=True, decimals=args.decimals),
        axis=1
    )
    table_full = grouped.pivot(
        index="disability_profile",
        columns="condition",
        values="formatted"
    ).reset_index()

    # Clean version without n counts
    grouped["formatted_clean"] = grouped.apply(
        lambda row: format_mean_std(row["mean"], row["std"], int(row["count"]), include_n=False, decimals=args.decimals),
        axis=1
    )
    table_clean = grouped.pivot(
        index="disability_profile",
        columns="condition",
        values="formatted_clean"
    ).reset_index()

    # Rename columns for publication
    if "baseline" in table_full.columns:
        table_full = table_full.rename(columns={"baseline": "Baseline Tutor"})
    if "adaptive" in table_full.columns:
        table_full = table_full.rename(columns={"adaptive": "Adaptive Tutor"})
    
    if "baseline" in table_clean.columns:
        table_clean = table_clean.rename(columns={"baseline": "Baseline Tutor"})
    if "adaptive" in table_clean.columns:
        table_clean = table_clean.rename(columns={"adaptive": "Adaptive Tutor"})

    # Sort by profile (custom order)
    profile_order = ["none", "dyslexia", "hearing_impairment", "low_vision"]
    table_full["disability_profile"] = pd.Categorical(
        table_full["disability_profile"],
        categories=[p for p in profile_order if p in table_full["disability_profile"].values] + 
                   [p for p in table_full["disability_profile"].values if p not in profile_order],
        ordered=True
    )
    table_full = table_full.sort_values("disability_profile").reset_index(drop=True)
    
    table_clean["disability_profile"] = pd.Categorical(
        table_clean["disability_profile"],
        categories=[p for p in profile_order if p in table_clean["disability_profile"].values] + 
                   [p for p in table_clean["disability_profile"].values if p not in profile_order],
        ordered=True
    )
    table_clean = table_clean.sort_values("disability_profile").reset_index(drop=True)

    # Ensure output directory exists
    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    # Save full version
    table_full.to_csv(args.out, index=False)
    print(f"\n✓ Saved full table: {args.out}")

    # Save clean version
    if args.out_clean:
        outdir_clean = os.path.dirname(args.out_clean)
        if outdir_clean:
            os.makedirs(outdir_clean, exist_ok=True)
        table_clean.to_csv(args.out_clean, index=False)
        print(f"✓ Saved clean table: {args.out_clean}")

    # Print formatted tables
    print("\n" + "=" * 80)
    print("TABLE 5.1: Mean Final Knowledge (± SD) by Profile and Condition")
    print("=" * 80)
    print("\nFull version (with n counts):")
    print(table_full.to_string(index=False))
    
    print("\n" + "-" * 80)
    print("Clean version (for LibreOffice/publication):")
    print("-" * 80)
    print(table_clean.to_string(index=False))

    # Print summary statistics
    print("\n" + "=" * 80)
    print("Summary Statistics")
    print("=" * 80)
    summary = df.groupby("condition")["final_knowledge"].agg(["mean", "std", "count"])
    for condition in ["baseline", "adaptive"]:
        if condition in summary.index:
            mean_val = summary.loc[condition, "mean"]
            std_val = summary.loc[condition, "std"]
            count = int(summary.loc[condition, "count"])
            print(f"\n{condition.capitalize()} Tutor:")
            print(f"  Overall mean: {format_mean_std(mean_val, std_val, count, include_n=True, decimals=args.decimals)}")
            print(f"  Total runs: {count}")


if __name__ == "__main__":
    main()
