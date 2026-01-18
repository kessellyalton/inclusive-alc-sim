#!/usr/bin/env python3
"""
Generate Table 5.1 / 5.2 from raw run CSV logs.

Scans log directories, extracts final knowledge per run, and creates a formatted table
comparing two conditions (by default: Baseline vs Adaptive).

This version is corrected to:
- handle empty CSVs safely (avoids "single positional indexer is out-of-bounds")
- sort runs robustly (by `step` if present, else file order)
- NOT collapse `inclusive_adaptive` into `adaptive` (unless you explicitly request it)
- let you choose which two conditions to compare via CLI
"""

import argparse
import json
import os
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pandas as pd


# ---------------------------
# File discovery / filtering
# ---------------------------

def is_derived_file(filename: str) -> bool:
    """Check if file should be excluded (derived/summary files)."""
    exclude_keywords = ["summary", "run_metrics", "condition_summary", "table_", "appendix"]
    filename_lower = filename.lower()
    return any(keyword in filename_lower for keyword in exclude_keywords)


def find_run_csvs(search_dirs: List[str]) -> List[str]:
    """Find all run CSV files, excluding derived files."""
    csv_files: List[str] = []
    for search_dir in search_dirs:
        if not os.path.exists(search_dir):
            continue
        for root, _, files in os.walk(search_dir):
            for file in files:
                if file.endswith(".csv") and not is_derived_file(file):
                    csv_files.append(os.path.join(root, file))
    return sorted(csv_files)


# ---------------------------
# Normalization / extraction
# ---------------------------

def normalize_condition(condition: str) -> str:
    """
    Normalize condition names.

    - fixed/baseline -> baseline
    - adaptive -> adaptive
    - inclusive_adaptive -> inclusive_adaptive (kept distinct)
    """
    condition_lower = str(condition).lower().strip()
    if condition_lower in ["fixed", "baseline"]:
        return "baseline"
    return condition_lower


def extract_profile_from_path(csv_path: str) -> Optional[str]:
    """Extract disability profile from folder path or filename."""
    profiles = ["none", "dyslexia", "hearing_impairment", "low_vision", "autism", "asd", "motor_difficulty"]
    path_lower = csv_path.lower()

    for profile in profiles:
        if profile in path_lower:
            return profile

    match = re.search(r"inclusive[_-](\w+)", path_lower)
    if match:
        potential_profile = match.group(1)
        if potential_profile in profiles:
            return potential_profile

    return None


def extract_profile_from_metadata(csv_path: str) -> Optional[str]:
    """Try to extract disability_profile from metadata JSON file next to CSV."""
    csv_dir = os.path.dirname(csv_path)
    csv_basename = os.path.splitext(os.path.basename(csv_path))[0]

    # Most common pattern in your logs: <run_name>_metadata.json
    metadata_path = os.path.join(csv_dir, f"{csv_basename}_metadata.json")
    if os.path.exists(metadata_path):
        try:
            with open(metadata_path, "r", encoding="utf-8") as f:
                metadata = json.load(f)
            if "disability_profile" in metadata:
                return str(metadata["disability_profile"]).strip()
        except Exception:
            pass

    # Fallback patterns (less likely)
    metadata_patterns = [
        os.path.join(csv_dir, f"{csv_basename}.metadata.json"),
        os.path.join(csv_dir, "metadata.json"),
    ]

    for mp in metadata_patterns:
        if os.path.exists(mp):
            try:
                with open(mp, "r", encoding="utf-8") as f:
                    metadata = json.load(f)
                if "disability_profile" in metadata:
                    return str(metadata["disability_profile"]).strip()
            except Exception:
                pass

    return None


def _pick_profile_from_df(df: pd.DataFrame) -> Optional[str]:
    """Extract profile from known CSV columns, if present."""
    for col in ["disability_profile_param", "disability_profile_state"]:
        if col in df.columns:
            val = str(df[col].iloc[0]).strip()
            if val and val.lower() != "nan":
                return val
    return None


def _safe_final_knowledge(df: pd.DataFrame) -> Optional[float]:
    """Return final knowledge after sorting by step if possible; safe for empty frames."""
    if df is None or df.empty:
        return None
    if "knowledge" not in df.columns:
        return None

    d = df.copy()

    # Robust sort by step if column exists
    if "step" in d.columns:
        # ensure step is numeric where possible
        d["__step_num__"] = pd.to_numeric(d["step"], errors="coerce")
        d = d.sort_values(["__step_num__"], kind="mergesort").drop(columns=["__step_num__"])
    # else: keep file order (already chronological in logger output)

    # final knowledge
    if d["knowledge"].dropna().empty:
        return None

    return float(d["knowledge"].dropna().iloc[-1])


def extract_final_knowledge(csv_path: str, allowed_conditions: List[str]) -> Optional[Dict]:
    """
    Extract final knowledge + metadata from a run CSV.

    Returns dict with:
      - condition
      - disability_profile
      - final_knowledge
      - run_name
      - csv_path
    """
    try:
        df = pd.read_csv(csv_path)

        # Guard: empty / no rows
        if df.empty:
            return None

        # Guard: must have knowledge
        if "knowledge" not in df.columns:
            return None

        # Condition
        if "condition" not in df.columns:
            return None

        condition_raw = df["condition"].iloc[0]
        condition = normalize_condition(condition_raw)

        if condition not in allowed_conditions:
            return None

        # Profile (CSV columns > metadata > path)
        profile = _pick_profile_from_df(df)
        if not profile:
            profile = extract_profile_from_metadata(csv_path)
        if not profile:
            profile = extract_profile_from_path(csv_path)
        if not profile:
            return None

        final_knowledge = _safe_final_knowledge(df)
        if final_knowledge is None:
            return None

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


# ---------------------------
# Formatting
# ---------------------------

def format_mean_std(mean: float, std: float, count: int, include_n: bool = True, decimals: int = 2) -> str:
    """Format mean and SD as 'mean ± SD (n=...)' or 'mean ± SD'."""
    if pd.isna(std):
        std = 0.0
    if include_n:
        return f"{mean:.{decimals}f} ± {std:.{decimals}f} (n={count})"
    return f"{mean:.{decimals}f} ± {std:.{decimals}f}"


def build_condition_labels(conditions: List[str]) -> Dict[str, str]:
    """
    Default human-friendly labels.
    You can override via CLI if you want different names in the table.
    """
    labels = {}
    for c in conditions:
        if c == "baseline":
            labels[c] = "Baseline Tutor"
        elif c == "adaptive":
            labels[c] = "Adaptive Tutor"
        elif c == "inclusive_adaptive":
            labels[c] = "Inclusive Adaptive Tutor"
        else:
            labels[c] = c
    return labels


# ---------------------------
# Main
# ---------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a final-knowledge comparison table from raw run CSV logs.")
    parser.add_argument(
        "--search-dirs",
        nargs="+",
        default=["/home/alton/alc_logs/final", "/home/alton/alc_logs"],
        help="Directories to search for CSV files",
    )
    parser.add_argument(
        "--conditions",
        nargs="+",
        default=["baseline", "adaptive"],
        help="Exactly two conditions to compare (e.g., baseline adaptive OR baseline inclusive_adaptive)",
    )
    parser.add_argument(
        "--out",
        default="/home/alton/alc_logs/final/table_final_knowledge.csv",
        help="Output CSV path (full version with n counts)",
    )
    parser.add_argument(
        "--out-clean",
        default="/home/alton/alc_logs/final/table_final_knowledge_clean.csv",
        help="Output CSV path for clean version (no n counts)",
    )
    parser.add_argument(
        "--decimals",
        type=int,
        default=2,
        help="Number of decimal places",
    )
    parser.add_argument(
        "--title",
        default="TABLE: Mean Final Knowledge (± SD) by Profile and Condition",
        help="Title printed to console (does not affect CSV output).",
    )
    args = parser.parse_args()

    if len(args.conditions) != 2:
        raise SystemExit("Error: --conditions must specify exactly two conditions (e.g., baseline adaptive).")

    allowed_conditions = [normalize_condition(c) for c in args.conditions]

    print(f"Scanning for run CSV files in: {args.search_dirs}")
    csv_files = find_run_csvs(args.search_dirs)
    print(f"Found {len(csv_files)} potential run CSV files (excluding derived files)")

    if not csv_files:
        raise SystemExit("Error: No run CSV files found. Check search directories.")

    print("\nExtracting final knowledge from runs...")
    rows: List[Dict] = []
    for csv_file in csv_files:
        result = extract_final_knowledge(csv_file, allowed_conditions=allowed_conditions)
        if result:
            rows.append(result)

    if not rows:
        raise SystemExit(
            "Error: No valid runs found. Ensure CSVs include columns: condition, knowledge (and ideally step/profile)."
        )

    df = pd.DataFrame(rows)
    print(f"\n✓ Processed {len(df)} runs")

    # Validation report
    print("\n=== Validation Report ===")
    print(f"\nProfiles found: {sorted(df['disability_profile'].unique())}")
    print(f"\nConditions found: {sorted(df['condition'].unique())}")

    print("\nRun counts by profile × condition:")
    counts = df.groupby(["disability_profile", "condition"]).size().reset_index(name="count")
    print(counts.to_string(index=False))

    # Missing combinations among the selected conditions
    print("\n⚠️  Missing combinations (profile × condition):")
    profiles = sorted(df["disability_profile"].unique())
    missing: List[Tuple[str, str]] = []
    for profile in profiles:
        for condition in allowed_conditions:
            c = len(df[(df["disability_profile"] == profile) & (df["condition"] == condition)])
            if c == 0:
                missing.append((profile, condition))
                print(f"  - {profile} × {condition}: 0 runs")
    if not missing:
        print("  None - all combinations have data!")

    # Compute stats
    print("\n=== Computing Statistics ===")
    grouped = (
        df.groupby(["disability_profile", "condition"])["final_knowledge"]
        .agg(mean="mean", std="std", count="count")
        .reset_index()
    )

    # Create tables
    print("\n=== Generating Table ===")
    grouped["formatted"] = grouped.apply(
        lambda r: format_mean_std(r["mean"], r["std"], int(r["count"]), include_n=True, decimals=args.decimals),
        axis=1,
    )
    grouped["formatted_clean"] = grouped.apply(
        lambda r: format_mean_std(r["mean"], r["std"], int(r["count"]), include_n=False, decimals=args.decimals),
        axis=1,
    )

    table_full = grouped.pivot(index="disability_profile", columns="condition", values="formatted").reset_index()
    table_clean = grouped.pivot(index="disability_profile", columns="condition", values="formatted_clean").reset_index()

    # Rename columns for publication
    labels = build_condition_labels(allowed_conditions)
    for cond, label in labels.items():
        if cond in table_full.columns:
            table_full = table_full.rename(columns={cond: label})
        if cond in table_clean.columns:
            table_clean = table_clean.rename(columns={cond: label})

    # Sort by profile (custom order)
    profile_order = ["none", "dyslexia", "hearing_impairment", "low_vision"]
    def _ordered_categories(values: List[str]) -> List[str]:
        values_set = list(values)
        return [p for p in profile_order if p in values_set] + [p for p in values_set if p not in profile_order]

    table_full["disability_profile"] = pd.Categorical(
        table_full["disability_profile"],
        categories=_ordered_categories(list(table_full["disability_profile"].values)),
        ordered=True,
    )
    table_full = table_full.sort_values("disability_profile").reset_index(drop=True)

    table_clean["disability_profile"] = pd.Categorical(
        table_clean["disability_profile"],
        categories=_ordered_categories(list(table_clean["disability_profile"].values)),
        ordered=True,
    )
    table_clean = table_clean.sort_values("disability_profile").reset_index(drop=True)

    # Ensure output dirs exist
    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    outdir_clean = os.path.dirname(args.out_clean)
    if outdir_clean:
        os.makedirs(outdir_clean, exist_ok=True)

    # Save
    table_full.to_csv(args.out, index=False)
    table_clean.to_csv(args.out_clean, index=False)
    print(f"\n✓ Saved full table:  {args.out}")
    print(f"✓ Saved clean table: {args.out_clean}")

    # Print tables
    print("\n" + "=" * 80)
    print(args.title)
    print("=" * 80)
    print("\nFull version (with n counts):")
    print(table_full.to_string(index=False))

    print("\n" + "-" * 80)
    print("Clean version (for LibreOffice/publication):")
    print("-" * 80)
    print(table_clean.to_string(index=False))

    # Summary statistics
    print("\n" + "=" * 80)
    print("Summary Statistics")
    print("=" * 80)
    summary = df.groupby("condition")["final_knowledge"].agg(["mean", "std", "count"])
    for condition in allowed_conditions:
        if condition in summary.index:
            mean_val = summary.loc[condition, "mean"]
            std_val = summary.loc[condition, "std"]
            count_val = int(summary.loc[condition, "count"])
            label = labels.get(condition, condition)
            print(f"\n{label}:")
            print(f"  Overall mean: {format_mean_std(mean_val, std_val, count_val, include_n=True, decimals=args.decimals)}")
            print(f"  Total runs: {count_val}")


if __name__ == "__main__":
    main()

