#!/usr/bin/env python3
"""
Generate Table 5.1: Mean final knowledge (± SD) comparing Baseline (fixed) and Adaptive tutors.

This script processes batch analysis results to create a formatted table showing
mean final knowledge with standard deviation for each disability profile × condition combination.
"""
import argparse
import os
import pandas as pd


def format_mean_std(mean: float, std: float, decimals: int = 2) -> str:
    """
    Format mean and standard deviation as "mean ± SD".
    
    Args:
        mean: Mean value
        std: Standard deviation
        decimals: Number of decimal places (default: 2)
    
    Returns:
        Formatted string like "0.75 ± 0.12"
    """
    return f"{mean:.{decimals}f} ± {std:.{decimals}f}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate Table 5.1: Mean final knowledge (± SD) by condition and disability profile"
    )
    parser.add_argument(
        "--csv",
        default="results/summary_all.csv",
        help="Path to batch summary CSV (output from analyze_batch.py)"
    )
    parser.add_argument(
        "--out",
        default="results/table_5_1.csv",
        help="Output path for Table 5.1 CSV"
    )
    parser.add_argument(
        "--decimals",
        type=int,
        default=2,
        help="Number of decimal places for formatting (default: 2)"
    )
    args = parser.parse_args()

    # Load batch summary
    if not os.path.exists(args.csv):
        raise SystemExit(f"Error: Input file not found: {args.csv}\n"
                        f"Run: python3 scripts/analyze_batch.py --glob '~/alc_logs/*.csv' --out {args.csv}")

    df = pd.read_csv(args.csv)
    
    # Verify required columns exist
    required_cols = ["condition", "disability_profile", "final_knowledge"]
    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise SystemExit(f"Error: Missing required columns in CSV: {missing}\n"
                        f"Available columns: {list(df.columns)}")

    # Filter to only "fixed" and "adaptive" conditions
    df_filtered = df[df["condition"].isin(["fixed", "adaptive"])].copy()
    
    if len(df_filtered) == 0:
        raise SystemExit(f"Error: No rows found with condition in ['fixed', 'adaptive']\n"
                        f"Available conditions: {df['condition'].unique()}")

    # Group by disability_profile and condition, compute mean and std
    grouped = df_filtered.groupby(["disability_profile", "condition"])["final_knowledge"].agg([
        ("mean", "mean"),
        ("std", "std"),
        ("count", "count")
    ]).reset_index()

    # Format as "mean ± std"
    grouped["final_knowledge_formatted"] = grouped.apply(
        lambda row: format_mean_std(row["mean"], row["std"], args.decimals),
        axis=1
    )

    # Pivot to create table format: disability_profile as rows, condition as columns
    table = grouped.pivot(
        index="disability_profile",
        columns="condition",
        values="final_knowledge_formatted"
    ).reset_index()

    # Ensure columns are in desired order: disability_profile, fixed, adaptive
    if "fixed" in table.columns and "adaptive" in table.columns:
        table = table[["disability_profile", "fixed", "adaptive"]]
    elif "fixed" in table.columns:
        table = table[["disability_profile", "fixed"]]
    elif "adaptive" in table.columns:
        table = table[["disability_profile", "adaptive"]]

    # Sort by disability profile (custom order: none, dyslexia, hearing_impairment, low_vision)
    profile_order = ["none", "dyslexia", "hearing_impairment", "low_vision"]
    table["disability_profile"] = pd.Categorical(
        table["disability_profile"],
        categories=profile_order,
        ordered=True
    )
    table = table.sort_values("disability_profile").reset_index(drop=True)

    # Create output directory if needed
    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    # Save to CSV
    table.to_csv(args.out, index=False)
    print(f"\n✓ Generated Table 5.1: {args.out}")
    
    # Print formatted table to console
    print("\n=== Table 5.1: Mean Final Knowledge (± SD) ===")
    print("\nDisability Profile | Baseline (fixed) | Adaptive")
    print("-" * 60)
    for _, row in table.iterrows():
        profile = row["disability_profile"]
        fixed_val = row.get("fixed", "N/A")
        adaptive_val = row.get("adaptive", "N/A")
        print(f"{profile:18s} | {fixed_val:16s} | {adaptive_val}")

    # Print summary statistics
    print("\n=== Summary Statistics ===")
    stats = grouped.groupby("condition")["mean"].agg(["mean", "std", "count"])
    print("\nOverall means by condition:")
    for condition in ["fixed", "adaptive"]:
        if condition in stats.index:
            mean_val = stats.loc[condition, "mean"]
            std_val = stats.loc[condition, "std"]
            count = int(stats.loc[condition, "count"])
            print(f"  {condition}: {format_mean_std(mean_val, std_val, args.decimals)} (n={count})")


if __name__ == "__main__":
    main()
