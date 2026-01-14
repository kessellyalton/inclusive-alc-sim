#!/usr/bin/env python3
"""
Analyze multiple experiment run CSVs and generate aggregated summary.

Processes multiple CSV files and creates a summary table with metrics grouped by condition.
"""
import argparse
import glob
import os
import pandas as pd


def accessibility_match(modality: str, disability: str) -> int:
    """
    Proxy rule set (keep consistent with reward.py).
    
    Returns 1 if modality is a good match for disability, 0 otherwise.
    """
    if disability == "dyslexia":
        return 1 if modality in ["audio", "multimodal"] else 0
    if disability == "hearing_impairment":
        return 1 if modality in ["visual", "multimodal"] else 0
    if disability == "low_vision":
        return 1 if modality in ["audio", "multimodal"] else 0
    return 1


def metrics_for_file(path: str, cmax: float) -> dict:
    """
    Compute metrics for a single CSV file.
    
    Returns a dictionary with metrics for the run.
    """
    df = pd.read_csv(path).sort_values("step").reset_index(drop=True)

    df["access_match"] = df.apply(
        lambda r: accessibility_match(str(r["action_modality"]), str(r["disability_profile_state"])),
        axis=1
    )

    return {
        "csv": path,
        "run_name": df["run_name"].iloc[0] if "run_name" in df.columns else os.path.basename(path),
        "condition": df["condition"].iloc[0] if "condition" in df.columns else "",
        "disability_profile": df["disability_profile_state"].iloc[0] if "disability_profile_state" in df.columns else "",
        "learning_gain": float(df["knowledge"].iloc[-1] - df["knowledge"].iloc[0]),
        "overload_rate": float((df["cognitive_load"] > cmax).mean()),
        "success_rate": float(df["correct"].mean()),
        "accessibility_match_rate": float(df["access_match"].mean()),
        "mean_reward": float(df["reward"].mean()),
        "n_steps": int(len(df)),
    }


def main() -> None:
    p = argparse.ArgumentParser(
        description="Analyze multiple experiment run CSVs and generate aggregated summary"
    )
    p.add_argument("--glob", required=True, help="Glob pattern e.g. '~/alc_logs/*.csv'")
    p.add_argument("--out", default="results/summary_all.csv", help="Output CSV path")
    p.add_argument("--cmax", type=float, default=0.75, help="Cognitive load overload threshold")
    args = p.parse_args()

    paths = glob.glob(os.path.expanduser(args.glob))
    if not paths:
        raise SystemExit(f"No files matched: {args.glob}")

    print(f"Processing {len(paths)} CSV files...")
    rows = [metrics_for_file(x, args.cmax) for x in paths]
    outdir = os.path.dirname(args.out)
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    df = pd.DataFrame(rows)
    df.to_csv(args.out, index=False)
    print(f"\nWrote: {args.out}")
    
    # Print aggregated statistics by condition and disability profile
    print("\n=== Aggregated Metrics by Condition and Disability Profile ===")
    if "condition" in df.columns and "disability_profile" in df.columns:
        agg = df.groupby(["condition", "disability_profile"])[
            ["learning_gain", "success_rate", "overload_rate", "accessibility_match_rate", "mean_reward"]
        ].mean()
        print(agg)
    else:
        print("Warning: condition or disability_profile columns not found in data")


if __name__ == "__main__":
    main()