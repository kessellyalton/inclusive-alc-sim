#!/usr/bin/env python3
"""
Figure 5.2: Accessibility mismatch rates across tutoring conditions.

We operationalize "accessibility mismatch" as a proxy event where the tutor's
selected modality and/or pacing is inconsistent with the learner's disability profile,
based on simple, explicitly stated rules.

Output:
- PNG figure saved to /home/alton/alc_logs/final/figure_5_2_mismatch.png
- CSV summary saved to /home/alton/alc_logs/final/figure_5_2_mismatch_summary.csv
"""

import argparse
import glob
import os
import pandas as pd
import matplotlib.pyplot as plt


LOG_DIR_DEFAULT = "/home/alton/alc_logs"
OUT_PNG_DEFAULT = "/home/alton/alc_logs/final/figure_5_2_mismatch.png"
OUT_CSV_DEFAULT = "/home/alton/alc_logs/final/figure_5_2_mismatch_summary.csv"


def normalize_condition(cond: str) -> str:
    c = str(cond).lower().strip()
    if c == "fixed":
        return "baseline"
    return c


def mismatch_flag(row: pd.Series) -> int:
    """
    Accessibility mismatch proxy rules.

    NOTE: These rules are intentionally simple and transparent.
    Adjust ONLY if your dissertation defines different constraints.

    Profiles used in your experiments:
      - none
      - dyslexia
      - hearing_impairment
      - low_vision
    """

    profile = str(row.get("disability_profile_param", "")).lower().strip()
    modality = str(row.get("action_modality", "")).lower().strip()
    pacing = float(row.get("action_pacing", 1.0))
    difficulty = int(row.get("action_difficulty_level", 0))

    # No-disability baseline: we do not count modality mismatch
    if profile == "none":
        return 0

    # Low vision: visual-heavy instruction is a mismatch
    if profile == "low_vision":
        return 1 if modality == "visual" else 0

    # Hearing impairment: audio-only instruction is a mismatch
    # (multimodal is NOT treated as mismatch here, to remain conservative)
    if profile == "hearing_impairment":
        return 1 if modality == "audio" else 0

    # Dyslexia: reading-heavy + fast/high-demand combinations are mismatches.
    # Conservative operationalization:
    #  (A) visual modality is considered higher risk
    #  (B) high pacing and high difficulty is also higher risk
    if profile == "dyslexia":
        if modality == "visual":
            return 1
        if pacing > 1.0 and difficulty >= 4:
            return 1
        return 0

    # Default: unknown profiles -> no mismatch counted
    return 0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir", default=LOG_DIR_DEFAULT, help="Directory containing run CSV logs")
    ap.add_argument(
        "--conditions",
        nargs=2,
        default=["baseline", "adaptive"],
        help="Two conditions to compare. Example: baseline adaptive OR baseline inclusive_adaptive",
    )
    ap.add_argument("--out-png", default=OUT_PNG_DEFAULT, help="Output PNG path")
    ap.add_argument("--out-csv", default=OUT_CSV_DEFAULT, help="Output CSV path (summary stats)")
    args = ap.parse_args()

    cond_a = normalize_condition(args.conditions[0])
    cond_b = normalize_condition(args.conditions[1])
    keep_conditions = {cond_a, cond_b}

    csv_files = sorted(glob.glob(os.path.join(args.log_dir, "*.csv")))
    if not csv_files:
        raise SystemExit(f"No CSV files found in {args.log_dir}")

    per_run = []

    for path in csv_files:
        try:
            df = pd.read_csv(path)
            if df.empty:
                continue

            required = {"condition", "step", "action_modality", "action_pacing", "action_difficulty_level"}
            if not required.issubset(df.columns):
                continue
            if "disability_profile_param" not in df.columns:
                continue

            cond = normalize_condition(df["condition"].iloc[0])
            if cond not in keep_conditions:
                continue

            # Compute mismatch per step
            d = df[[
                "condition",
                "step",
                "disability_profile_param",
                "action_modality",
                "action_pacing",
                "action_difficulty_level",
            ]].copy()

            d["condition"] = d["condition"].map(normalize_condition)
            d["mismatch"] = d.apply(mismatch_flag, axis=1)

            # Per-run mismatch rate
            profile = str(d["disability_profile_param"].iloc[0]).strip()
            mismatch_rate = float(d["mismatch"].mean())  # fraction of steps with mismatch

            per_run.append({
                "run_csv": os.path.basename(path),
                "condition": cond,
                "disability_profile": profile,
                "mismatch_rate": mismatch_rate,
                "n_steps": int(len(d)),
            })

        except Exception:
            continue

    if not per_run:
        raise SystemExit("No valid runs found for the requested conditions.")

    runs = pd.DataFrame(per_run)

    # Keep only the common profiles you report
    profile_order = ["none", "dyslexia", "hearing_impairment", "low_vision"]
    runs["disability_profile"] = runs["disability_profile"].astype(str)

    # Summary stats for plotting
    summary = (
        runs
        .groupby(["disability_profile", "condition"])["mismatch_rate"]
        .agg(mean="mean", std="std", count="count")
        .reset_index()
    )

    # Save summary CSV for reporting
    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    summary.to_csv(args.out_csv, index=False)

    # Pivot into columns for plotting
    # Ensure both conditions exist as columns where possible
    pivot_mean = summary.pivot(index="disability_profile", columns="condition", values="mean")
    pivot_std = summary.pivot(index="disability_profile", columns="condition", values="std")

    # Order profiles
    ordered_profiles = [p for p in profile_order if p in pivot_mean.index] + [p for p in pivot_mean.index if p not in profile_order]
    pivot_mean = pivot_mean.loc[ordered_profiles]
    pivot_std = pivot_std.loc[ordered_profiles]

    # Plot (no explicit colors specified)
    plt.figure(figsize=(7.2, 4.2))

    x = range(len(pivot_mean.index))
    width = 0.38

    # For side-by-side bars
    conds = [cond_a, cond_b]
    labels = {
        "baseline": "Baseline Tutor",
        "adaptive": "Adaptive Tutor",
        "inclusive_adaptive": "Inclusive Adaptive Tutor",
    }

    for i, cond in enumerate(conds):
        if cond not in pivot_mean.columns:
            continue
        means = pivot_mean[cond].fillna(0).values
        stds = pivot_std[cond].fillna(0).values
        offsets = [v + (i - 0.5) * width for v in x]
        plt.bar(offsets, means, width=width, yerr=stds, capsize=3, label=labels.get(cond, cond))

    plt.xticks(list(x), list(pivot_mean.index), rotation=0)
    plt.xlabel("Learner Profile")
    plt.ylabel("Accessibility Mismatch Rate (fraction of steps)")
    plt.title("Accessibility Mismatch Rates by Profile and Condition")
    plt.legend()
    plt.tight_layout()

    os.makedirs(os.path.dirname(args.out_png), exist_ok=True)
    plt.savefig(args.out_png, dpi=300)
    plt.show()

    print(f"Saved figure: {args.out_png}")
    print(f"Saved summary: {args.out_csv}")


if __name__ == "__main__":
    main()

