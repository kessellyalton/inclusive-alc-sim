#!/usr/bin/env python3
"""
Generate Table 5.2: Cumulative Cognitive Load by learner profile and tutoring condition.
Baseline (fixed) vs Adaptive (adaptive).
"""

import pandas as pd
import glob
import json
import os

LOG_DIR = "/home/alton/alc_logs"
OUT_FULL = "/home/alton/alc_logs/final/table_5_2_cognitive_load_full.csv"
OUT_CLEAN = "/home/alton/alc_logs/final/table_5_2_cognitive_load_clean.csv"

rows = []

for csv_path in glob.glob(os.path.join(LOG_DIR, "*.csv")):
    try:
        df = pd.read_csv(csv_path)
        if df.empty:
            continue
        if not {"cognitive_load", "condition"}.issubset(df.columns):
            continue

        condition = str(df["condition"].iloc[0]).lower().strip()
        if condition == "fixed":
            condition = "baseline"
        if condition not in ["baseline", "adaptive"]:
            continue

        # Disability profile (prefer CSV param; fallback to metadata json)
        profile = None
        if "disability_profile_param" in df.columns:
            profile = str(df["disability_profile_param"].iloc[0]).strip()

        if (not profile) or (profile.lower() == "nan"):
            meta_path = csv_path.replace(".csv", "_metadata.json")
            if os.path.exists(meta_path):
                with open(meta_path, "r", encoding="utf-8") as f:
                    profile = json.load(f).get("disability_profile")

        if not profile:
            continue

        # Cumulative cognitive load over all steps in the run
        cumulative_load = float(df["cognitive_load"].sum())

        rows.append({
            "disability_profile": profile,
            "condition": condition,
            "cumulative_cognitive_load": cumulative_load
        })

    except Exception as e:
        print(f"Skipping {csv_path}: {e}")

df = pd.DataFrame(rows)
if df.empty:
    raise SystemExit("No valid runs found to compute cumulative cognitive load.")

grouped = (
    df.groupby(["disability_profile", "condition"])["cumulative_cognitive_load"]
      .agg(mean="mean", std="std", count="count")
      .reset_index()
)

def fmt(m, s, n, clean=False):
    s = 0.0 if pd.isna(s) else float(s)
    if clean:
        return f"{m:.2f} ± {s:.2f}"
    return f"{m:.2f} ± {s:.2f} (n={int(n)})"

grouped["formatted"] = grouped.apply(
    lambda r: fmt(r["mean"], r["std"], r["count"], clean=False), axis=1
)
grouped["formatted_clean"] = grouped.apply(
    lambda r: fmt(r["mean"], r["std"], r["count"], clean=True), axis=1
)

table_full = grouped.pivot(
    index="disability_profile", columns="condition", values="formatted"
).reset_index()

table_clean = grouped.pivot(
    index="disability_profile", columns="condition", values="formatted_clean"
).reset_index()

table_full = table_full.rename(columns={"baseline": "Baseline Tutor", "adaptive": "Adaptive Tutor"})
table_clean = table_clean.rename(columns={"baseline": "Baseline Tutor", "adaptive": "Adaptive Tutor"})

os.makedirs(os.path.dirname(OUT_FULL), exist_ok=True)
table_full.to_csv(OUT_FULL, index=False)
table_clean.to_csv(OUT_CLEAN, index=False)

print("✓ Saved:")
print("  Full :", OUT_FULL)
print("  Clean:", OUT_CLEAN)

print("\nTable 5.2 (full):")
print(table_full.to_string(index=False))
