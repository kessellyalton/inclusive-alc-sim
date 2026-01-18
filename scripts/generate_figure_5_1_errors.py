#!/usr/bin/env python3
"""
Generate Figure 5.1: Cumulative error frequency over instructional steps
for baseline vs adaptive tutoring.
"""

import glob
import pandas as pd
import matplotlib.pyplot as plt

LOG_DIR = "/home/alton/alc_logs"
OUT_PATH = "/home/alton/alc_logs/final/figure_5_1_errors.png"

rows = []

for csv_path in glob.glob(f"{LOG_DIR}/*.csv"):
    try:
        df = pd.read_csv(csv_path)

        # Required columns
        if not {"step", "condition", "correct"}.issubset(df.columns):
            continue
        if df.empty:
            continue

        df = df[["step", "condition", "correct"]].copy()
        df["condition"] = df["condition"].str.lower().replace({"fixed": "baseline"})

        # Keep only baseline and adaptive
        df = df[df["condition"].isin(["baseline", "adaptive"])]
        if df.empty:
            continue

        # Binary error per step
        df["error"] = 1 - df["correct"].astype(int)

        rows.append(df)

    except Exception:
        continue

if not rows:
    raise RuntimeError("No valid CSV files found for Figure 5.1")

data = pd.concat(rows, ignore_index=True)
data = data.sort_values("step")

# Mean cumulative error across runs
cum_errors = (
    data
    .groupby(["condition", "step"])["error"]
    .mean()
    .groupby(level=0)
    .cumsum()
    .reset_index()
)

plt.figure(figsize=(7, 4))

for cond in ["baseline", "adaptive"]:
    d = cum_errors[cum_errors["condition"] == cond]
    plt.plot(d["step"], d["error"], label=cond.capitalize())

plt.xlabel("Instructional Step")
plt.ylabel("Cumulative Errors")
plt.legend()
plt.tight_layout()
plt.savefig(OUT_PATH, dpi=300)
plt.show()

print(f"Saved Figure 5.1 to: {OUT_PATH}")

