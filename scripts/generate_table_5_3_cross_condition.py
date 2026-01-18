#!/usr/bin/env python3
"""
Generate Table 5.3: Cross-condition comparison across learner profiles.

Metrics:
- Final knowledge (mean ± SD)
- Cumulative cognitive load (mean per run)
- Cumulative error frequency (mean per run)

Conditions compared:
- baseline
- adaptive

Data source:
- Raw run CSV logs in ~/alc_logs
"""

import pandas as pd
import json
from pathlib import Path

LOG_DIR = Path("/home/alton/alc_logs")
OUT_PATH = Path("/home/alton/alc_logs/final/table_5_3_cross_condition.csv")

ALLOWED_CONDITIONS = ["baseline", "adaptive"]

def normalize_condition(c):
    c = str(c).lower().strip()
    if c in ["fixed", "baseline"]:
        return "baseline"
    return c

def extract_profile(csv_path, df):
    if "disability_profile_param" in df.columns:
        v = str(df["disability_profile_param"].iloc[0]).strip()
        if v and v != "nan":
            return v

    meta = csv_path.with_name(csv_path.stem + "_metadata.json")
    if meta.exists():
        try:
            with open(meta) as f:
                return json.load(f).get("disability_profile")
        except Exception:
            pass

    return None

rows = []

for csv_path in sorted(LOG_DIR.glob("*.csv")):
    try:
        df = pd.read_csv(csv_path)
        if df.empty:
            continue

        if "condition" not in df.columns:
            continue

        condition = normalize_condition(df["condition"].iloc[0])
        if condition not in ALLOWED_CONDITIONS:
            continue

        profile = extract_profile(csv_path, df)
        if not profile:
            continue

        # --- Metrics ---
        final_knowledge = (
            df["knowledge"].dropna().iloc[-1]
            if "knowledge" in df.columns and not df["knowledge"].dropna().empty
            else None
        )

        cumulative_load = (
            df["cognitive_load"].sum()
            if "cognitive_load" in df.columns
            else None
        )

        cumulative_errors = (
            df["error_rate"].sum()
            if "error_rate" in df.columns
            else None
        )

        rows.append({
            "profile": profile,
            "condition": condition,
            "final_knowledge": final_knowledge,
            "cognitive_load": cumulative_load,
            "error_frequency": cumulative_errors,
        })

    except Exception as e:
        print(f"Warning: failed on {csv_path.name}: {e}")

df = pd.DataFrame(rows)

if df.empty:
    raise RuntimeError("No valid runs found for Table 5.3")

# --- Aggregate ---
summary = (
    df.groupby(["profile", "condition"])
    .agg(
        final_knowledge_mean=("final_knowledge", "mean"),
        final_knowledge_std=("final_knowledge", "std"),
        cumulative_load_mean=("cognitive_load", "mean"),
        error_frequency_mean=("error_frequency", "mean"),
        n_runs=("final_knowledge", "count"),
    )
    .reset_index()
)

# --- Format final knowledge ---
summary["final_knowledge"] = summary.apply(
    lambda r: f"{r['final_knowledge_mean']:.2f} ± {r['final_knowledge_std']:.2f}",
    axis=1
)

# --- Pivot to wide format ---
table = summary.pivot(
    index="profile",
    columns="condition",
    values=[
        "final_knowledge",
        "cumulative_load_mean",
        "error_frequency_mean",
    ]
)

table.columns = [
    f"{metric.replace('_', ' ').title()} ({cond.title()})"
    for metric, cond in table.columns
]

table = table.reset_index()

# --- Save ---
OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
table.to_csv(OUT_PATH, index=False)

print(f"✓ Saved Table 5.3 to: {OUT_PATH}")
print("\nTable 5.3 Preview:\n")
print(table.to_string(index=False))

