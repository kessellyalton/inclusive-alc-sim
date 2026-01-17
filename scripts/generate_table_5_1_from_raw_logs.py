#!/usr/bin/env python3

import pandas as pd
from pathlib import Path

LOG_DIR = Path("/home/alton/alc_logs")
OUTPUT_DIR = LOG_DIR / "final"
OUTPUT_DIR.mkdir(exist_ok=True)

OUTPUT_FILE = OUTPUT_DIR / "table_5_1_final_knowledge_all_profiles.csv"

rows = []

for csv_path in LOG_DIR.glob("*.csv"):
    try:
        df = pd.read_csv(csv_path)

        # Required columns from logger output
        if not {"condition", "disability_profile", "knowledge"}.issubset(df.columns):
            continue

        condition = str(df["condition"].iloc[0]).strip().lower()
        profile = str(df["disability_profile"].iloc[0]).strip().lower()

        if condition not in {"fixed", "baseline", "adaptive"}:
            continue

        condition = "baseline" if condition in {"fixed", "baseline"} else "adaptive"

        final_knowledge = float(df["knowledge"].iloc[-1])

        rows.append({
            "profile": profile,
            "condition": condition,
            "final_knowledge": final_knowledge
        })

    except Exception:
        continue

if not rows:
    raise RuntimeError("No valid run CSVs found in /home/alton/alc_logs")

data = pd.DataFrame(rows)

agg = (
    data.groupby(["profile", "condition"])["final_knowledge"]
        .agg(["count", "mean", "std"])
        .reset_index()
)

def fmt(row):
    if pd.isna(row["std"]):
        return f"{row['mean']:.3f} ± N/A (n={int(row['count'])})"
    return f"{row['mean']:.3f} ± {row['std']:.3f} (n={int(row['count'])})"

agg["Final Knowledge (mean ± SD)"] = agg.apply(fmt, axis=1)

table = agg.pivot(
    index="profile",
    columns="condition",
    values="Final Knowledge (mean ± SD)"
).reset_index()

# Ensure both columns exist
if "baseline" not in table.columns:
    table["baseline"] = ""
if "adaptive" not in table.columns:
    table["adaptive"] = ""

table = table.rename(columns={
    "profile": "Learner Profile",
    "baseline": "Baseline Tutor (Final knowledge: mean ± SD)",
    "adaptive": "Adaptive Tutor (Final knowledge: mean ± SD)",
})

preferred_order = ["none", "dyslexia", "hearing_impairment", "low_vision"]
table["__order"] = table["Learner Profile"].apply(
    lambda x: preferred_order.index(x) if x in preferred_order else 999
)
table = table.sort_values("__order").drop(columns="__order")

table.to_csv(OUTPUT_FILE, index=False)

print("\nTable 5.1 generated successfully:\n")
print(table.to_string(index=False))
print(f"\nSaved to: {OUTPUT_FILE}")

