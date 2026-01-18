#!/usr/bin/env python3
"""
Generate Figure 5.3: Policy reward convergence over training episodes.

Aggregates total reward per run (episode) and plots learning dynamics
for adaptive and inclusive adaptive tutoring policies.
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

LOG_DIR = Path("/home/alton/alc_logs")
OUT_PATH = Path("/home/alton/alc_logs/final/figure_5_3_policy_convergence.png")

ALLOWED_CONDITIONS = ["adaptive", "inclusive_adaptive"]

def is_run_csv(path: Path) -> bool:
    return path.suffix == ".csv" and not any(
        k in path.name.lower() for k in ["summary", "table_", "figure_"]
    )

def main():
    rows = []

    for csv_path in sorted(LOG_DIR.glob("*.csv")):
        if not is_run_csv(csv_path):
            continue

        try:
            df = pd.read_csv(csv_path)
            if df.empty or "reward" not in df.columns or "condition" not in df.columns:
                continue

            condition = str(df["condition"].iloc[0]).lower()
            if condition not in ALLOWED_CONDITIONS:
                continue

            total_reward = df["reward"].sum()

            rows.append({
                "episode": len(rows) + 1,
                "condition": condition,
                "total_reward": total_reward,
            })

        except Exception:
            continue

    if not rows:
        raise RuntimeError("No valid runs found for policy convergence plot.")

    df_all = pd.DataFrame(rows)

    plt.figure(figsize=(7, 4))

    for condition in df_all["condition"].unique():
        sub = df_all[df_all["condition"] == condition]
        plt.plot(
            sub["episode"],
            sub["total_reward"],
            marker="o",
            linewidth=2,
            label=condition.replace("_", " ").title(),
        )

    plt.xlabel("Training Episode")
    plt.ylabel("Cumulative Reward per Episode")
    plt.title("Policy Reward Convergence Over Training Episodes")
    plt.legend()
    plt.grid(alpha=0.3)

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(OUT_PATH, dpi=300)
    plt.close()

    print(f"✓ Saved Figure 5.3 to: {OUT_PATH}")

if __name__ == "__main__":
    main()

