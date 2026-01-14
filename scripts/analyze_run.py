#!/usr/bin/env python3
"""
Analyze a single experiment run CSV.

Computes metrics and generates plots from a single CSV file produced by logger_node.
"""
import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt


def accessibility_match(modality: str, disability: str) -> int:
    """
    Replicates the exact logic from alc_core.utils.reward.accessibility_match.
    
    Uses the same DISABILITY_LIBRARY structure to ensure consistency with
    the reward computation. Returns 1 for good match, 0 for bad match.
    """
    # DISABILITY_LIBRARY structure (must match models.py exactly)
    DISABILITY_LIBRARY = {
        "none": {
            "modality_penalty": {"visual": 0.00, "audio": 0.00, "multimodal": -0.02}
        },
        "dyslexia": {
            "modality_penalty": {"visual": 0.08, "audio": -0.02, "multimodal": 0.00}
        },
        "hearing_impairment": {
            "modality_penalty": {"visual": 0.00, "audio": 0.10, "multimodal": 0.02}
        },
        "low_vision": {
            "modality_penalty": {"visual": 0.10, "audio": -0.01, "multimodal": 0.02}
        },
    }
    
    # Get disability params (default to "none" if not found)
    d = DISABILITY_LIBRARY.get(disability, DISABILITY_LIBRARY["none"])
    penalty = d["modality_penalty"].get(modality, 0.0)
    
    # Replicate exact logic from reward.py
    # multimodal often helps; allow bonus if not penalized
    if modality == "multimodal" and penalty <= 0.02:
        return 1
    
    # if penalty is negative or zero, that's a match
    if penalty <= 0.0:
        return 1
    
    # penalized modality (returns -0.5 in reward.py, but we return 0 for binary match)
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze a single experiment run CSV and generate metrics and plots"
    )
    parser.add_argument("--csv", required=True, help="Path to a single run CSV produced by logger_node")
    parser.add_argument("--outdir", default="results", help="Where to write plots/summary")
    parser.add_argument("--cmax", type=float, default=0.75, help="Cognitive load overload threshold")
    parser.add_argument("--save_plots", action="store_true", help="Save plots instead of showing them")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    df = pd.read_csv(args.csv)
    df = df.sort_values("step").reset_index(drop=True)

    # ---- core metrics ----
    learning_gain = float(df["knowledge"].iloc[-1] - df["knowledge"].iloc[0])
    overload_rate = float((df["cognitive_load"] > args.cmax).mean())
    success_rate = float(df["correct"].mean())

    df["access_match"] = df.apply(
        lambda r: accessibility_match(str(r["action_modality"]), str(r["disability_profile_state"])),
        axis=1
    )
    accessibility_rate = float(df["access_match"].mean())

    mean_reward = float(df["reward"].mean())
    final_knowledge = float(df["knowledge"].iloc[-1])

    metrics = {
        "csv": args.csv,
        "learning_gain": learning_gain,
        "overload_rate": overload_rate,
        "success_rate": success_rate,
        "accessibility_match_rate": accessibility_rate,
        "final_knowledge": final_knowledge,
        "mean_reward": mean_reward,
        "n_steps": int(len(df)),
    }

    # ---- print metrics ----
    print("\n=== Metrics ===")
    for k, v in metrics.items():
        print(f"{k}: {v}")

    # ---- save a one-row summary CSV ----
    base = os.path.splitext(os.path.basename(args.csv))[0]
    summary_path = os.path.join(args.outdir, f"{base}_summary.csv")
    pd.DataFrame([metrics]).to_csv(summary_path, index=False)
    print(f"\nWrote summary: {summary_path}")

    # ---- plots ----
    # reward curve
    plt.figure()
    plt.plot(df["step"], df["reward"])
    plt.xlabel("Step")
    plt.ylabel("Reward")
    plt.title("Reward vs Step")
    if args.save_plots:
        p = os.path.join(args.outdir, f"{base}_reward.png")
        plt.savefig(p, dpi=200, bbox_inches="tight")
        print(f"Wrote plot: {p}")
    else:
        plt.show()
    plt.close()

    # cumulative reward
    df["cumulative_reward"] = df["reward"].cumsum()
    plt.figure()
    plt.plot(df["step"], df["cumulative_reward"])
    plt.xlabel("Step")
    plt.ylabel("Cumulative Reward")
    plt.title("Cumulative Reward vs Step")
    if args.save_plots:
        p = os.path.join(args.outdir, f"{base}_cum_reward.png")
        plt.savefig(p, dpi=200, bbox_inches="tight")
        print(f"Wrote plot: {p}")
    else:
        plt.show()
    plt.close()

    # knowledge curve
    plt.figure()
    plt.plot(df["step"], df["knowledge"])
    plt.xlabel("Step")
    plt.ylabel("Knowledge")
    plt.title("Knowledge vs Step")
    if args.save_plots:
        p = os.path.join(args.outdir, f"{base}_knowledge.png")
        plt.savefig(p, dpi=200, bbox_inches="tight")
        print(f"Wrote plot: {p}")
    else:
        plt.show()
    plt.close()


if __name__ == "__main__":
    main()
