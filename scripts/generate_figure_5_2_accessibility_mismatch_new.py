#!/usr/bin/env python3
"""
Generate Figure 5.2: Accessibility Mismatch Rates by Profile and Condition

This script produces a publication-ready bar chart comparing accessibility mismatch
rates across learner profiles under Baseline vs Inclusive Adaptive tutoring.

It follows the inclusive-alc-sim pipeline described in CODE_DOCUMENTATION.md:
- logger_node writes per-step CSV logs including:
  - action_modality, action_pacing, disability_profile_param/state
- we compute mismatch using the same DISABILITY_LIBRARY logic used in reward/accessibility_match.

Output:
  ~/dev/inclusive-alc-sim/figures/figure_5_2_mismatch.png  (default)

Usage:
  python3 scripts/generate_figure_5_2_accessibility_mismatch.py \
    --search-dirs /home/alton/alc_logs/final /home/alton/alc_logs \
    --out ~/dev/inclusive-alc-sim/figures/figure_5_2_mismatch.png

Notes:
- "Mismatch" here is defined as fraction of steps where the tutor's modality conflicts
  with the learner's accessibility needs (profile-specific modality penalties).
- This intentionally focuses on modality (not pacing) because pacing mismatch thresholds
  are highly model-specific. If you want a pacing component too, tell me and I’ll extend it.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pandas as pd
import matplotlib.pyplot as plt


# -----------------------------
# Configuration / Label polish
# -----------------------------

KNOWN_PROFILES = ["none", "dyslexia", "hearing_impairment", "low_vision"]

DISPLAY_LABELS = {
    "none": "No disability",
    "dyslexia": "Dyslexia",
    "hearing_impairment": "Hearing impairment",
    "low_vision": "Low vision",
}

# Normalize many possible condition strings to the ones used in dissertation text
COND_ALIASES = {
    "fixed": "baseline",
    "baseline": "baseline",
    "traditional": "baseline",
    "control": "baseline",
    "adaptive": "adaptive",
    "inclusive_adaptive": "inclusive_adaptive",
    "inclusive": "inclusive_adaptive",
    "inclusive-adaptive": "inclusive_adaptive",
}

# Figure compares baseline vs inclusive adaptive (per your Chapter 5.3.2 wording)
PLOT_CONDITIONS = ["baseline", "inclusive_adaptive"]
PLOT_LABELS = {
    "baseline": "Baseline Tutor",
    "inclusive_adaptive": "Inclusive Adaptive Tutor",
}


# -----------------------------
# Disability library
# -----------------------------
# This replicates the exact DISABILITY_LIBRARY from alc_core.utils.models
# and uses the same accessibility_match logic from alc_core.utils.reward
# to ensure consistency with the reward computation.

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


def normalize_condition(raw: str) -> Optional[str]:
    if not raw:
        return None
    raw = raw.strip().lower()
    return COND_ALIASES.get(raw, raw)


def is_derived_file(p: Path) -> bool:
    name = p.name.lower()
    bad = ["summary", "table_", "appendix", "condition_summary", "run_metrics", "_metadata"]
    return any(x in name for x in bad)


def extract_profile_from_path(path: Path) -> Optional[str]:
    s = str(path).lower()
    for prof in KNOWN_PROFILES:
        # whole-word match where possible
        if re.search(rf"(^|[^a-z_]){re.escape(prof)}([^a-z_]|$)", s):
            return prof
    for prof in KNOWN_PROFILES:
        if f"inclusive_{prof}" in s or f"profile_{prof}" in s:
            return prof
    return None


def extract_profile_from_metadata(csv_path: Path) -> Optional[str]:
    meta = csv_path.with_name(csv_path.stem + "_metadata.json")
    if not meta.exists():
        return None
    try:
        data = json.loads(meta.read_text())
        prof = (data.get("disability_profile") or "").strip().lower()
        return prof if prof in KNOWN_PROFILES else None
    except Exception:
        return None


def extract_condition_from_metadata(csv_path: Path) -> Optional[str]:
    meta = csv_path.with_name(csv_path.stem + "_metadata.json")
    if not meta.exists():
        return None
    try:
        data = json.loads(meta.read_text())
        cond = (data.get("condition") or "").strip().lower()
        return normalize_condition(cond)
    except Exception:
        return None


def extract_run_fields(df: pd.DataFrame, csv_path: Path) -> Tuple[Optional[str], Optional[str]]:
    # Condition
    cond = None
    if "condition" in df.columns:
        cond = normalize_condition(str(df["condition"].iloc[0]))

    if not cond:
        cond = extract_condition_from_metadata(csv_path)

    # Profile
    prof = None
    for col in ["disability_profile_param", "disability_profile_state", "disability_profile"]:
        if col in df.columns:
            p = str(df[col].iloc[0]).strip().lower()
            if p in KNOWN_PROFILES:
                prof = p
                break

    if not prof:
        prof = extract_profile_from_metadata(csv_path)

    if not prof:
        prof = extract_profile_from_path(csv_path)

    return cond, prof


def find_run_csvs(search_dirs: List[Path]) -> List[Path]:
    out: List[Path] = []
    for d in search_dirs:
        if not d.exists():
            continue
        for p in d.rglob("*.csv"):
            if is_derived_file(p):
                continue
            out.append(p)
    return sorted(out)


def accessibility_match(modality: str, disability: str) -> int:
    """
    Replicates the exact logic from alc_core.utils.reward.accessibility_match.
    
    Uses the same DISABILITY_LIBRARY structure to ensure consistency with
    the reward computation. Returns 1 for good match, 0 for bad match (mismatch).
    """
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
    
    # penalized modality (mismatch)
    return 0


def modality_mismatch_rate(df: pd.DataFrame, profile: str) -> Optional[float]:
    """
    Fraction of steps where action_modality conflicts with profile accessibility.
    
    Uses the same accessibility_match logic as the reward function to ensure
    consistency. Returns the fraction of steps with mismatch (0 = match, 1 = mismatch).
    """
    if "action_modality" not in df.columns:
        return None

    mods = df["action_modality"].astype(str).str.strip().str.lower()
    
    if profile not in DISABILITY_LIBRARY:
        return None

    # Compute match (1) or mismatch (0) for each step
    matches = mods.apply(lambda m: accessibility_match(m, profile))
    
    if len(matches) == 0:
        return None

    # Return mismatch rate (1 - match rate)
    match_rate = float(matches.mean())
    mismatch_rate = 1.0 - match_rate
    
    return mismatch_rate


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--search-dirs",
        nargs="+",
        default=["/home/alton/alc_logs/final", "/home/alton/alc_logs", str(Path.home() / "alc_logs")],
        help="Directories to search recursively for raw run CSV logs.",
    )
    ap.add_argument(
        "--out",
        default=str(Path.home() / "dev" / "inclusive-alc-sim" / "figures" / "figure_5_2_mismatch.png"),
        help="Output PNG path.",
    )
    ap.add_argument(
        "--min-rows",
        type=int,
        default=10,
        help="Skip CSVs with fewer than this many rows (likely incomplete).",
    )
    args = ap.parse_args()

    search_dirs = [Path(x).expanduser() for x in args.search_dirs]
    csvs = find_run_csvs(search_dirs)

    if not csvs:
        raise SystemExit(f"No run CSVs found under: {search_dirs}")

    records = []
    skipped = 0

    for csv_path in csvs:
        try:
            df = pd.read_csv(csv_path)
        except Exception:
            skipped += 1
            continue

        if len(df) < args.min_rows:
            skipped += 1
            continue

        cond, prof = extract_run_fields(df, csv_path)
        if cond not in PLOT_CONDITIONS:
            skipped += 1
            continue
        if prof not in KNOWN_PROFILES:
            skipped += 1
            continue

        rate = modality_mismatch_rate(df, prof)
        if rate is None:
            skipped += 1
            continue

        records.append(
            {
                "csv": str(csv_path),
                "condition": cond,
                "profile": prof,
                "mismatch_rate": rate,
            }
        )

    if not records:
        raise SystemExit("No usable runs found (check log dirs, action_modality column, and conditions).")

    data = pd.DataFrame.from_records(records)

    # Aggregate across runs (seeds)
    agg = (
        data.groupby(["profile", "condition"])["mismatch_rate"]
        .agg(["mean", "std", "count"])
        .reset_index()
    )

    # Order for plotting
    agg["profile"] = pd.Categorical(agg["profile"], categories=KNOWN_PROFILES, ordered=True)
    agg["condition"] = pd.Categorical(agg["condition"], categories=PLOT_CONDITIONS, ordered=True)
    agg = agg.sort_values(["profile", "condition"])

    # Pivot for plotting
    pivot_mean = agg.pivot(index="profile", columns="condition", values="mean")
    pivot_std = agg.pivot(index="profile", columns="condition", values="std")

    # Plot - matching Figure 5.2 visual style
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    x = list(range(len(KNOWN_PROFILES)))
    width = 0.38  # Matches Figure 5.2 style for 2 bars per profile

    for i, cond in enumerate(PLOT_CONDITIONS):
        y = [float(pivot_mean.loc[p, cond]) if p in pivot_mean.index and cond in pivot_mean.columns else float("nan")
             for p in KNOWN_PROFILES]
        yerr = [float(pivot_std.loc[p, cond]) if p in pivot_std.index and cond in pivot_std.columns else float("nan")
                for p in KNOWN_PROFILES]
        ax.bar(
            [k + (i - 0.5) * width for k in x],
            y,
            width=width,
            yerr=yerr,
            capsize=3,  # Matches Figure 5.2 style
            label=PLOT_LABELS[cond],
        )

    ax.set_xticks(x)
    ax.set_xticklabels([DISPLAY_LABELS[p] for p in KNOWN_PROFILES])
    ax.set_xlabel("Learner accessibility profile")
    ax.set_ylabel("Accessibility Mismatch Rate (fraction of steps)")
    ax.set_title("Accessibility Mismatch Rates by Profile and Condition")
    ax.legend(loc="best")
    fig.tight_layout()

    out_path = Path(args.out).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300)
    print(f"✅ Saved Figure 5.2 to: {out_path}")
    print(f"Runs used: {len(records)} | Skipped: {skipped}")
    print("\nCoverage (count per profile × condition):")
    print(agg.pivot(index="profile", columns="condition", values="count").fillna(0).astype(int))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
