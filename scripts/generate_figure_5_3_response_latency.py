#!/usr/bin/env python3
"""
Generate Figure 5.3: Mean learner response latency across experimental conditions.

This figure emphasizes interaction efficiency (how well the tutor adapts to learner needs)
rather than raw speed. Lower latency may indicate better cognitive load management and
more appropriate instructional pacing.

Uses per-step response_time logged by logger_node into CSV rows (TrialOutcome.response_time).
See CODE_DOCUMENTATION.md for CSV columns and logger design.

Visual style matches Figure 5.2 (bar chart with error bars).
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pandas as pd
import matplotlib.pyplot as plt


KNOWN_PROFILES = ["none", "dyslexia", "hearing_impairment", "low_vision"]
# We allow common aliases coming from different scripts / metadata conventions
COND_ALIASES = {
    "fixed": "baseline",
    "baseline": "baseline",
    "traditional": "baseline",
    "adaptive": "adaptive",
    "inclusive_adaptive": "inclusive_adaptive",
    "inclusive": "inclusive_adaptive",
}


def is_derived_file(p: Path) -> bool:
    name = p.name.lower()
    bad = ["summary", "table_", "appendix", "condition_summary", "run_metrics", "_metadata"]
    return any(x in name for x in bad)


def normalize_condition(raw: str) -> Optional[str]:
    if not raw:
        return None
    raw = raw.strip().lower()
    return COND_ALIASES.get(raw, raw)


def extract_profile_from_path(path: Path) -> Optional[str]:
    s = str(path).lower()
    for prof in KNOWN_PROFILES:
        if re.search(rf"\b{re.escape(prof)}\b", s):
            return prof
    # handle patterns like "inclusive_low_vision"
    for prof in KNOWN_PROFILES:
        if f"inclusive_{prof}" in s or f"adaptive_{prof}" in s or f"profile_{prof}" in s:
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
    # Condition sources (prefer explicit column)
    cond = None
    if "condition" in df.columns:
        c = str(df["condition"].iloc[0]).strip().lower()
        cond = normalize_condition(c)

    if not cond:
        cond = extract_condition_from_metadata(csv_path)

    # Profile sources (prefer explicit param/state columns)
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
        default=str(Path.home() / "dev" / "inclusive-alc-sim" / "figures" / "figure_5_3_response_latency.png"),
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

        if "response_time" not in df.columns:
            skipped += 1
            continue

        if len(df) < args.min_rows:
            skipped += 1
            continue

        cond, prof = extract_run_fields(df, csv_path)
        if cond not in ["baseline", "adaptive", "inclusive_adaptive"]:
            skipped += 1
            continue
        if prof not in KNOWN_PROFILES:
            skipped += 1
            continue

        # Mean response latency per run
        rt = pd.to_numeric(df["response_time"], errors="coerce").dropna()
        if rt.empty:
            skipped += 1
            continue

        records.append(
            {
                "csv": str(csv_path),
                "condition": cond,
                "profile": prof,
                "mean_response_time": float(rt.mean()),
            }
        )

    if not records:
        raise SystemExit("No usable runs found (check log dirs, CSV columns, and conditions).")

    data = pd.DataFrame.from_records(records)

    # Aggregate across runs (seeds)
    agg = (
        data.groupby(["profile", "condition"])["mean_response_time"]
        .agg(["mean", "std", "count"])
        .reset_index()
    )

    # Order profiles and conditions for plotting
    prof_order = KNOWN_PROFILES
    cond_order = ["baseline", "adaptive", "inclusive_adaptive"]
    agg["profile"] = pd.Categorical(agg["profile"], categories=prof_order, ordered=True)
    agg["condition"] = pd.Categorical(agg["condition"], categories=cond_order, ordered=True)
    agg = agg.sort_values(["profile", "condition"])

    # Pivot for plotting
    pivot_mean = agg.pivot(index="profile", columns="condition", values="mean")
    pivot_std = agg.pivot(index="profile", columns="condition", values="std")

    # Plot - matching Figure 5.2 visual style
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    x = range(len(prof_order))
    width = 0.25  # Bar width for 3 bars per profile

    # Labels matching Figure 5.2 style
    labels = {
        "baseline": "Baseline Tutor",
        "adaptive": "Adaptive Tutor",
        "inclusive_adaptive": "Inclusive Adaptive Tutor",
    }
    
    # Profile display labels for x-axis
    profile_labels = {
        "none": "No disability",
        "dyslexia": "Dyslexia",
        "hearing_impairment": "Hearing impairment",
        "low_vision": "Low vision",
    }

    for i, cond in enumerate(cond_order):
        if cond not in pivot_mean.columns:
            continue
        y = []
        yerr = []
        for p in prof_order:
            if p in pivot_mean.index and cond in pivot_mean.columns:
                y.append(pivot_mean.loc[p, cond])
                yerr.append(pivot_std.loc[p, cond] if p in pivot_std.index and cond in pivot_std.columns else 0.0)
            else:
                y.append(float("nan"))
                yerr.append(0.0)
        
        # Position bars: center bar at x, left at x-width, right at x+width
        offsets = [v + (i - 1) * width for v in x]
        ax.bar(offsets, y, width=width, yerr=yerr, capsize=3, label=labels.get(cond, cond.replace("_", " ").title()))

    ax.set_xticks(list(x))
    ax.set_xticklabels([profile_labels.get(p, p.replace("_", " ").title()) for p in prof_order])
    ax.set_xlabel("Learner Profile")
    ax.set_ylabel("Mean Response Latency (seconds)")
    ax.set_title("Response Latency by Profile and Condition")
    ax.legend(loc="best")
    fig.tight_layout()

    out_path = Path(args.out).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300)
    print(f"✅ Saved Figure 5.3 to: {out_path}")
    print(f"Runs used: {len(records)} | Skipped: {skipped}")
    print("\nCoverage (count per profile × condition):")
    print(agg.pivot(index="profile", columns="condition", values="count").fillna(0).astype(int))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
