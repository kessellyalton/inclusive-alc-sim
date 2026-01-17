#!/usr/bin/env python3
"""
Parameter Sanity Check Script
=============================
Compares learner dynamics with CURRENT vs SUGGESTED parameters.

This script replicates the core equations from models.py and reward.py
to show why knowledge collapses with current parameters.

Run: python3 scripts/parameter_sanity_check.py
"""

import math
import random
from dataclasses import dataclass
from typing import Dict, List

# =============================================================================
# HELPER FUNCTIONS (copied from models.py)
# =============================================================================

def clip(x: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))

def sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


# =============================================================================
# PARAMETER SETS
# =============================================================================

@dataclass
class Params:
    """All tunable parameters in one place."""
    name: str
    
    # Disability baselines (c0)
    c0_none: float
    c0_dyslexia: float
    c0_hearing: float
    c0_lowvision: float
    
    # Knowledge update
    alpha0: float       # base learning rate
    alpha_min: float    # min learning rate (clipped)
    alpha_max: float    # max learning rate (clipped)
    beta: float         # cognitive load penalty on knowledge
    
    # Cognitive load weights
    w1: float           # difficulty contribution
    w2: float           # pacing contribution
    w3: float           # modality cost weight
    
    # Error probability
    gamma0: float
    gamma1: float
    gamma2: float
    
    # Reward weights
    l1: float           # Δk reward
    l2: float           # cognitive load penalty
    l3: float           # error penalty
    l4: float           # overload penalty
    l5: float           # accessibility match
    c_max: float        # overload threshold


# CURRENT parameters (from your code)
CURRENT = Params(
    name="CURRENT",
    c0_none=0.25,
    c0_dyslexia=0.35,
    c0_hearing=0.32,
    c0_lowvision=0.34,
    alpha0=0.06,
    alpha_min=0.005,
    alpha_max=0.08,
    beta=0.12,          # <-- THE PROBLEM
    w1=0.12,
    w2=0.10,
    w3=0.18,
    gamma0=0.2,
    gamma1=2.0,
    gamma2=2.0,
    l1=3.0,
    l2=1.0,
    l3=1.0,
    l4=2.0,
    l5=1.0,
    c_max=0.75,
)

# SUGGESTED parameters (fixes)
SUGGESTED = Params(
    name="SUGGESTED",
    c0_none=0.15,       # lower baseline for "none"
    c0_dyslexia=0.40,   # more separation
    c0_hearing=0.35,
    c0_lowvision=0.42,
    alpha0=0.10,        # higher base learning rate
    alpha_min=0.01,
    alpha_max=0.15,
    beta=0.03,          # <-- FIXED: much lower penalty
    w1=0.12,
    w2=0.10,
    w3=0.20,            # slightly higher for more modality differentiation
    gamma0=0.2,
    gamma1=2.0,
    gamma2=2.0,
    l1=3.0,
    l2=0.8,             # slightly lower constant penalty
    l3=0.8,
    l4=2.0,
    l5=1.5,             # higher match bonus
    c_max=0.75,
)


# =============================================================================
# SIMULATION FUNCTIONS
# =============================================================================

def get_c0(disability: str, p: Params) -> float:
    return {
        "none": p.c0_none,
        "dyslexia": p.c0_dyslexia,
        "hearing_impairment": p.c0_hearing,
        "low_vision": p.c0_lowvision,
    }.get(disability, p.c0_none)


def modality_penalty(modality: str, disability: str) -> float:
    """Returns the modality penalty for a given disability."""
    penalties = {
        "none": {"visual": 0.0, "audio": 0.0, "multimodal": -0.02},
        "dyslexia": {"visual": 0.14, "audio": -0.02, "multimodal": 0.0},  # visual + visual_penalty
        "hearing_impairment": {"visual": 0.0, "audio": 0.18, "multimodal": 0.02},
        "low_vision": {"visual": 0.18, "audio": -0.01, "multimodal": 0.02},
    }
    return penalties.get(disability, penalties["none"]).get(modality, 0.0)


def accessibility_match(modality: str, disability: str) -> float:
    """Returns +1 for good match, -0.5 for mismatch."""
    penalty = modality_penalty(modality, disability)
    if modality == "multimodal" and penalty <= 0.02:
        return 1.0
    if penalty <= 0.0:
        return 1.0
    return -0.5


def simulate_episode(
    disability: str,
    modality: str,
    difficulty: int,
    pacing: float,
    p: Params,
    n_steps: int = 50,
    seed: int = 42,
) -> Dict[str, List[float]]:
    """
    Simulate n_steps of learner dynamics.
    Returns trajectories of k, c, e, reward.
    """
    rng = random.Random(seed)
    
    # Initial state
    k = 0.5
    c = get_c0(disability, p)
    
    # Normalize inputs
    difficulty_u = clip(difficulty / 5.0)
    pacing_u = clip((pacing - 0.5) / 1.0)
    
    # Trajectories
    ks, cs, es, rs = [k], [c], [], []
    
    for step in range(n_steps):
        # Noise
        eps_k = rng.uniform(-0.01, 0.01)
        eps_c = rng.uniform(-0.02, 0.02)
        
        # Cognitive load
        m_cost = modality_penalty(modality, disability)
        c_next = (
            get_c0(disability, p)
            + p.w1 * difficulty_u
            + p.w2 * pacing_u
            + p.w3 * m_cost
            + eps_c
        )
        c_next = clip(c_next)
        
        # Alpha (learning rate)
        diff_penalty = 0.5 * difficulty_u
        pace_penalty = 0.35 * pacing_u
        mismatch_penalty = clip(m_cost, 0.0, 0.25)
        alpha = p.alpha0 * (1 - diff_penalty) * (1 - pace_penalty) * (1 - mismatch_penalty)
        alpha = clip(alpha, p.alpha_min, p.alpha_max)
        
        # Knowledge update
        k_next = k + alpha * (1 - k) - p.beta * c + eps_k
        k_next = clip(k_next)
        
        # Error probability
        e_next = sigmoid(p.gamma0 - p.gamma1 * k_next + p.gamma2 * c_next)
        
        # Reward
        delta_k = k_next - k
        overload = 1.0 if c_next > p.c_max else 0.0
        match = accessibility_match(modality, disability)
        r = p.l1 * delta_k - p.l2 * c_next - p.l3 * e_next - p.l4 * overload + p.l5 * match
        
        # Update state
        k, c = k_next, c_next
        
        # Store
        ks.append(k)
        cs.append(c)
        es.append(e_next)
        rs.append(r)
    
    return {"k": ks, "c": cs, "e": es, "r": rs}


def print_diagnostic(disability: str, modality: str, difficulty: int, pacing: float, p: Params):
    """Print a single-step diagnostic breakdown."""
    difficulty_u = clip(difficulty / 5.0)
    pacing_u = clip((pacing - 0.5) / 1.0)
    m_cost = modality_penalty(modality, disability)
    
    c0 = get_c0(disability, p)
    c = c0 + p.w1 * difficulty_u + p.w2 * pacing_u + p.w3 * m_cost
    c = clip(c)
    
    diff_penalty = 0.5 * difficulty_u
    pace_penalty = 0.35 * pacing_u
    mismatch_penalty = clip(m_cost, 0.0, 0.25)
    alpha = p.alpha0 * (1 - diff_penalty) * (1 - pace_penalty) * (1 - mismatch_penalty)
    alpha = clip(alpha, p.alpha_min, p.alpha_max)
    
    k = 0.5
    learning_gain = alpha * (1 - k)
    load_penalty = p.beta * c
    net_change = learning_gain - load_penalty
    
    print(f"\n{'='*60}")
    print(f"DIAGNOSTIC: {p.name}")
    print(f"{'='*60}")
    print(f"Disability: {disability}, Modality: {modality}")
    print(f"Difficulty: {difficulty} (u={difficulty_u:.2f}), Pacing: {pacing} (u={pacing_u:.2f})")
    print(f"\nCognitive Load Breakdown:")
    print(f"  c0 (baseline)           = {c0:.3f}")
    print(f"  + w1 * difficulty       = {p.w1 * difficulty_u:.3f}")
    print(f"  + w2 * pacing           = {p.w2 * pacing_u:.3f}")
    print(f"  + w3 * modality_cost    = {p.w3 * m_cost:.3f}")
    print(f"  ---------------------------------")
    print(f"  Total c                 = {c:.3f}")
    print(f"\nKnowledge Update (at k=0.5):")
    print(f"  alpha (learning rate)   = {alpha:.4f}")
    print(f"  Learning gain: α(1-k)   = {learning_gain:+.4f}")
    print(f"  Load penalty: β*c       = {load_penalty:+.4f}")
    print(f"  ---------------------------------")
    print(f"  NET CHANGE              = {net_change:+.4f}")
    if net_change < 0:
        print(f"  ⚠️  KNOWLEDGE WILL DECREASE!")
    else:
        print(f"  ✓  Knowledge can increase")


def draw_ascii_trajectory(values: List[float], width: int = 50, height: int = 10, 
                          min_val: float = 0.0, max_val: float = 1.0, label: str = ""):
    """Draw a simple ASCII chart of a trajectory."""
    # Normalize values to height
    def to_row(v):
        normalized = (v - min_val) / (max_val - min_val) if max_val != min_val else 0.5
        return int((1 - normalized) * (height - 1))
    
    # Sample values to fit width
    step = max(1, len(values) // width)
    sampled = [values[i] for i in range(0, len(values), step)][:width]
    
    # Create grid
    grid = [[' ' for _ in range(len(sampled))] for _ in range(height)]
    
    # Plot points
    for col, val in enumerate(sampled):
        row = to_row(val)
        row = max(0, min(height - 1, row))
        grid[row][col] = '█'
    
    # Print with y-axis labels
    print(f"\n{label}")
    for i, row in enumerate(grid):
        if i == 0:
            y_label = f"{max_val:.2f}"
        elif i == height - 1:
            y_label = f"{min_val:.2f}"
        elif i == height // 2:
            y_label = f"{(max_val + min_val) / 2:.2f}"
        else:
            y_label = "    "
        print(f"  {y_label:>5} │{''.join(row)}│")
    print(f"        └{'─' * len(sampled)}┘")
    print(f"         0{' ' * (len(sampled) - 4)}step {len(values)-1}")


def main():
    # Configuration for test scenarios
    scenarios = [
        ("none", "multimodal", 3, 1.0),
        ("dyslexia", "visual", 3, 1.0),      # mismatched
        ("dyslexia", "audio", 3, 1.0),       # matched
        ("hearing_impairment", "audio", 3, 1.0),  # mismatched
        ("hearing_impairment", "visual", 3, 1.0), # matched
        ("low_vision", "visual", 3, 1.0),    # mismatched
    ]
    
    print("\n" + "="*70)
    print("PARAMETER SANITY CHECK: CURRENT vs SUGGESTED")
    print("="*70)
    
    # Print diagnostics for key scenario
    print_diagnostic("dyslexia", "visual", 3, 1.0, CURRENT)
    print_diagnostic("dyslexia", "visual", 3, 1.0, SUGGESTED)
    
    # Show summary statistics
    print("\n" + "="*70)
    print("SUMMARY: Final Knowledge (k) after 50 steps")
    print("="*70)
    print(f"{'Scenario':<35} {'CURRENT':>10} {'SUGGESTED':>10} {'Δ':>10}")
    print("-"*70)
    
    for disability, modality, diff, pace in scenarios:
        result_curr = simulate_episode(disability, modality, diff, pace, CURRENT, n_steps=50)
        result_sugg = simulate_episode(disability, modality, diff, pace, SUGGESTED, n_steps=50)
        
        k_curr = result_curr["k"][-1]
        k_sugg = result_sugg["k"][-1]
        delta = k_sugg - k_curr
        
        match_status = "✓" if accessibility_match(modality, disability) > 0 else "✗"
        scenario_name = f"{disability}-{modality} {match_status}"
        print(f"{scenario_name:<35} {k_curr:>10.3f} {k_sugg:>10.3f} {delta:>+10.3f}")
    
    print("\n" + "="*70)
    print("SUMMARY: Mean Reward over 50 steps")
    print("="*70)
    print(f"{'Scenario':<35} {'CURRENT':>10} {'SUGGESTED':>10} {'Δ':>10}")
    print("-"*70)
    
    for disability, modality, diff, pace in scenarios:
        result_curr = simulate_episode(disability, modality, diff, pace, CURRENT, n_steps=50)
        result_sugg = simulate_episode(disability, modality, diff, pace, SUGGESTED, n_steps=50)
        
        r_curr = sum(result_curr["r"]) / len(result_curr["r"])
        r_sugg = sum(result_sugg["r"]) / len(result_sugg["r"])
        delta = r_sugg - r_curr
        
        match_status = "✓" if accessibility_match(modality, disability) > 0 else "✗"
        scenario_name = f"{disability}-{modality} {match_status}"
        print(f"{scenario_name:<35} {r_curr:>10.3f} {r_sugg:>10.3f} {delta:>+10.3f}")
    
    # ASCII Trajectories for a key comparison
    print("\n" + "="*70)
    print("KNOWLEDGE TRAJECTORIES (dyslexia + visual, mismatched)")
    print("="*70)
    
    result_curr = simulate_episode("dyslexia", "visual", 3, 1.0, CURRENT, n_steps=50)
    result_sugg = simulate_episode("dyslexia", "visual", 3, 1.0, SUGGESTED, n_steps=50)
    
    draw_ascii_trajectory(result_curr["k"], label="CURRENT: Knowledge collapses to 0")
    draw_ascii_trajectory(result_sugg["k"], label="SUGGESTED: Knowledge grows toward 1")
    
    print("\n" + "="*70)
    print("REWARD TRAJECTORIES (dyslexia + visual, mismatched)")
    print("="*70)
    
    r_min = min(min(result_curr["r"]), min(result_sugg["r"]))
    r_max = max(max(result_curr["r"]), max(result_sugg["r"]))
    
    draw_ascii_trajectory(result_curr["r"], min_val=r_min, max_val=r_max, 
                          label="CURRENT: Rewards are negative")
    draw_ascii_trajectory(result_sugg["r"], min_val=r_min, max_val=r_max,
                          label="SUGGESTED: Rewards improve over time")
    
    # Key insight
    print("\n" + "="*70)
    print("KEY INSIGHT")
    print("="*70)
    print("""
With CURRENT parameters:
  - beta=0.12 creates a penalty of ~0.048-0.060 per step
  - alpha maxes out at 0.06, giving a gain of ~0.030 per step
  - NET: Knowledge DECREASES every step → collapse to 0

With SUGGESTED parameters:
  - beta=0.03 creates a penalty of ~0.012-0.015 per step
  - alpha can reach 0.10+, giving a gain of ~0.050 per step
  - NET: Knowledge INCREASES over time → learning happens

The fix: Reduce beta from 0.12 to 0.03 (or increase alpha0 to 0.10+)
""")
    
    print("\n" + "="*70)
    print("CONDITION SEPARATION (comparing disabilities under matched modalities)")
    print("="*70)
    
    matched_scenarios = [
        ("none", "multimodal"),
        ("dyslexia", "audio"),
        ("hearing_impairment", "visual"),
        ("low_vision", "audio"),
    ]
    
    print(f"{'Disability':<25} {'Modality':<12} {'k_final CURR':>12} {'k_final SUGG':>12}")
    print("-"*70)
    
    for disability, modality in matched_scenarios:
        result_curr = simulate_episode(disability, modality, 3, 1.0, CURRENT, n_steps=50)
        result_sugg = simulate_episode(disability, modality, 3, 1.0, SUGGESTED, n_steps=50)
        print(f"{disability:<25} {modality:<12} {result_curr['k'][-1]:>12.3f} {result_sugg['k'][-1]:>12.3f}")
    
    print("\nWith SUGGESTED params, you get more separation between conditions.")
    print("With CURRENT params, everything collapses to ~0, so no separation is visible.")


if __name__ == "__main__":
    main()
