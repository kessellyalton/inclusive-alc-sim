from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple
import math
import random


def clip(x: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))


def sigmoid(x: float) -> float:
    # numerically stable-ish for our ranges
    return 1.0 / (1.0 + math.exp(-x))


@dataclass(frozen=True)
class DisabilityParams:
    # Baselines
    c0: float          # baseline cognitive load
    rt0: float         # baseline response time (seconds)
    # Modality penalties (cost added to cognitive load)
    modality_penalty: Dict[str, float]
    # Reading / audio effectiveness modifiers
    visual_penalty: float  # extra penalty when modality is visual
    audio_penalty: float   # extra penalty when modality is audio


DISABILITY_LIBRARY: Dict[str, DisabilityParams] = {
    "none": DisabilityParams(
        c0=0.15,  # lowered from 0.25 for better separation
        rt0=1.2,
        modality_penalty={"visual": 0.00, "audio": 0.00, "multimodal": -0.02},
        visual_penalty=0.00,
        audio_penalty=0.00,
    ),
    "dyslexia": DisabilityParams(
        c0=0.40,  # raised from 0.35 for better separation
        rt0=1.6,
        modality_penalty={"visual": 0.08, "audio": -0.02, "multimodal": 0.00},
        visual_penalty=0.06,   # reading-heavy cost
        audio_penalty=0.00,
    ),
    "hearing_impairment": DisabilityParams(
        c0=0.35,  # raised from 0.32 for better separation
        rt0=1.3,
        modality_penalty={"visual": 0.00, "audio": 0.10, "multimodal": 0.02},
        visual_penalty=0.00,
        audio_penalty=0.08,    # audio less effective/more effort
    ),
    "low_vision": DisabilityParams(
        c0=0.42,  # raised from 0.34 for better separation
        rt0=1.5,
        modality_penalty={"visual": 0.10, "audio": -0.01, "multimodal": 0.02},
        visual_penalty=0.08,   # visual-only harder
        audio_penalty=0.00,
    ),
}


@dataclass
class LearnerStateVars:
    k: float  # knowledge
    c: float  # cognitive load
    e: float  # error likelihood
    disability_profile: str
    step: int = 0


@dataclass(frozen=True)
class ModelWeights:
    # Knowledge update
    beta: float = 0.03  # penalty from cognitive load (reduced from 0.12 to allow learning)

    # Cognitive load weights
    w1: float = 0.12    # difficulty contribution
    w2: float = 0.10    # pacing contribution
    w3: float = 0.20    # modality cost weight (raised from 0.18 for better differentiation)

    # Error probability (sigmoid)
    gamma0: float = 0.2
    gamma1: float = 2.0
    gamma2: float = 2.0

    # Response time
    eta1: float = 0.7   # difficulty effect (seconds)
    eta2: float = 0.8   # cognitive load effect (seconds)


def difficulty_level_to_unit(difficulty_level: int) -> float:
    # map integer level to [0,1], assuming levels 0..5 typical
    return clip(difficulty_level / 5.0, 0.0, 1.0)


def pacing_to_unit(pacing: float) -> float:
    # pacing ~ 0.5..1.5, map to [0,1] "demand"
    # slower pacing => lower demand
    # normal(1.0) -> 0.5; fast(1.5)->1.0; slow(0.5)->0.0
    return clip((pacing - 0.5) / 1.0, 0.0, 1.0)


def modality_cost(modality: str, d: DisabilityParams) -> float:
    base = d.modality_penalty.get(modality, 0.0)
    if modality == "visual":
        base += d.visual_penalty
    if modality == "audio":
        base += d.audio_penalty
    return base


def alpha_for_action(modality: str, pacing: float, difficulty_u: float, d: DisabilityParams) -> float:
    """
    α(a_t): effective learning rate as a function of action and disability.
    We assume:
    - very high difficulty reduces effective learning rate
    - very fast pacing reduces learning rate
    - modality mismatches reduce learning rate
    """
    # base learning rate (raised from 0.06 to allow meaningful learning)
    alpha0 = 0.10

    pace_demand = pacing_to_unit(pacing)         # higher means faster
    diff_penalty = 0.5 * difficulty_u            # harder => lower α
    pace_penalty = 0.35 * pace_demand            # faster => lower α

    # modality mismatch reduces learning: reuse cost as a proxy
    m_cost = modality_cost(modality, d)
    mismatch_penalty = clip(m_cost, 0.0, 0.25)   # only penalize positive costs

    alpha = alpha0 * (1.0 - diff_penalty) * (1.0 - pace_penalty) * (1.0 - mismatch_penalty)
    return clip(alpha, 0.01, 0.15)  # widened range from (0.005, 0.08)


def step_dynamics(
    s: LearnerStateVars,
    modality: str,
    pacing: float,
    difficulty_level: int,
    weights: ModelWeights,
    rng: random.Random,
) -> Tuple[LearnerStateVars, bool, float]:
    """
    Applies the equations described in Chapter 4.4:
      k_{t+1} = clip(k_t + α(a_t)(1-k_t) - β c_t + ε_k)
      c_{t+1} = clip(c0(d) + w1*difficulty + w2*pacing + w3*modality_cost + ε_c)
      e_{t+1} = σ(γ0 - γ1*k_{t+1} + γ2*c_{t+1})
      rt_t    = rt0(d) + η1*difficulty + η2*c_t + ε_rt

    Returns: (new_state, correct, response_time)
    """
    d = DISABILITY_LIBRARY.get(s.disability_profile, DISABILITY_LIBRARY["none"])
    difficulty_u = difficulty_level_to_unit(difficulty_level)
    pacing_u = pacing_to_unit(pacing)

    # Noise terms
    eps_k = rng.uniform(-0.01, 0.01)
    eps_c = rng.uniform(-0.02, 0.02)
    eps_rt = rng.uniform(-0.15, 0.15)

    # Cognitive load (uses action + disability)
    c_next = (
        d.c0
        + weights.w1 * difficulty_u
        + weights.w2 * pacing_u
        + weights.w3 * modality_cost(modality, d)
        + eps_c
    )
    c_next = clip(c_next, 0.0, 1.0)

    # Knowledge update
    alpha = alpha_for_action(modality, pacing, difficulty_u, d)
    k_next = s.k + alpha * (1.0 - s.k) - weights.beta * s.c + eps_k
    k_next = clip(k_next, 0.0, 1.0)

    # Error likelihood after update
    e_next = sigmoid(weights.gamma0 - weights.gamma1 * k_next + weights.gamma2 * c_next)
    e_next = clip(e_next, 0.0, 1.0)

    # Sample correctness from error probability
    correct = rng.random() > e_next

    # Response time
    rt = d.rt0 + weights.eta1 * difficulty_u + weights.eta2 * s.c + eps_rt
    rt = max(0.2, rt)

    s2 = LearnerStateVars(
        k=k_next,
        c=c_next,
        e=e_next,
        disability_profile=s.disability_profile,
        step=s.step + 1,
    )
    return s2, correct, rt
