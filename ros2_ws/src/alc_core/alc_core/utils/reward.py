from __future__ import annotations
from dataclasses import dataclass
from typing import Dict

from .models import DISABILITY_LIBRARY, DisabilityParams


@dataclass(frozen=True)
class RewardWeights:
    l1: float = 3.0   # Δk
    l2: float = 1.0   # cognitive load penalty
    l3: float = 1.0   # error likelihood penalty
    l4: float = 2.0   # overload penalty
    l5: float = 1.0   # accessibility match

    c_max: float = 0.75


def accessibility_match(modality: str, disability_profile: str) -> float:
    """
    Returns +1 if modality is a good match for disability profile, else 0 or negative.
    This is deliberately simple and explainable for the dissertation.
    """
    d: DisabilityParams = DISABILITY_LIBRARY.get(disability_profile, DISABILITY_LIBRARY["none"])

    # Heuristic "match": prefer lower modality penalty
    penalty = d.modality_penalty.get(modality, 0.0)

    # multimodal often helps; allow bonus if not penalized
    if modality == "multimodal" and penalty <= 0.02:
        return 1.0

    # if penalty is negative or zero, that's a match
    if penalty <= 0.0:
        return 1.0

    # penalized modality
    return -0.5


def compute_reward(
    delta_k: float,
    c: float,
    e: float,
    modality: str,
    disability_profile: str,
    w: RewardWeights = RewardWeights(),
) -> float:
    overload = 1.0 if c > w.c_max else 0.0
    match = accessibility_match(modality, disability_profile)

    r = (
        w.l1 * delta_k
        - w.l2 * c
        - w.l3 * e
        - w.l4 * overload
        + w.l5 * match
    )
    return float(r)
