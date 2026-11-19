"""Deterministic paper metrics and 32/64-beam subset labels.

Implement the supplied paper's equations (3) and (5) using computed *linear*
candidate powers.
"""
from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np


def beam_subset(powers_64: np.ndarray, size: int) -> tuple[np.ndarray, np.ndarray]:
    """Return selected powers and freshly computed zero-based local labels.

    The 32-beam setting retains original zero-based indices 0, 2, ..., 62.
    Labels are recomputed by argmax over the selected candidate powers.
    """
    powers = np.asarray(powers_64, dtype=np.float64)
    if powers.ndim != 2 or powers.shape[1] != 64:
        raise ValueError("powers_64 must have shape [frames,64]")
    if size not in (32, 64):
        raise ValueError("paper beam codebook size must be 32 or 64")
    if not np.isfinite(powers).all() or np.any(powers < 0):
        raise ValueError("candidate powers must be finite, nonnegative, linear powers")
    subset = powers[:, ::2] if size == 32 else powers.copy()
    return subset, np.argmax(subset, axis=1)


def ranking_metrics(
    powers: np.ndarray,
    rankings: Sequence[Sequence[int]],
    scan_budgets: Iterable[int],
    *,
    valid: Sequence[bool] | None = None,
) -> dict[str, object]:
    """Compute NG@k and Top-k accuracy over all frames, including failures.

    Empty rankings or ``valid=False`` represent detection/association/prediction
    failures and contribute zero. Zero-power full scans also contribute zero
    normalized gain and receive a separate zero-reference count.
    Candidate IDs are zero based. Rankings may be shorter than the scan budget.
    """
    p = np.asarray(powers, dtype=np.float64)
    if p.ndim != 2 or p.shape[0] == 0 or p.shape[1] == 0:
        raise ValueError("powers must be nonempty [frames,candidates]")
    if not np.isfinite(p).all() or np.any(p < 0):
        raise ValueError("powers must be finite, nonnegative, linear powers")
    if len(rankings) != p.shape[0]:
        raise ValueError("one ranking is required for each frame")
    ok = np.ones(p.shape[0], dtype=bool) if valid is None else np.asarray(valid, dtype=bool)
    if ok.shape != (p.shape[0],):
        raise ValueError("valid must have one value per frame")
    ordered: list[list[int]] = []
    for ranking in rankings:
        row = list(ranking)
        if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, np.integer))
               or not 0 <= v < p.shape[1] for v in row):
            raise ValueError("rankings must contain zero-based integer candidate IDs")
        if len(row) != len(set(row)):
            raise ValueError("rankings cannot repeat candidates")
        ordered.append(row)
    reference = np.max(p, axis=1)
    labels = np.argmax(p, axis=1)
    metrics: dict[str, object] = {}
    for k in scan_budgets:
        if isinstance(k, bool) or not isinstance(k, (int, np.integer)) or not 1 <= k <= p.shape[1]:
            raise ValueError("scan budgets must be positive integers within the codebook size")
        gains = np.zeros(p.shape[0], dtype=np.float64)
        hits = np.zeros(p.shape[0], dtype=np.float64)
        for i, ranking in enumerate(ordered):
            candidates = ranking[:int(k)] if ok[i] else []
            if candidates:
                hits[i] = float(int(labels[i]) in candidates)
                if reference[i] > 0:
                    gains[i] = np.max(p[i, candidates]) / reference[i]
        metrics[str(int(k))] = {
            "normalized_gain": float(gains.mean()),
            "top_k_accuracy": float(hits.mean()),
        }
    return {
        "frames": int(p.shape[0]), "candidates": int(p.shape[1]),
        "failed_frames": int(sum(not ok[i] or not ordered[i] for i in range(p.shape[0]))),
        "zero_reference_frames": int(np.count_nonzero(reference == 0)),
        "scan_budgets": metrics,
    }
