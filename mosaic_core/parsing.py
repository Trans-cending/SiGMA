"""Small parsing helpers shared across MOSAIC stages."""
from __future__ import annotations

import numpy as np


def safe_float_list(values, n, default=None):
    default = default if default is not None else [0.0] * n
    try:
        values = list(values)
    except Exception:
        values = list(default)
    if len(values) < n:
        values = values + list(default)[len(values):]
    return [float(v) for v in values[:n]]



def rotation_dict_from_pitch_yaw_roll(values):
    vals = np.asarray(values, dtype=float).reshape(-1)
    if vals.shape[0] < 3:
        raise ValueError(f"rotation expects [pitch,yaw,roll], got {values!r}")
    return {"pitch": float(vals[0]), "yaw": float(vals[1]), "roll": float(vals[2])}
