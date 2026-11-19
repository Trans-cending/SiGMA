"""Canonical lightweight coordinate transforms shared by MOSAIC stages."""
from __future__ import annotations
import math
import numpy as np


def carla_to_sionna_xyz(value, dtype=np.float32) -> np.ndarray:
    if isinstance(value, dict):
        x, y, z = value.get("x", 0.0), value.get("y", 0.0), value.get("z", 0.0)
    else:
        x, y, z = value[0], value[1], value[2]
    return np.asarray([float(x), -float(y), float(z)], dtype=dtype)


def carla_to_blender_location(value) -> tuple[float, float, float]:
    if isinstance(value, dict):
        x, y, z = value.get("x", 0.0), value.get("y", 0.0), value.get("z", 0.0)
    else:
        x, y, z = value[0], value[1], value[2]
    return float(x), -float(y), float(z)


def normalize_carla_rotation(value) -> dict[str, float]:
    if isinstance(value, dict):
        return {k: float(value.get(k, 0.0)) for k in ("pitch", "yaw", "roll")}
    vals = list(value or [0.0, 0.0, 0.0])
    vals += [0.0] * (3-len(vals))
    return {"pitch": float(vals[0]), "yaw": float(vals[1]), "roll": float(vals[2])}


def carla_forward_vector(rotation) -> np.ndarray:
    r = normalize_carla_rotation(rotation)
    pitch = math.radians(r["pitch"])
    yaw = math.radians(r["yaw"])
    return np.asarray([
        math.cos(pitch) * math.cos(yaw),
        -math.cos(pitch) * math.sin(yaw),
        math.sin(pitch),
    ], dtype=np.float32)
