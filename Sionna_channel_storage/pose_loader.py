# -*- coding: utf-8 -*-
from __future__ import annotations
from .sionna_config import *

from mosaic_core.coordinates import carla_to_sionna_xyz as _core_carla_to_sionna_xyz

carla_to_sionna_xyz = lambda loc_carla: _core_carla_to_sionna_xyz(loc_carla, dtype=float)

def normalize_rotation(rot) -> dict:
    if isinstance(rot, dict):
        return {
            "pitch": float(rot.get("pitch", 0.0)),
            "yaw": float(rot.get("yaw", 0.0)),
            "roll": float(rot.get("roll", 0.0)),
        }
    if isinstance(rot, (list, tuple, np.ndarray)) and len(rot) >= 3:
        return {"pitch": float(rot[0]), "yaw": float(rot[1]), "roll": float(rot[2])}
    raise TypeError(f"Missing/invalid CARLA rotation: {rot!r}")

def carla_rot_to_sionna_forward(rot_carla) -> np.ndarray:
    rot = normalize_rotation(rot_carla)
    yaw = math.radians(-float(rot["yaw"]))
    pitch = math.radians(float(rot["pitch"]))
    v = np.array([
        math.cos(yaw) * math.cos(pitch),
        math.sin(yaw) * math.cos(pitch),
        math.sin(pitch),
    ], dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 0 else np.array([1.0, 0.0, 0.0], dtype=float)

def read_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}

