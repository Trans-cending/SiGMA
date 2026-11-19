"""Shared MOSAIC dataset naming and path helpers.

Canonical physical layout:
    <dataset>/<map>/stage*/000000/...

Only six-digit physical trajectory directory names are supported. Map identity
is derived from the map-scoped parent directory.
"""
from __future__ import annotations
from pathlib import Path
import re


def frame_idx_from_name(frame_name: str) -> int:
    text = str(frame_name)
    matches = re.findall(r"(\d{6})(?!.*\d)", text)
    if matches:
        return int(matches[-1])
    digits = re.findall(r"\d+", text)
    if digits:
        return int(digits[-1])
    raise ValueError(f"Cannot infer frame_idx from frame name: {frame_name!r}")


def frame_idx_from_name_or_none(frame_name: str):
    try:
        matches = re.findall(r"(\d{6})(?!.*\d)", str(frame_name))
        if not matches:
            return None
        return int(matches[-1])
    except Exception:
        return None


def parse_town_trajectory_name(value: str | Path) -> tuple[str, int]:
    path = Path(value)
    name = path.name
    if not re.fullmatch(r"\d{6}", name):
        raise ValueError(
            f"Trajectory directory must be the canonical six-digit name NNNNNN: {value}"
        )
    stage_parent = path.parent
    if stage_parent.name not in {"stage1_multimodal", "stage2_xml", "stage3_radio"}:
        raise ValueError(
            "Canonical trajectory directory must be under a stage root: "
            f"<dataset>/<map>/stage*/NNNNNN, got {value}"
        )
    map_name = stage_parent.parent.name
    if not map_name:
        raise ValueError(f"Cannot infer map from canonical trajectory path: {value}")
    return map_name, int(name)


def stage1_world_state_dir(traj_dir: str | Path) -> Path:
    return Path(traj_dir) / "world_state"


def stage1_world_state_candidates(traj_dir: str | Path, frame_name: str) -> list[Path]:
    traj_dir = Path(traj_dir)
    parse_town_trajectory_name(traj_dir)
    frame_idx = frame_idx_from_name(frame_name)
    return [traj_dir / "world_state" / f"world_state_{frame_idx:06d}.yaml"]


def radio_node_alias(kind: str, ordinal: int) -> str:
    prefix = {"rsu": "r", "cav": "c", "uav": "u", "static_rx": "s"}.get(str(kind).lower())
    if prefix is None:
        raise ValueError(f"Unsupported MOSAIC radio node kind: {kind!r}")
    return f"{prefix}{int(ordinal):03d}"
