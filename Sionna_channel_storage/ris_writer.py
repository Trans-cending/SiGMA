"""Atomic, cache-safe persistence for optional RIS Stage3 products."""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Mapping

import numpy as np


RIS_OUTPUT_SCHEMA_VERSION = 2


def ris_root(stage3_trajectory_root: Path) -> Path:
    return Path(stage3_trajectory_root) / "ris"


def ris_component_path(
    stage3_trajectory_root: Path, config_id: str, link_id: str, frame_idx: int
) -> Path:
    return (
        ris_root(stage3_trajectory_root)
        / "channels"
        / str(config_id)
        / str(link_id)
        / f"ris_{int(frame_idx):06d}.npz"
    )


def ris_path_path(
    stage3_trajectory_root: Path, leg: str, link_id: str, frame_idx: int
) -> Path:
    if leg not in {"rsu_to_ris", "ris_to_cav"}:
        raise ValueError(f"invalid RIS leg {leg!r}")
    prefix = "br" if leg == "rsu_to_ris" else "ru"
    return ris_root(stage3_trajectory_root) / "paths" / leg / str(link_id) / f"p_{prefix}_{int(frame_idx):06d}.npz"


def _atomic_savez(path: Path, arrays: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + f".tmp.{os.getpid()}")
    with tmp.open("wb") as handle:
        np.savez_compressed(handle, **arrays)
    os.replace(tmp, path)


def save_ris_paths(path: Path, arrays: Mapping[str, Any], *, config_hash: str) -> None:
    payload = dict(arrays)
    payload.update(
        ris_schema_version=np.asarray(RIS_OUTPUT_SCHEMA_VERSION, dtype=np.int16),
        ris_config_hash=np.asarray(str(config_hash)),
    )
    _atomic_savez(Path(path), payload)


def load_ris_paths(path: Path) -> dict[str, Any]:
    """Load a saved scalar RT leg into the existing channel synthesizer shape."""
    with np.load(path, allow_pickle=False) as data:
        coeff = np.asarray(data["coefficient"], dtype=np.complex64)
        delay = np.asarray(data["delay_s"], dtype=np.float32)
        aod = np.asarray(data["aod_rad"], dtype=np.float32)
        aoa = np.asarray(data["aoa_rad"], dtype=np.float32)
        n = int(coeff.shape[0])
        keep = np.asarray(data["dedup_keep"], dtype=bool) if "dedup_keep" in data else np.ones(n, bool)
        path_type = np.asarray(data["path_type"], dtype=np.uint8) if "path_type" in data else np.zeros(n, np.uint8)
        los = np.asarray(data["los"], dtype=bool) if "los" in data else path_type == 0
        doppler = np.asarray(data["doppler_hz"], dtype=np.float32) if "doppler_hz" in data else np.zeros(n, np.float32)
    rx = {
        "a": coeff,
        "tau": delay,
        "theta_t": (np.pi / 2.0 - aod[:, 1]).astype(np.float32),
        "phi_t": aod[:, 0].astype(np.float32),
        "glob_theta_t": (np.pi / 2.0 - aod[:, 1]).astype(np.float32),
        "glob_phi_t": aod[:, 0].astype(np.float32),
        "theta_r": (np.pi / 2.0 - aoa[:, 1]).astype(np.float32),
        "phi_r": aoa[:, 0].astype(np.float32),
        "glob_theta_r": (np.pi / 2.0 - aoa[:, 1]).astype(np.float32),
        "glob_phi_r": aoa[:, 0].astype(np.float32),
        "dedup_keep": keep,
        "path_type": path_type,
        "los": los,
        "doppler_hz": doppler,
    }
    return {
        "theta_t": rx["theta_t"], "phi_t": rx["phi_t"],
        "glob_theta_t": rx["glob_theta_t"], "glob_phi_t": rx["glob_phi_t"],
        "rx_users": [rx], "num_users": 1,
    }


def ris_path_complete(path: Path, *, config_hash: str) -> bool:
    try:
        with np.load(path, allow_pickle=False) as data:
            required = {"coefficient", "delay_s", "doppler_hz", "aod_rad", "aoa_rad",
                        "ris_schema_version", "ris_config_hash"}
            return (
                required.issubset(data.files)
                and int(np.asarray(data["ris_schema_version"]).reshape(())) == RIS_OUTPUT_SCHEMA_VERSION
                and str(np.asarray(data["ris_config_hash"]).reshape(())) == str(config_hash)
            )
    except Exception:
        return False


def save_ris_channels(
    path: Path,
    *,
    config_hash: str,
    h_direct: np.ndarray,
    h_br: np.ndarray | None,
    h_ru: np.ndarray | None,
    phi: np.ndarray | None,
    h_cascade: np.ndarray | None,
    h_effective: np.ndarray | None,
    metadata: Mapping[str, Any],
    save_full_component_channels: bool,
) -> None:
    payload: dict[str, Any] = {
        "ris_schema_version": np.asarray(RIS_OUTPUT_SCHEMA_VERSION, dtype=np.int16),
        "ris_config_hash": np.asarray(str(config_hash)),
        "h_direct": np.asarray(h_direct, dtype=np.complex64),
        "metadata_json": np.asarray(json.dumps(dict(metadata), sort_keys=True)),
    }
    if save_full_component_channels:
        if h_br is not None:
            payload["h_br"] = np.asarray(h_br, dtype=np.complex64)
        if h_ru is not None:
            payload["h_ru"] = np.asarray(h_ru, dtype=np.complex64)
    if phi is not None:
        payload["phi"] = np.asarray(phi, dtype=np.complex64)
    if h_cascade is not None:
        payload["h_cascade"] = np.asarray(h_cascade, dtype=np.complex64)
    if h_effective is not None:
        payload["h_effective"] = np.asarray(h_effective, dtype=np.complex64)
    _atomic_savez(Path(path), payload)


def ris_file_complete(
    path: Path, *, config_hash: str, mode: str,
    save_full_component_channels: bool = False,
) -> bool:
    path = Path(path)
    if not path.is_file() or path.stat().st_size <= 0:
        return False
    required = {"ris_schema_version", "ris_config_hash", "h_direct", "metadata_json"}
    if mode == "effective":
        required |= {"phi", "h_cascade", "h_effective"}
    if save_full_component_channels:
        required |= {"h_br", "h_ru"}
    try:
        with np.load(path, allow_pickle=False) as data:
            if not required.issubset(data.files):
                return False
            return (
                int(np.asarray(data["ris_schema_version"]).reshape(())) == RIS_OUTPUT_SCHEMA_VERSION
                and str(np.asarray(data["ris_config_hash"]).reshape(())) == str(config_hash)
            )
    except Exception:
        return False
