"""Simulator-independent RIS configuration and array geometry helpers.

These simulator-independent helpers are shared by Stage 1 metadata generation,
Stage 3 channel generation, cache validation, and unit tests.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Any, Mapping, Sequence

import numpy as np


SPEED_OF_LIGHT_M_S = 299_792_458.0
RIS_SCHEMA_VERSION = 1
SUPPORTED_RIS_CHANNEL_MODES = frozenset({"off", "components", "effective"})
SUPPORTED_RIS_PHASE_MODES = frozenset({"zero", "focus"})


@dataclass(frozen=True)
class RISPanelGeometry:
    num_rows: int
    num_cols: int
    element_spacing_lambda: float
    wavelength_m: float
    element_spacing_m: float
    boundary_width_m: float
    boundary_height_m: float
    center_span_width_m: float
    center_span_height_m: float
    thickness_m: float

    @property
    def num_elements(self) -> int:
        return self.num_rows * self.num_cols

    @property
    def aperture_diagonal_m(self) -> float:
        return math.hypot(self.boundary_width_m, self.boundary_height_m)

    @property
    def fraunhofer_distance_m(self) -> float:
        return 2.0 * self.aperture_diagonal_m**2 / self.wavelength_m

    def as_dict(self) -> dict[str, Any]:
        return {
            "num_rows": self.num_rows,
            "num_cols": self.num_cols,
            "num_elements": self.num_elements,
            "element_spacing_lambda": self.element_spacing_lambda,
            "wavelength_m": self.wavelength_m,
            "element_spacing_m": self.element_spacing_m,
            # Physical panel boundary requested by the YAML contract.
            "boundary_width_m": self.boundary_width_m,
            "boundary_height_m": self.boundary_height_m,
            # Distance between the outermost element centers, measured
            # separately from the physical panel boundary.
            "center_span_width_m": self.center_span_width_m,
            "center_span_height_m": self.center_span_height_m,
            "thickness_m": self.thickness_m,
            "aperture_diagonal_m": self.aperture_diagonal_m,
            "fraunhofer_distance_m": self.fraunhofer_distance_m,
        }


def _positive_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"{path} must be an integer")
    value = int(value)
    if value <= 0:
        raise ValueError(f"{path} must be > 0")
    return value


def _finite_float(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
        raise ValueError(f"{path} must be numeric")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{path} must be finite")
    return value


def normalize_ris_config(raw: Mapping[str, Any] | None) -> dict[str, Any]:
    """Validate and return a fully populated optional RIS configuration."""
    raw = dict(raw or {})
    unknown = sorted(set(raw) - {
        "schema_version", "enabled", "frequency_hz", "nodes", "channel"
    })
    if unknown:
        raise ValueError("Unknown RIS setting(s): " + ", ".join(unknown))
    if "schema_version" in raw and int(raw["schema_version"]) != RIS_SCHEMA_VERSION:
        raise ValueError(
            f"ris.schema_version must be {RIS_SCHEMA_VERSION}; got {raw['schema_version']!r}"
        )
    if "enabled" in raw and type(raw["enabled"]) is not bool:
        raise ValueError("ris.enabled must be boolean")
    enabled = raw.get("enabled", False)
    frequency_hz = _finite_float(raw.get("frequency_hz", 28e9), "ris.frequency_hz")
    if frequency_hz <= 0.0:
        raise ValueError("ris.frequency_hz must be > 0")

    channel_raw = dict(raw.get("channel") or {})
    for flag in ("save_path_parameters", "save_full_component_channels", "require_far_field"):
        if flag in channel_raw and type(channel_raw[flag]) is not bool:
            raise ValueError(f"ris.channel.{flag} must be boolean")
    unknown_channel = sorted(set(channel_raw) - {
        "mode", "save_path_parameters", "save_full_component_channels",
        "reflection_amplitude", "phase_mode", "max_storage_gib",
        "require_far_field",
    })
    if unknown_channel:
        raise ValueError("Unknown ris.channel setting(s): " + ", ".join(unknown_channel))
    mode = str(channel_raw.get("mode", "off" if not enabled else "components")).strip().lower()
    if mode not in SUPPORTED_RIS_CHANNEL_MODES:
        raise ValueError(
            f"ris.channel.mode must be one of {sorted(SUPPORTED_RIS_CHANNEL_MODES)}; got {mode!r}"
        )
    if not enabled and mode != "off":
        raise ValueError("ris.channel.mode must be 'off' when ris.enabled is false")

    reflection_amplitude = _finite_float(
        channel_raw.get("reflection_amplitude", 1.0),
        "ris.channel.reflection_amplitude",
    )
    if not (0.0 <= reflection_amplitude <= 1.0):
        raise ValueError("ris.channel.reflection_amplitude must be in [0, 1]")
    phase_mode = str(channel_raw.get("phase_mode", "zero")).strip().lower()
    if phase_mode not in SUPPORTED_RIS_PHASE_MODES:
        raise ValueError(
            f"ris.channel.phase_mode must be one of {sorted(SUPPORTED_RIS_PHASE_MODES)}; got {phase_mode!r}"
        )

    nodes_raw = raw.get("nodes") or []
    if not isinstance(nodes_raw, list):
        raise ValueError("ris.nodes must be a list")
    if enabled and len(nodes_raw) != 1:
        raise ValueError(
            "RIS v1 supports exactly one enabled node; multi-RIS joint calculation is not implemented"
        )

    nodes: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for index, node_raw in enumerate(nodes_raw):
        if not isinstance(node_raw, Mapping):
            raise ValueError(f"ris.nodes[{index}] must be a mapping")
        node_raw = dict(node_raw)
        path = f"ris.nodes[{index}]"
        unknown_node = sorted(set(node_raw) - {
            "id", "latitude", "longitude", "altitude", "rotation", "panel", "z_mode"
        })
        if unknown_node:
            raise ValueError(f"Unknown {path} setting(s): " + ", ".join(unknown_node))
        node_id = str(node_raw.get("id", f"ris_{index + 1:03d}")).strip()
        if not node_id or node_id in seen_ids:
            raise ValueError(f"{path}.id must be non-empty and unique")
        seen_ids.add(node_id)
        latitude = _finite_float(node_raw.get("latitude"), f"{path}.latitude")
        longitude = _finite_float(node_raw.get("longitude"), f"{path}.longitude")
        altitude = _finite_float(node_raw.get("altitude"), f"{path}.altitude")
        z_mode = str(node_raw.get('z_mode', 'absolute'))
        if z_mode not in ('absolute', 'relative_to_ground'):
            raise ValueError(f'{path}.z_mode must be absolute or relative_to_ground')
        if not (-90.0 <= latitude <= 90.0):
            raise ValueError(f"{path}.latitude must be in [-90, 90]")
        if not (-180.0 <= longitude <= 180.0):
            raise ValueError(f"{path}.longitude must be in [-180, 180]")

        rotation_raw = dict(node_raw.get("rotation") or {})
        unknown_rotation = sorted(set(rotation_raw) - {"pitch", "yaw", "roll"})
        if unknown_rotation:
            raise ValueError(
                f"Unknown {path}.rotation setting(s): " + ", ".join(unknown_rotation)
            )
        rotation = {
            name: _finite_float(rotation_raw.get(name, 0.0), f"{path}.rotation.{name}")
            for name in ("pitch", "yaw", "roll")
        }
        panel_raw = dict(node_raw.get("panel") or {})
        unknown_panel = sorted(set(panel_raw) - {
            "num_rows", "num_cols", "element_spacing_lambda", "thickness_m"
        })
        if unknown_panel:
            raise ValueError(
                f"Unknown {path}.panel setting(s): " + ", ".join(unknown_panel)
            )
        panel = {
            "num_rows": _positive_int(panel_raw.get("num_rows", 32), f"{path}.panel.num_rows"),
            "num_cols": _positive_int(panel_raw.get("num_cols", 32), f"{path}.panel.num_cols"),
            "element_spacing_lambda": _finite_float(
                panel_raw.get("element_spacing_lambda", 0.5),
                f"{path}.panel.element_spacing_lambda",
            ),
            "thickness_m": _finite_float(
                panel_raw.get("thickness_m", 0.02), f"{path}.panel.thickness_m"
            ),
        }
        if panel["element_spacing_lambda"] <= 0.0:
            raise ValueError(f"{path}.panel.element_spacing_lambda must be > 0")
        if panel["thickness_m"] <= 0.0:
            raise ValueError(f"{path}.panel.thickness_m must be > 0")
        nodes.append(
            {
                "id": node_id,
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "rotation": rotation,
                "panel": panel,
                **({'z_mode': z_mode} if 'z_mode' in node_raw else {}),
            }
        )

    max_storage_gib = _finite_float(
        channel_raw.get("max_storage_gib", 8.0), "ris.channel.max_storage_gib"
    )
    if max_storage_gib <= 0.0:
        raise ValueError("ris.channel.max_storage_gib must be > 0")

    return {
        "schema_version": RIS_SCHEMA_VERSION,
        "enabled": enabled,
        "frequency_hz": frequency_hz,
        "nodes": nodes,
        "channel": {
            "mode": mode,
            "save_path_parameters": bool(channel_raw.get("save_path_parameters", True)),
            "save_full_component_channels": bool(
                channel_raw.get("save_full_component_channels", True)
            ),
            "reflection_amplitude": reflection_amplitude,
            "phase_mode": phase_mode,
            "max_storage_gib": max_storage_gib,
            "require_far_field": bool(channel_raw.get("require_far_field", True)),
        },
    }


def panel_geometry(config: Mapping[str, Any], node_index: int = 0) -> RISPanelGeometry:
    cfg = normalize_ris_config(config)
    if node_index >= len(cfg["nodes"]):
        raise IndexError(f"RIS node index {node_index} is unavailable")
    panel = cfg["nodes"][node_index]["panel"]
    wavelength = SPEED_OF_LIGHT_M_S / float(cfg["frequency_hz"])
    spacing = float(panel["element_spacing_lambda"]) * wavelength
    rows = int(panel["num_rows"])
    cols = int(panel["num_cols"])
    return RISPanelGeometry(
        num_rows=rows,
        num_cols=cols,
        element_spacing_lambda=float(panel["element_spacing_lambda"]),
        wavelength_m=wavelength,
        element_spacing_m=spacing,
        boundary_width_m=cols * spacing,
        boundary_height_m=rows * spacing,
        center_span_width_m=max(cols - 1, 0) * spacing,
        center_span_height_m=max(rows - 1, 0) * spacing,
        thickness_m=float(panel["thickness_m"]),
    )


def carla_rotation_matrix(rotation: Mapping[str, Any] | Sequence[float]) -> np.ndarray:
    """Return CARLA local-to-world axes for roll/pitch/yaw in degrees."""
    if isinstance(rotation, Mapping):
        pitch, yaw, roll = (float(rotation.get(k, 0.0)) for k in ("pitch", "yaw", "roll"))
    else:
        pitch, yaw, roll = (float(v) for v in rotation[:3])
    p, y, r = map(math.radians, (pitch, yaw, roll))
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    cr, sr = math.cos(r), math.sin(r)
    # CARLA/UE transform convention (X forward, Y right, Z up).
    forward = np.array([cp * cy, cp * sy, sp], dtype=np.float64)
    right = np.array([
        cy * sp * sr - sy * cr,
        sy * sp * sr + cy * cr,
        -cp * sr,
    ], dtype=np.float64)
    up = np.array([
        -cy * sp * cr - sy * sr,
        -sy * sp * cr + cy * sr,
        cp * cr,
    ], dtype=np.float64)
    return np.column_stack((forward, right, up))


def element_world_coordinates(
    center_xyz: Sequence[float],
    rotation: Mapping[str, Any] | Sequence[float],
    geometry: RISPanelGeometry,
) -> np.ndarray:
    """Return row-major RIS element centers in CARLA world coordinates.

    The panel lies in local Y-Z, with local +X as its outward normal. Columns
    grow along local +Y and rows grow along local +Z.
    """
    col = (np.arange(geometry.num_cols, dtype=np.float64) - (geometry.num_cols - 1) / 2.0)
    row = (np.arange(geometry.num_rows, dtype=np.float64) - (geometry.num_rows - 1) / 2.0)
    yy, zz = np.meshgrid(col * geometry.element_spacing_m, row * geometry.element_spacing_m)
    local = np.column_stack((np.zeros(yy.size), yy.reshape(-1), zz.reshape(-1)))
    center = np.asarray(center_xyz, dtype=np.float64).reshape(1, 3)
    return (local @ carla_rotation_matrix(rotation).T + center).astype(np.float64)


def ris_config_hash(config: Mapping[str, Any]) -> str:
    normalized = normalize_ris_config(config)
    encoded = json.dumps(normalized, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def estimate_component_storage_bytes(
    *, num_subcarriers: int, num_ris_elements: int, num_bs: int, num_ue: int
) -> int:
    # complex64 H_BR + H_RU; excludes compressed path metadata and H_eff.
    return int(num_subcarriers) * int(num_ris_elements) * (int(num_bs) + int(num_ue)) * 8
