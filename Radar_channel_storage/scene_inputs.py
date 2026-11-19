"""Build frame-specific Sionna sensing targets from canonical Stage1/Stage2 data."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from mosaic_core.coordinates import carla_to_sionna_xyz
from mosaic_core.ris import carla_rotation_matrix


@dataclass(frozen=True)
class VehicleTargetSpec:
    actor_id: int
    blueprint_id: str
    mesh_parts: tuple[Path, ...]
    position_sionna_m: np.ndarray
    orientation_sionna_rad: np.ndarray
    velocity_sionna_m_s: np.ndarray
    dimensions_m: np.ndarray


def read_yaml(path: Path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}


def _selected_cav_actor_ids(world_state: dict) -> list[int]:
    cav = ((world_state.get("nodes") or {}).get("cav") or {})
    values = cav.values() if isinstance(cav, dict) else cav
    return [int(v["actor_id"]) for v in values or [] if v.get("actor_id") is not None]


def dynamic_vehicle_meshes(dynamic_xml: Path, world_state: dict) -> dict[int, tuple[Path, ...]]:
    """Map actor IDs to reusable local vehicle mesh parts from Stage2 XML."""
    root = ET.parse(dynamic_xml).getroot()
    selected = _selected_cav_actor_ids(world_state)
    groups: dict[int, set[Path]] = {}
    for shape in root.findall("shape"):
        name = str(shape.get("id") or shape.get("name") or "")
        match = re.search(r"mesh-Vehicle_(\d+)__", name, flags=re.IGNORECASE)
        if match:
            actor_id = int(match.group(1))
        elif re.search(r"mesh-CAV__", name, flags=re.IGNORECASE):
            if len(selected) != 1:
                raise ValueError(
                    f"dynamic XML uses CAV alias but world state has selected actor IDs {selected}"
                )
            actor_id = selected[0]
        else:
            continue
        filename = None
        for child in shape:
            if child.tag.endswith("string") and child.get("name") == "filename":
                filename = child.get("value")
                break
        if not filename:
            continue
        path = Path(filename)
        if not path.is_absolute():
            path = (dynamic_xml.parent / path).resolve()
        if not path.is_file():
            raise FileNotFoundError(f"vehicle mesh part does not exist: {path}")
        groups.setdefault(actor_id, set()).add(path)
    return {key: tuple(sorted(value)) for key, value in groups.items()}


def _rotation_from_vehicle_info(info: dict) -> dict[str, float]:
    # Stage1 scene.angle is [roll,yaw,pitch].
    angle = list(info.get("angle") or [0.0, 0.0, 0.0])
    angle += [0.0] * (3 - len(angle))
    return {"roll": float(angle[0]), "yaw": float(angle[1]), "pitch": float(angle[2])}


def _vehicle_velocity(info: dict, rotation: dict[str, float]) -> np.ndarray:
    velocity = info.get("velocity")
    if isinstance(velocity, (list, tuple)) and len(velocity) >= 3:
        return carla_to_sionna_xyz(velocity, dtype=np.float64)
    # Backward-compatible reconstruction for datasets created before full
    # velocity vectors were added to world_state.
    speed = float(info.get("speed", 0.0))
    forward = carla_rotation_matrix(rotation)[:, 0]
    return carla_to_sionna_xyz(forward * speed, dtype=np.float64)


def build_vehicle_target_specs(world_state: dict, dynamic_xml: Path) -> list[VehicleTargetSpec]:
    vehicles = ((world_state.get("scene") or {}).get("vehicles") or {})
    mesh_map = dynamic_vehicle_meshes(dynamic_xml, world_state)
    specs: list[VehicleTargetSpec] = []
    for raw_actor_id, info in vehicles.items():
        actor_id = int(raw_actor_id)
        if actor_id not in mesh_map:
            continue
        rotation = _rotation_from_vehicle_info(info)
        location = np.asarray(info["location"], dtype=np.float64)
        center_local = np.asarray(info.get("center") or [0.0, 0.0, 0.0], dtype=np.float64)
        center_world = location + carla_rotation_matrix(rotation) @ center_local
        extent = np.asarray(info.get("extent") or [2.0, 1.0, 0.75], dtype=np.float64)
        orientation = np.radians([
            -rotation["yaw"], rotation["pitch"], -rotation["roll"]
        ]).astype(np.float64)
        specs.append(VehicleTargetSpec(
            actor_id=actor_id,
            blueprint_id=str(info.get("bp_id") or "unknown"),
            mesh_parts=mesh_map[actor_id],
            position_sionna_m=carla_to_sionna_xyz(center_world, dtype=np.float64),
            orientation_sionna_rad=orientation,
            velocity_sionna_m_s=_vehicle_velocity(info, rotation),
            dimensions_m=2.0 * extent,
        ))
    return specs


def reusable_target_mesh(spec: VehicleTargetSpec, cache_root: Path) -> Path:
    """Merge material-split CARLA mesh parts once per source-mesh set."""
    try:
        import trimesh
    except ImportError as exc:
        raise RuntimeError(
            "sionna_rcs requires trimesh in the isolated radar environment to merge CARLA mesh parts"
        ) from exc
    digest = hashlib.sha256()
    for path in spec.mesh_parts:
        stat = path.stat()
        digest.update(str(path.resolve()).encode("utf-8"))
        digest.update(str((stat.st_size, stat.st_mtime_ns)).encode("ascii"))
    safe_bp = re.sub(r"[^A-Za-z0-9_.-]+", "_", spec.blueprint_id)
    out = Path(cache_root) / f"{safe_bp}_{digest.hexdigest()[:16]}.ply"
    if out.is_file() and out.stat().st_size > 0:
        return out
    meshes = []
    for path in spec.mesh_parts:
        loaded = trimesh.load_mesh(path, process=False)
        if isinstance(loaded, trimesh.Scene):
            loaded = loaded.dump(concatenate=True)
        meshes.append(loaded)
    if not meshes:
        raise RuntimeError(f"no mesh pieces for vehicle actor {spec.actor_id}")
    combined = trimesh.util.concatenate(meshes)
    Path(cache_root).mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(out.suffix + ".tmp")
    combined.export(tmp, file_type="ply")
    tmp.replace(out)
    return out


def radar_sensor_pose(world_state: dict, calibration: dict, sensor_name: str) -> tuple[np.ndarray, np.ndarray]:
    sensors = dict(calibration.get("sensors") or {})
    if sensor_name not in sensors:
        raise KeyError(f"calibration has no radar sensor {sensor_name!r}")
    entry = sensors[sensor_name]
    world = entry.get("world_transform_at_calibration")
    node_id = str(calibration.get("node_id") or "")
    if world is not None and node_id.startswith("rsu_"):
        location = np.asarray(world["location"], dtype=np.float64)
        rotation = dict(world["rotation"])
    else:
        nodes = world_state.get("nodes") or {}
        kind = "cav" if node_id.startswith("cav_") else "rsu"
        node = (nodes.get(kind) or {}).get(node_id)
        if node is None:
            raise KeyError(f"world_state has no parent node {node_id!r}")
        parent_location = np.asarray(node["location"], dtype=np.float64)
        parent_rotation = dict(node["rotation"])
        transform = ((entry.get("config") or {}).get("transform") or {})
        mount_location = np.asarray(transform.get("location") or [0.0, 0.0, 0.0], dtype=np.float64)
        mount_rotation = list(transform.get("rotation") or [0.0, 0.0, 0.0])
        location = parent_location + carla_rotation_matrix(parent_rotation) @ mount_location
        rotation = {
            "pitch": float(parent_rotation.get("pitch", 0.0)) + float(mount_rotation[0]),
            "yaw": float(parent_rotation.get("yaw", 0.0)) + float(mount_rotation[1]),
            "roll": float(parent_rotation.get("roll", 0.0)) + float(mount_rotation[2]),
        }
    orientation = np.radians([
        -float(rotation.get("yaw", 0.0)),
        float(rotation.get("pitch", 0.0)),
        -float(rotation.get("roll", 0.0)),
    ])
    return carla_to_sionna_xyz(location, dtype=np.float64), orientation.astype(np.float64)

