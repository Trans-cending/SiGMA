"""Pure helpers for Stage2 static-scene cropping.

Canonical Stage1 layout::

    <dataset-base>/<map>/stage1_multimodal/000000/world_state/*.yaml

Only compact six-digit physical trajectory directories are supported.
"""
from __future__ import annotations

import json
import os
import re
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

try:
    from .static_streetlight_filter import static_streetlight_filter_audit_error
except ImportError:
    from Blender_xml_storage.static_streetlight_filter import static_streetlight_filter_audit_error

_COMPACT_TRAJ_RE = re.compile(r"^\d{6}$")


def crop_metadata_path(xml_path: str | Path) -> Path:
    return Path(xml_path).with_suffix(".crop.json")


def discover_stage1_trajectory_dirs(
    stage1_root: str | Path,
    town: str,
    *,
    map_name: str = "",
) -> list[Path]:
    """Return canonical compact trajectory directories for one map-scoped root."""
    root = Path(stage1_root).expanduser().resolve()
    scoped_name = str(map_name or town or "").strip()
    if not root.is_dir():
        return []
    if root.name != "stage1_multimodal":
        raise ValueError(f"Stage1 root must end with stage1_multimodal: {root}")
    if scoped_name and root.parent.name != scoped_name:
        raise ValueError(
            f"Stage1 root/map mismatch: root map={root.parent.name!r}, requested={scoped_name!r}"
        )
    return [
        path for path in sorted(root.iterdir())
        if path.is_dir() and _COMPACT_TRAJ_RE.fullmatch(path.name)
    ]

def build_stage1_txrx_crop_spec(
    stage1_root: str | Path,
    town: str,
    margin_m: float,
    *,
    map_name: str = "",
) -> dict[str, Any] | None:
    """Build a Blender-coordinate crop specification from Stage1 world_state."""
    points: list[list[float]] = []
    n_yaml = 0
    trajectories = discover_stage1_trajectory_dirs(stage1_root, town, map_name=map_name)
    for traj_dir in trajectories:
        ws_dir = traj_dir / "world_state"
        if not ws_dir.is_dir():
            continue
        for path in sorted(ws_dir.glob("*.yaml")):
            try:
                data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            except Exception:
                continue
            n_yaml += 1
            nodes = data.get("nodes") or {}
            for kind in ("rsu", "ris", "static_rx", "cav", "uav"):
                group = nodes.get(kind) or {}
                entries = group.values() if isinstance(group, dict) else group
                for node in entries:
                    loc = (node or {}).get("location") if isinstance(node, dict) else None
                    if isinstance(loc, (list, tuple)) and len(loc) >= 3:
                        # CARLA -> Blender: X unchanged, Y mirrored.
                        points.append([float(loc[0]), -float(loc[1]), float(loc[2])])
    if not points:
        return None
    mins = [min(p[i] for p in points) for i in range(3)]
    maxs = [max(p[i] for p in points) for i in range(3)]
    z_buf = 30.0
    box = [
        [mins[0] - float(margin_m), mins[1] - float(margin_m), mins[2] - z_buf],
        [maxs[0] + float(margin_m), maxs[1] + float(margin_m), maxs[2] + z_buf],
    ]
    return {
        "town": str(town),
        "map_name": str(map_name or ""),
        "stage1_root": str(Path(stage1_root).expanduser().resolve()),
        "trajectory_count": len(trajectories),
        "world_state_yaml_count": n_yaml,
        "tx_rx_points": len(points),
        "margin_m": float(margin_m),
        "bbox_min": box[0],
        "bbox_max": box[1],
    }


def write_crop_metadata(xml_path: str | Path, payload: dict[str, Any]) -> Path:
    xml_path = Path(xml_path).resolve()
    out = crop_metadata_path(xml_path)
    data = dict(payload)
    data.setdefault("schema_version", 1)
    data.setdefault("generated_at", datetime.now().astimezone().isoformat())
    data["xml_path"] = str(xml_path)
    try:
        stat = xml_path.stat()
        data["xml_size_bytes"] = int(stat.st_size)
        data["xml_mtime_ns"] = int(stat.st_mtime_ns)
    except OSError:
        pass
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.name + f".tmp.{os.getpid()}")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(tmp, out)
    return out


def read_crop_metadata(xml_path: str | Path) -> dict[str, Any]:
    path = crop_metadata_path(xml_path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def missing_xml_mesh_dependencies(xml_path: str | Path) -> list[str]:
    """Return unique mesh paths referenced by XML but absent on disk."""
    xml_path = Path(xml_path).expanduser().resolve()
    root = ET.parse(xml_path).getroot()
    missing: set[str] = set()
    for node in root.iter("string"):
        if node.get("name") != "filename":
            continue
        value = str(node.get("value") or "").strip()
        if not value:
            continue
        dependency = Path(value).expanduser()
        if not dependency.is_absolute():
            dependency = xml_path.parent / dependency
        if not dependency.is_file():
            missing.add(value)
    return sorted(missing)


def crop_metadata_identity_error(
    xml_path: str | Path,
    meta: dict[str, Any],
) -> str:
    """Return an error when a crop sidecar belongs to another map/artifact."""
    xml_path = Path(xml_path).expanduser().absolute()
    static_root = xml_path.parent
    if static_root.name != "static" or static_root.parent.name != "stage2_xml":
        return f"static XML is outside canonical stage2_xml/static layout: {xml_path}"

    map_root = static_root.parent.parent
    expected_map = map_root.name
    if str(meta.get("map_name") or "").strip() != expected_map:
        return (
            "static crop map mismatch: "
            f"metadata={meta.get('map_name')!r} expected={expected_map!r}"
        )

    expected_stage1 = (map_root / "stage1_multimodal").resolve()
    actual_stage1 = Path(str(meta.get("stage1_root") or "")).expanduser().resolve()
    if actual_stage1 != expected_stage1:
        return (
            "static crop Stage1 root mismatch: "
            f"metadata={actual_stage1} expected={expected_stage1}"
        )

    expected_backup = xml_path.with_name(f"{xml_path.stem}.full.xml")
    actual_backup = Path(str(meta.get("full_backup") or "")).expanduser().resolve()
    if actual_backup != expected_backup or not expected_backup.is_file():
        return (
            "static crop full backup mismatch or missing: "
            f"metadata={actual_backup} expected={expected_backup}"
        )

    actual_xml = Path(str(meta.get("xml_path") or "")).expanduser().resolve()
    if actual_xml != xml_path:
        return (
            "static crop XML path mismatch: "
            f"metadata={actual_xml} expected={xml_path}"
        )
    return ""


def crop_metadata_is_usable(xml_path: str | Path) -> tuple[bool, str, dict[str, Any]]:
    """Validate crop identity and every referenced mesh before Stage3 reuse."""
    xml_path = Path(xml_path)
    if not xml_path.is_file() or xml_path.stat().st_size <= 0:
        return False, "static XML is missing or empty", {}
    if xml_path.name.endswith(".full.xml"):
        return False, "full static XML backup is never safe for Stage3 reuse", {}
    meta = read_crop_metadata(xml_path)
    if not meta:
        return False, "static XML has no crop metadata sidecar", {}
    if meta.get("applied") is not True:
        return False, "static XML crop metadata says crop was not applied", meta
    if int(meta.get("tx_rx_points") or 0) <= 0:
        return False, "static XML crop metadata has no TX/RX points", meta
    if int(meta.get("kept_shapes") or 0) <= 0:
        return False, "static XML crop metadata has no kept shapes", meta
    expected_mode = os.environ.get(
        "SIONNA_EXPECTED_DTT_STREETLIGHT_MODE", ""
    ).strip()
    filter_error = static_streetlight_filter_audit_error(meta, expected_mode)
    if expected_mode and filter_error:
        return False, filter_error, meta
    identity_error = crop_metadata_identity_error(xml_path, meta)
    if identity_error:
        return False, identity_error, meta
    expected_size = int(meta.get("xml_size_bytes") or 0)
    if expected_size > 0 and xml_path.stat().st_size != expected_size:
        return False, "static XML changed after crop metadata was written", meta
    try:
        missing = missing_xml_mesh_dependencies(xml_path)
    except ET.ParseError as exc:
        return False, f"invalid static XML: {exc}", meta
    if missing:
        return False, (
            f"static XML references {len(missing)} missing mesh file(s); "
            f"examples={missing[:3]}"
        ), meta
    return True, "validated TX/RX-cropped static XML", meta
