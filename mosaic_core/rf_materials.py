"""Canonical RF material registry shared by radar and Sionna RT."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml


class UnknownRFMaterialError(ValueError):
    pass


_REGISTRY_PATH = Path(__file__).resolve().parent / "rf_materials.yaml"
_CACHE: Optional[Dict[str, Any]] = None


def registry_path() -> Path:
    return _REGISTRY_PATH


def load_rf_material_registry() -> Dict[str, Any]:
    global _CACHE
    if _CACHE is None:
        raw = yaml.safe_load(_REGISTRY_PATH.read_text(encoding="utf-8"))
        if not isinstance(raw, dict) or not isinstance(raw.get("materials"), dict):
            raise ValueError(f"Invalid RF material registry: {_REGISTRY_PATH}")
        _CACHE = raw
    return _CACHE


def material_config_hash() -> str:
    registry = load_rf_material_registry()
    # Compatibility declarations are metadata about previous configurations;
    # adding one preserves the current material hash.
    canonical_registry = {
        key: value for key, value in registry.items()
        if key != "compatible_legacy_hashes"
    }
    canonical = json.dumps(canonical_registry, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def material_compatible_hashes() -> set[str]:
    registry = load_rf_material_registry()
    declared = registry.get("compatible_legacy_hashes", []) or []
    return {material_config_hash(), *(str(value) for value in declared)}


def material_entry(rf_material_id: str) -> Dict[str, Any]:
    materials = load_rf_material_registry()["materials"]
    try:
        return materials[str(rf_material_id)]
    except KeyError as exc:
        raise UnknownRFMaterialError(f"Unknown rf_material_id={rf_material_id!r}") from exc


def rf_material_from_carla_semantic(
    semantic_tag: int,
    *,
    object_id: Optional[int] = None,
    frame_id: Optional[int] = None,
    trajectory_id: Optional[int] = None,
) -> str:
    mapping = load_rf_material_registry().get("carla_semantic_tags", {})
    value = mapping.get(int(semantic_tag), mapping.get(str(int(semantic_tag))))
    if value is None:
        raise UnknownRFMaterialError(
            "Unresolved CARLA semantic RF material: "
            f"object_id={object_id}, semantic_tag={semantic_tag}, "
            f"frame={frame_id}, trajectory={trajectory_id}"
        )
    material_entry(value)
    return str(value)


def _strip_material_name(value: str) -> str:
    text = str(value or "")
    while text.rsplit(".", 1)[-1].isdigit() and "." in text:
        text = text.rsplit(".", 1)[0]
    return text.lower().replace("mat-", "").replace("-", "_").replace(" ", "_")


def rf_material_from_sionna(value: str) -> str:
    raw = _strip_material_name(value)
    # Older Blender/Mitsuba exports in the established Scenario3/4 datasets
    # wrote the duplicated prefix in ``mat-itu_itu_concrete``. Collapse
    # only this mechanically repeated prefix; arbitrary unknown names remain a
    # hard error below.
    while raw.startswith("itu_itu_"):
        raw = raw[len("itu_"):]
    # ``mat-Material`` is the single generic material emitted by the legacy
    # Blender exporter.  That exporter used it as its concrete/default wall
    # material. Treat this exact legacy token like ``default_bsdf``; every
    # other unknown material retains strict validation.
    if raw == "material":
        raw = "default_bsdf"
    aliases = load_rf_material_registry().get("sionna_aliases", {})
    candidate = aliases.get(raw)
    if candidate is None and raw.startswith("itu_"):
        # Sionna built-in ground materials omit this registry's frequency-profile
        # suffix (for example itu_medium_dry_ground versus
        # itu_medium_dry_ground_28). Resolve the stripped alias first.
        candidate = aliases.get(raw[len("itu_"):], raw)
    if candidate is None:
        raise UnknownRFMaterialError(f"Unresolved Sionna RF material={value!r}")
    material_entry(candidate)
    return str(candidate)


def rf_material_from_ue(
    material_name: str,
    *,
    object_name: str = "",
    mesh_name: str = "",
    actor_class: Optional[str] = None,
) -> str:
    if actor_class == "human":
        return "itu_human"
    if actor_class in {"vehicle", "truck", "rsu"}:
        return "itu_metal"
    object_text = _strip_material_name(" ".join((object_name or "", mesh_name or "")))
    static_vehicle_markers = (
        "vh_car_",
        "vh_truck_",
        "vh_van_",
        "vehicle_",
        "sm_carlacola",
    )
    if any(marker in object_text for marker in static_vehicle_markers):
        # Official CARLA maps contain parked vehicles baked into the static
        # FBX. Keep their RF identity consistent with runtime vehicle actors.
        return "itu_metal"
    text = _strip_material_name(" ".join((material_name or "", object_name or "", mesh_name or "")))
    if text in load_rf_material_registry()["materials"]:
        return text
    for rule in load_rf_material_registry().get("ue_material_rules", []):
        if any(str(word).lower() in text for word in rule.get("keywords", [])):
            material_id = str(rule["rf_material_id"])
            material_entry(material_id)
            return material_id
    raise UnknownRFMaterialError(
        "Unresolved UE RF material: "
        f"object={object_name!r}, mesh={mesh_name!r}, material={material_name!r}, "
        f"actor_class={actor_class!r}"
    )


def evaluate_dielectric(rf_material_id: str, frequency_hz: float) -> Tuple[float, float]:
    entry = material_entry(rf_material_id)
    f_ghz = float(frequency_hz) / 1e9
    models = list(entry.get("frequency_models_ghz") or [])
    if not models:
        raise ValueError(f"No dielectric model for {rf_material_id}")
    selected = next((row for row in models if float(row[0]) <= f_ghz <= float(row[1])), None)
    if selected is None:
        selected = min(models, key=lambda row: min(abs(f_ghz - float(row[0])), abs(f_ghz - float(row[1]))))
    _lo, _hi, a, b, c, d = (float(x) for x in selected)
    return a * (f_ghz ** b), c * (f_ghz ** d)


def sionna_properties_table() -> Dict[str, list]:
    table: Dict[str, list] = {}
    for entry in load_rf_material_registry()["materials"].values():
        typ = str(entry["sionna_type"])
        rows = [
            ((float(row[0]), float(row[1])), tuple(float(x) for x in row[2:6]))
            for row in entry.get("frequency_models_ghz", [])
        ]
        if typ not in table or len(rows) > len(table[typ]):
            table[typ] = rows
    return table


def material_rgba_table() -> Dict[str, tuple]:
    return {
        key: tuple(float(x) for x in entry["rgba"])
        for key, entry in load_rf_material_registry()["materials"].items()
    }
