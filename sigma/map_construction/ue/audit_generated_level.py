# -*- coding: utf-8 -*-
"""Audit one generated Level after Generate and Save Map complete.

This diagnostic variant is read-only.  In addition to the existing content and
street-light checks it records Vegetation visibility/culling properties so
MOSAIC can distinguish map-asset culling from runtime sensor/rendering issues.
"""

from __future__ import print_function

import json
import os
import sys
import time
import traceback
from pathlib import Path

import unreal

SCRIPT_DIR = str(Path(__file__).resolve().parent)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from level_content_classification import (
    building_night_lighting_classification as classify_building_lighting,
    normalized_context,
    streetlight_kind as classify_streetlight_kind,
)


TARGET_LEVEL = os.environ["MOSAIC_AUDIT_TARGET_LEVEL"]
TARGET_LABEL = os.environ.get(
    "MOSAIC_AUDIT_TARGET_LABEL",
    "unknown",
)
REPORT_PATH = Path(os.environ["MOSAIC_AUDIT_REPORT"])

STATE = {
    "release_version": "1.0.3",
    "runner": "generated-level-content-audit",
    "diagnostic_profile": "vegetation_visibility_v1",
    "target_level": TARGET_LEVEL,
    "target_label": TARGET_LABEL,
    "started_at": time.time(),
    "success": False,
}


VEGETATION_TOKENS = (
    "/carla/static/vegetation/",
    "vegetation",
    "foliage",
    "/trees/",
    "/tree",
    "tree_",
    "sm_tree",
    "/bush",
    "bush_",
    "bush",
    "plant",
    "shrub",
)


def package_path(value):
    return str(value).split(".", 1)[0]


def safe_path(obj):
    if obj is None:
        return ""
    try:
        return str(obj.get_path_name())
    except Exception:
        return ""


def safe_label(actor):
    try:
        return str(actor.get_actor_label())
    except Exception:
        return ""


def safe_class_path(obj):
    try:
        return str(obj.get_class().get_path_name())
    except Exception:
        return ""


def safe_property(obj, name, default=None):
    try:
        return obj.get_editor_property(name)
    except Exception:
        return default


def json_scalar(value):
    if value is None:
        return None
    if isinstance(value, (bool, int, float, str)):
        return value
    try:
        if hasattr(value, "get_path_name"):
            return safe_path(value)
    except Exception:
        pass
    try:
        return float(value)
    except Exception:
        pass
    try:
        return str(value)
    except Exception:
        return None


def positive_number(value):
    try:
        return float(value) > 0.0
    except Exception:
        return False


def actor_has_light_component(actor):
    for class_name in (
        "PointLightComponent",
        "SpotLightComponent",
        "RectLightComponent",
        "LightComponent",
    ):
        cls = getattr(unreal, class_name, None)
        if cls is None:
            continue
        try:
            if actor.get_components_by_class(cls):
                return True
        except Exception:
            continue
    return False


def actor_context_text(actor, mesh_paths):
    return normalized_context(
        safe_path(actor),
        safe_class_path(actor),
        safe_label(actor),
        *mesh_paths
    )


def building_night_lighting_classification(actor, mesh_paths):
    return classify_building_lighting(
        actor_context_text(actor, mesh_paths),
        has_light_component=actor_has_light_component(actor),
    )


def streetlight_kind(actor, mesh_paths):
    return classify_streetlight_kind(
        actor_context_text(actor, mesh_paths),
        safe_label(actor),
        safe_path(actor),
        has_light_component=actor_has_light_component(actor),
    )


def is_vegetation_context(actor, mesh_path, component):
    context = " ".join((
        safe_path(actor),
        safe_class_path(actor),
        safe_label(actor),
        safe_path(component),
        safe_class_path(component),
        mesh_path,
    )).lower()
    return any(token in context for token in VEGETATION_TOKENS)


def component_kind(component):
    class_path = safe_class_path(component).lower()
    if "hierarchicalinstancedstaticmeshcomponent" in class_path:
        return "HISM"
    if "instancedstaticmeshcomponent" in class_path:
        return "ISM"
    if "staticmeshcomponent" in class_path:
        return "StaticMesh"
    return "Other"


def safe_instance_count(component):
    for method_name in ("get_instance_count",):
        method = getattr(component, method_name, None)
        if method is None:
            continue
        try:
            return int(method())
        except Exception:
            pass
    value = safe_property(component, "per_instance_sm_data", None)
    try:
        return len(value) if value is not None else 0
    except Exception:
        return 0


def collect_component_visibility(actor, component, mesh_path):
    properties = {}
    for name in (
        "min_draw_distance",
        "ld_max_draw_distance",
        "cached_max_draw_distance",
        "never_distance_cull",
        "allow_cull_distance_volume",
        "instance_start_cull_distance",
        "instance_end_cull_distance",
        "forced_lod_model",
        "min_lod",
        "bounds_scale",
        "visible",
        "hidden_in_game",
        "cast_shadow",
        "lod_parent_primitive",
    ):
        properties[name] = json_scalar(safe_property(component, name, None))

    actor_hlod_level = json_scalar(safe_property(actor, "hlod_level", None))
    actor_lod_parent = json_scalar(safe_property(actor, "lod_parent_primitive", None))
    actor_enable_auto_lod = json_scalar(
        safe_property(actor, "enable_auto_lod_generation", None)
    )

    nonzero_max_draw = any(
        positive_number(properties.get(name))
        for name in ("ld_max_draw_distance", "cached_max_draw_distance")
    )
    nonzero_instance_end = positive_number(
        properties.get("instance_end_cull_distance")
    )
    allow_cdv = properties.get("allow_cull_distance_volume") is True
    never_distance_cull = properties.get("never_distance_cull") is True
    hlod_related = bool(
        properties.get("lod_parent_primitive")
        or actor_lod_parent
        or positive_number(actor_hlod_level)
        or actor_enable_auto_lod is True
        or "lodactor" in safe_class_path(actor).lower()
        or "hlod" in safe_class_path(actor).lower()
    )

    return {
        "actor_label": safe_label(actor),
        "actor_path": safe_path(actor),
        "actor_class_path": safe_class_path(actor),
        "component_path": safe_path(component),
        "component_class_path": safe_class_path(component),
        "component_kind": component_kind(component),
        "mesh_path": mesh_path,
        "instance_count": safe_instance_count(component),
        "properties": properties,
        "actor_hlod_level": actor_hlod_level,
        "actor_lod_parent_primitive": actor_lod_parent,
        "actor_enable_auto_lod_generation": actor_enable_auto_lod,
        "flags": {
            "nonzero_max_draw_distance": nonzero_max_draw,
            "nonzero_instance_end_cull_distance": nonzero_instance_end,
            "allow_cull_distance_volume": allow_cdv,
            "never_distance_cull": never_distance_cull,
            "hlod_related": hlod_related,
        },
    }


def collect_cull_distance_volume(actor):
    payload = {
        "actor_label": safe_label(actor),
        "actor_path": safe_path(actor),
        "actor_class_path": safe_class_path(actor),
        "enabled": json_scalar(safe_property(actor, "enabled", None)),
        "cull_distance_pairs": [],
    }
    values = safe_property(actor, "cull_distances", None)
    if values is None:
        return payload
    try:
        values = list(values)
    except Exception:
        return payload
    for value in values[:128]:
        size = safe_property(value, "size", None)
        distance = safe_property(value, "cull_distance", None)
        if size is None:
            try:
                size = getattr(value, "size")
            except Exception:
                pass
        if distance is None:
            try:
                distance = getattr(value, "cull_distance")
            except Exception:
                pass
        payload["cull_distance_pairs"].append({
            "size": json_scalar(size),
            "cull_distance": json_scalar(distance),
        })
    return payload


def save_report():
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(
        json.dumps(STATE, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def run():
    world = unreal.EditorLevelLibrary.get_editor_world()
    if world is None:
        raise RuntimeError("Editor world is unavailable")

    loaded_world = package_path(world.get_path_name())
    STATE["loaded_world_package"] = loaded_world

    if loaded_world.lower() != TARGET_LEVEL.lower():
        raise RuntimeError(
            "Loaded Level mismatch: expected {}, got {}".format(
                TARGET_LEVEL,
                loaded_world,
            )
        )

    actors = list(unreal.EditorLevelLibrary.get_all_level_actors())

    component_class = getattr(unreal, "StaticMeshComponent", None)
    if component_class is None:
        raise RuntimeError("unreal.StaticMeshComponent is unavailable")

    actor_with_mesh_count = 0
    component_count = 0
    mesh_asset_paths = []
    streetlight_actor_count = 0
    osm_streetlight_actor_count = 0
    procedural_streetlight_actor_count = 0
    streetlight_actor_samples = []

    vegetation_components = []
    vegetation_actor_paths = set()
    vegetation_component_kind_counts = {
        "StaticMesh": 0,
        "ISM": 0,
        "HISM": 0,
        "Other": 0,
    }
    vegetation_instance_count = 0
    vegetation_nonzero_max_draw_count = 0
    vegetation_nonzero_instance_end_cull_count = 0
    vegetation_allow_cdv_count = 0
    vegetation_never_distance_cull_count = 0
    vegetation_hlod_related_count = 0
    instanced_foliage_actor_count = 0
    cull_distance_volumes = []

    for actor in actors:
        actor_class_path = safe_class_path(actor).lower()
        if "instancedfoliageactor" in actor_class_path:
            instanced_foliage_actor_count += 1
        if "culldistancevolume" in actor_class_path:
            cull_distance_volumes.append(collect_cull_distance_volume(actor))

        try:
            components = actor.get_components_by_class(component_class)
        except Exception:
            components = []

        actor_has_mesh = False
        actor_mesh_paths = []

        for component in components:
            mesh = safe_property(component, "static_mesh", None)
            if mesh is None:
                continue

            actor_has_mesh = True
            component_count += 1

            path = safe_path(mesh)
            if path:
                actor_mesh_paths.append(path)
            if path and path not in mesh_asset_paths and len(mesh_asset_paths) < 200:
                mesh_asset_paths.append(path)

            if not is_vegetation_context(actor, path, component):
                continue

            info = collect_component_visibility(actor, component, path)
            vegetation_components.append(info)
            vegetation_actor_paths.add(safe_path(actor))
            kind = info.get("component_kind", "Other")
            vegetation_component_kind_counts[kind] = (
                vegetation_component_kind_counts.get(kind, 0) + 1
            )
            vegetation_instance_count += int(info.get("instance_count", 0) or 0)
            flags = info.get("flags", {}) or {}
            if flags.get("nonzero_max_draw_distance"):
                vegetation_nonzero_max_draw_count += 1
            if flags.get("nonzero_instance_end_cull_distance"):
                vegetation_nonzero_instance_end_cull_count += 1
            if flags.get("allow_cull_distance_volume"):
                vegetation_allow_cdv_count += 1
            if flags.get("never_distance_cull"):
                vegetation_never_distance_cull_count += 1
            if flags.get("hlod_related"):
                vegetation_hlod_related_count += 1

        if actor_has_mesh:
            actor_with_mesh_count += 1

        lamp_kind = streetlight_kind(actor, actor_mesh_paths)
        if lamp_kind is not None:
            streetlight_actor_count += 1
            if lamp_kind == "osm":
                osm_streetlight_actor_count += 1
            else:
                procedural_streetlight_actor_count += 1
            if len(streetlight_actor_samples) < 100:
                streetlight_actor_samples.append({
                    "kind": lamp_kind,
                    "label": safe_label(actor),
                    "actor_path": safe_path(actor),
                    "actor_class_path": safe_class_path(actor),
                    "mesh_samples": actor_mesh_paths[:5],
                    "has_light_component": actor_has_light_component(actor),
                    "building_lighting_classification": (
                        building_night_lighting_classification(
                            actor, actor_mesh_paths
                        )
                    ),
                })

    is_tile = "_Tile_" in TARGET_LEVEL

    checks = {
        "actors_discovered": len(actors) > 0,
        "static_mesh_components_discovered": component_count > 0,
    }

    if is_tile:
        checks["tile_has_multiple_actors"] = len(actors) >= 10
        checks["tile_has_multiple_mesh_components"] = component_count >= 10

    # Diagnostics record evidence for later analysis. Vegetation counts,
    # including zero, are reported independently of the validity checks.
    diagnostic_samples = vegetation_components[:500]
    STATE.update({
        "actor_count": len(actors),
        "actor_with_static_mesh_count": actor_with_mesh_count,
        "static_mesh_component_count": component_count,
        "mesh_asset_samples": mesh_asset_paths,
        "streetlight_actor_count": streetlight_actor_count,
        "osm_streetlight_actor_count": osm_streetlight_actor_count,
        "procedural_streetlight_actor_count": procedural_streetlight_actor_count,
        "streetlight_actor_samples": streetlight_actor_samples,
        "vegetation_visibility_diagnostic": {
            "vegetation_actor_count": len(vegetation_actor_paths),
            "vegetation_component_count": len(vegetation_components),
            "vegetation_component_kind_counts": vegetation_component_kind_counts,
            "vegetation_instance_count": vegetation_instance_count,
            "instanced_foliage_actor_count": instanced_foliage_actor_count,
            "component_nonzero_max_draw_distance_count": vegetation_nonzero_max_draw_count,
            "component_nonzero_instance_end_cull_distance_count": vegetation_nonzero_instance_end_cull_count,
            "component_allow_cull_distance_volume_count": vegetation_allow_cdv_count,
            "component_never_distance_cull_count": vegetation_never_distance_cull_count,
            "component_hlod_related_count": vegetation_hlod_related_count,
            "cull_distance_volume_count": len(cull_distance_volumes),
            "cull_distance_volumes": cull_distance_volumes[:32],
            "component_samples": diagnostic_samples,
            "sample_truncated": len(vegetation_components) > len(diagnostic_samples),
        },
        "checks": checks,
        "success": all(checks.values()),
        "finish_reason": "generated_level_content_audit_completed",
        "finished_at": time.time(),
    })
    save_report()

    unreal.log(
        "[MOSAIC_GENERATION_AUDIT] "
        "TARGET={} ACTORS={} STATIC_MESH_COMPONENTS={} VEG_COMPONENTS={} "
        "VEG_MAX_DRAW={} VEG_INSTANCE_END_CULL={} CDV={} HLOD_RELATED={} "
        "SUCCESS={}".format(
            TARGET_LEVEL,
            len(actors),
            component_count,
            len(vegetation_components),
            vegetation_nonzero_max_draw_count,
            vegetation_nonzero_instance_end_cull_count,
            len(cull_distance_volumes),
            vegetation_hlod_related_count,
            int(bool(STATE["success"])),
        )
    )

    if not STATE["success"]:
        raise RuntimeError("Generated Level content audit failed")


try:
    run()
except Exception:
    STATE["success"] = False
    STATE["finish_reason"] = "generated_level_content_audit_exception"
    STATE["exception"] = traceback.format_exc()
    STATE["finished_at"] = time.time()
    save_report()
    unreal.log_error(STATE["exception"])
    raise
