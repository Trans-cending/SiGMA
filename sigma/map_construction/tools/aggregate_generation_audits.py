#!/usr/bin/env python3
"""Aggregate post-Generate Level audits, including vegetation visibility data."""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path


def veg(level):
    value = level.get("vegetation_visibility_diagnostic", {})
    return value if isinstance(value, dict) else {}


def main() -> int:
    if len(sys.argv) < 3:
        raise SystemExit("usage: aggregate_generation_audits.py OUTPUT REPORT...")

    output = Path(sys.argv[1])
    levels = [
        json.loads(Path(path).read_text(encoding="utf-8"))
        for path in sys.argv[2:]
    ]

    tile_levels = [
        level for level in levels
        if "_Tile_" in str(level.get("target_level", ""))
    ]

    checks = {
        "all_level_audits_success": all(bool(level.get("success")) for level in levels),
        "tile_level_audited": len(tile_levels) > 0,
        "tile_actor_count_positive": any(
            int(level.get("actor_count", 0) or 0) >= 10 for level in tile_levels
        ),
        "tile_static_mesh_components_positive": any(
            int(level.get("static_mesh_component_count", 0) or 0) >= 10
            for level in tile_levels
        ),
    }

    kind_counts = {"StaticMesh": 0, "ISM": 0, "HISM": 0, "Other": 0}
    cull_volume_samples = []
    component_samples = []
    for level in levels:
        vd = veg(level)
        for key, value in (vd.get("vegetation_component_kind_counts", {}) or {}).items():
            kind_counts[key] = kind_counts.get(key, 0) + int(value or 0)
        for item in vd.get("cull_distance_volumes", []) or []:
            if len(cull_volume_samples) < 64:
                sample = dict(item)
                sample["level"] = level.get("target_level")
                cull_volume_samples.append(sample)
        for item in vd.get("component_samples", []) or []:
            if len(component_samples) < 500:
                sample = dict(item)
                sample["level"] = level.get("target_level")
                component_samples.append(sample)

    vegetation_visibility = {
        "diagnostic_profile": "vegetation_visibility_v1",
        "vegetation_actor_count": sum(int(veg(level).get("vegetation_actor_count", 0) or 0) for level in levels),
        "vegetation_component_count": sum(int(veg(level).get("vegetation_component_count", 0) or 0) for level in levels),
        "vegetation_component_kind_counts": kind_counts,
        "vegetation_instance_count": sum(int(veg(level).get("vegetation_instance_count", 0) or 0) for level in levels),
        "instanced_foliage_actor_count": sum(int(veg(level).get("instanced_foliage_actor_count", 0) or 0) for level in levels),
        "component_nonzero_max_draw_distance_count": sum(int(veg(level).get("component_nonzero_max_draw_distance_count", 0) or 0) for level in levels),
        "component_nonzero_instance_end_cull_distance_count": sum(int(veg(level).get("component_nonzero_instance_end_cull_distance_count", 0) or 0) for level in levels),
        "component_allow_cull_distance_volume_count": sum(int(veg(level).get("component_allow_cull_distance_volume_count", 0) or 0) for level in levels),
        "component_never_distance_cull_count": sum(int(veg(level).get("component_never_distance_cull_count", 0) or 0) for level in levels),
        "component_hlod_related_count": sum(int(veg(level).get("component_hlod_related_count", 0) or 0) for level in levels),
        "cull_distance_volume_count": sum(int(veg(level).get("cull_distance_volume_count", 0) or 0) for level in levels),
        "cull_distance_volume_samples": cull_volume_samples,
        "component_samples": component_samples,
        "sample_truncated": any(bool(veg(level).get("sample_truncated")) for level in levels) or len(component_samples) >= 500,
    }

    result = {
        "release_version": "1.0.3",
        "runner": "generation-audit-aggregate",
        "streetlight_actor_count": sum(int(level.get("streetlight_actor_count", 0) or 0) for level in levels),
        "osm_streetlight_actor_count": sum(int(level.get("osm_streetlight_actor_count", 0) or 0) for level in levels),
        "procedural_streetlight_actor_count": sum(int(level.get("procedural_streetlight_actor_count", 0) or 0) for level in levels),
        "vegetation_visibility_diagnostic": vegetation_visibility,
        "levels": levels,
        "checks": checks,
        "success": all(checks.values()),
        "finished_at": time.time(),
    }

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    for name, value in sorted(checks.items()):
        print("CHECK_{}={}".format(name.upper(), int(bool(value))))

    for level in levels:
        vd = veg(level)
        print(
            "LEVEL_AUDIT={} actors={} static_mesh_components={} veg_components={} "
            "veg_max_draw={} veg_instance_end_cull={} cdv={} hlod_related={} success={}".format(
                level.get("target_level"),
                level.get("actor_count"),
                level.get("static_mesh_component_count"),
                vd.get("vegetation_component_count", 0),
                vd.get("component_nonzero_max_draw_distance_count", 0),
                vd.get("component_nonzero_instance_end_cull_distance_count", 0),
                vd.get("cull_distance_volume_count", 0),
                vd.get("component_hlod_related_count", 0),
                int(bool(level.get("success"))),
            )
        )

    print("VEGETATION_COMPONENTS={}".format(vegetation_visibility["vegetation_component_count"]))
    print("VEGETATION_HISM_COMPONENTS={}".format(kind_counts.get("HISM", 0)))
    print("VEGETATION_ISM_COMPONENTS={}".format(kind_counts.get("ISM", 0)))
    print("VEGETATION_NONZERO_MAX_DRAW_DISTANCE={}".format(vegetation_visibility["component_nonzero_max_draw_distance_count"]))
    print("VEGETATION_NONZERO_INSTANCE_END_CULL={}".format(vegetation_visibility["component_nonzero_instance_end_cull_distance_count"]))
    print("VEGETATION_ALLOW_CULL_DISTANCE_VOLUME={}".format(vegetation_visibility["component_allow_cull_distance_volume_count"]))
    print("VEGETATION_CULL_DISTANCE_VOLUMES={}".format(vegetation_visibility["cull_distance_volume_count"]))
    print("VEGETATION_HLOD_RELATED={}".format(vegetation_visibility["component_hlod_related_count"]))
    print("GENERATION_CONTENT_AUDIT_SUCCESS={}".format(int(bool(result["success"]))))

    return 0 if result["success"] else 41


if __name__ == "__main__":
    raise SystemExit(main())
