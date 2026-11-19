#!/usr/bin/env python3
"""Create a compact, read-only vegetation visibility diagnosis from Stage-1 audit."""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: summarize_vegetation_visibility.py GENERATION_AUDIT_JSON OUTPUT_JSON"
        )
    source = Path(sys.argv[1])
    output = Path(sys.argv[2])
    data = json.loads(source.read_text(encoding="utf-8"))
    v = data.get("vegetation_visibility_diagnostic", {}) or {}

    max_draw = int(v.get("component_nonzero_max_draw_distance_count", 0) or 0)
    instance_end = int(v.get("component_nonzero_instance_end_cull_distance_count", 0) or 0)
    allow_cdv = int(v.get("component_allow_cull_distance_volume_count", 0) or 0)
    cdv = int(v.get("cull_distance_volume_count", 0) or 0)
    hlod = int(v.get("component_hlod_related_count", 0) or 0)
    hism = int((v.get("vegetation_component_kind_counts", {}) or {}).get("HISM", 0) or 0)

    evidence = []
    if instance_end > 0:
        evidence.append("vegetation_instance_end_cull_distance_nonzero")
    if max_draw > 0:
        evidence.append("vegetation_component_max_draw_distance_nonzero")
    if cdv > 0 and allow_cdv > 0:
        evidence.append("cull_distance_volume_can_affect_vegetation")
    if hlod > 0:
        evidence.append("vegetation_hlod_relationship_detected")
    if hism > 0:
        evidence.append("hism_cluster_visibility_path_present")

    if instance_end > 0:
        primary = "asset_or_foliage_instance_distance_culling"
    elif max_draw > 0:
        primary = "component_distance_culling"
    elif cdv > 0 and allow_cdv > 0:
        primary = "cull_distance_volume"
    elif hlod > 0:
        primary = "hlod"
    elif hism > 0:
        primary = "hism_cluster_or_runtime_occlusion"
    else:
        primary = "no_asset_distance_cull_evidence_check_runtime_occlusion_or_streaming"

    result = {
        "success": True,
        "runner": "vegetation-visibility-diagnostic-summary",
        "source_generation_audit": str(source),
        "primary_suspect": primary,
        "evidence": evidence,
        "counts": {
            "vegetation_actor_count": int(v.get("vegetation_actor_count", 0) or 0),
            "vegetation_component_count": int(v.get("vegetation_component_count", 0) or 0),
            "vegetation_instance_count": int(v.get("vegetation_instance_count", 0) or 0),
            "static_mesh_component_count": int((v.get("vegetation_component_kind_counts", {}) or {}).get("StaticMesh", 0) or 0),
            "ism_component_count": int((v.get("vegetation_component_kind_counts", {}) or {}).get("ISM", 0) or 0),
            "hism_component_count": hism,
            "instanced_foliage_actor_count": int(v.get("instanced_foliage_actor_count", 0) or 0),
            "nonzero_max_draw_distance_count": max_draw,
            "nonzero_instance_end_cull_distance_count": instance_end,
            "allow_cull_distance_volume_count": allow_cdv,
            "cull_distance_volume_count": cdv,
            "hlod_related_count": hlod,
        },
        "next_test": (
            "Inspect component_samples in generation_content_audit.json. "
            "If all asset-level cull distances are zero, run the same trajectory "
            "with runtime occlusion disabled before changing map assets."
        ),
        "finished_at": time.time(),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    print("VEGETATION_DIAGNOSTIC_REPORT={}".format(output))
    print("VEGETATION_PRIMARY_SUSPECT={}".format(primary))
    for key, value in result["counts"].items():
        print("VEGETATION_DIAG_{}={}".format(key.upper(), value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
