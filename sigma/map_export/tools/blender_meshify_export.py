#!/usr/bin/env python3
"""Convert Blender curve-like objects in an imported FBX to mesh objects."""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import bpy


CONVERTIBLE_TYPES = ("CURVE", "SURFACE", "FONT", "META")


def blender_args() -> List[str]:
    if "--" not in sys.argv:
        return []
    return sys.argv[sys.argv.index("--") + 1 :]


def args_ns() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    return parser.parse_args(blender_args())


def atomic_json_write(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp.{}".format(os.getpid()))
    tmp.write_text(
        json.dumps(data, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.replace(str(tmp), str(path))


def reset_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def import_fbx(path: Path) -> None:
    bpy.ops.import_scene.fbx(filepath=str(path))


def visible_selectable(obj: bpy.types.Object) -> None:
    obj.hide_set(False)
    obj.hide_select = False
    try:
        obj.hide_viewport = False
    except Exception:
        pass


def convert_curve_like_objects() -> Dict[str, Any]:
    before_types = {}
    for obj in bpy.context.scene.objects:
        before_types[obj.type] = before_types.get(obj.type, 0) + 1

    targets = [
        obj
        for obj in list(bpy.context.scene.objects)
        if obj.type in CONVERTIBLE_TYPES
    ]
    converted = []
    failures = []

    for obj in targets:
        name = obj.name
        source_type = obj.type
        try:
            visible_selectable(obj)
            bpy.ops.object.select_all(action="DESELECT")
            bpy.context.view_layer.objects.active = obj
            obj.select_set(True)
            bpy.ops.object.convert(target="MESH")
            active = bpy.context.view_layer.objects.active
            converted.append(
                {
                    "name": name,
                    "source_type": source_type,
                    "result_name": active.name if active else name,
                    "result_type": active.type if active else "<unknown>",
                }
            )
        except Exception as exc:
            failures.append({"name": name, "type": source_type, "error": repr(exc)})

    after_types = {}
    non_mesh_after = []
    for obj in bpy.context.scene.objects:
        after_types[obj.type] = after_types.get(obj.type, 0) + 1
        if obj.type != "MESH":
            non_mesh_after.append({"name": obj.name, "type": obj.type})

    return {
        "object_type_counts_before": before_types,
        "convertible_types": list(CONVERTIBLE_TYPES),
        "convertible_object_count_before": len(targets),
        "converted_object_count": len(converted),
        "converted_objects": converted[:100],
        "conversion_failure_count": len(failures),
        "conversion_failures": failures[:100],
        "object_type_counts_after": after_types,
        "non_mesh_object_count_after": len(non_mesh_after),
        "non_mesh_objects_after": non_mesh_after[:100],
    }


def export_fbx_meshes_only(path: Path) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    selected_mesh_count = 0
    for obj in bpy.context.scene.objects:
        if obj.type == "MESH":
            obj.select_set(True)
            selected_mesh_count += 1
    if selected_mesh_count <= 0:
        raise RuntimeError("no mesh objects are available for FBX export")
    bpy.ops.export_scene.fbx(
        filepath=str(path),
        use_selection=True,
        object_types={"MESH"},
        use_mesh_modifiers=True,
        add_leaf_bones=False,
    )
    return selected_mesh_count


def main() -> int:
    started = time.time()
    args = args_ns()
    input_path = args.input.resolve()
    output_path = args.output.resolve()
    report_path = args.report.resolve()

    report: Dict[str, Any] = {
        "schema_version": 1,
        "tool": "blender_meshify_export",
        "input_file": str(input_path),
        "output_file": str(output_path),
        "success": False,
        "started_at": started,
    }

    try:
        if input_path.suffix.lower() != ".fbx":
            raise RuntimeError("only FBX meshify postprocess is supported")
        if not input_path.is_file():
            raise RuntimeError("input file is missing: {}".format(input_path))

        reset_scene()
        import_fbx(input_path)
        report.update(convert_curve_like_objects())

        if report["conversion_failure_count"] > 0:
            raise RuntimeError(
                "curve-like object conversion failed for {} object(s)".format(
                    report["conversion_failure_count"]
                )
            )

        remaining_convertible = [
            obj.name
            for obj in bpy.context.scene.objects
            if obj.type in CONVERTIBLE_TYPES
        ]
        if remaining_convertible:
            raise RuntimeError(
                "convertible non-mesh objects remain: {}".format(
                    remaining_convertible[:20]
                )
            )

        if report["convertible_object_count_before"] == 0:
            # The original FBX already contains the required mesh geometry.
            # Reuse it directly to preserve the large-scene encoding.
            report["export_skipped_reason"] = "no_convertible_objects_found"
            report["success"] = True
            report["finished_at"] = time.time()
            report["elapsed_seconds"] = report["finished_at"] - started
            atomic_json_write(report_path, report)
            print(
                "BLENDER_MESHIFY_SUCCESS=1 converted=0 "
                "skipped=no_convertible_objects_found",
                flush=True,
            )
            return 0

        if input_path == output_path:
            export_path = output_path.with_name(
                output_path.stem + ".meshify.tmp" + output_path.suffix
            )
        else:
            export_path = output_path
        try:
            export_path.unlink()
        except FileNotFoundError:
            pass

        report["exported_mesh_object_count"] = export_fbx_meshes_only(export_path)
        report["exported_object_types"] = ["MESH"]
        if not export_path.is_file() or export_path.stat().st_size <= 0:
            raise RuntimeError("mesh-only FBX export did not create output")
        if input_path == output_path:
            os.replace(str(export_path), str(output_path))

        report["output_size_bytes"] = output_path.stat().st_size
        report["success"] = True
        report["finished_at"] = time.time()
        report["elapsed_seconds"] = report["finished_at"] - started
        atomic_json_write(report_path, report)

        print(
            "BLENDER_MESHIFY_SUCCESS=1 converted={} non_mesh_after={}".format(
                report["converted_object_count"],
                report["non_mesh_object_count_after"],
            ),
            flush=True,
        )
        return 0
    except Exception as exc:
        report["success"] = False
        report["error"] = repr(exc)
        report["finished_at"] = time.time()
        report["elapsed_seconds"] = report["finished_at"] - started
        atomic_json_write(report_path, report)
        print("BLENDER_MESHIFY_SUCCESS=0", flush=True)
        print("BLENDER_MESHIFY_ERROR={!r}".format(exc), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
