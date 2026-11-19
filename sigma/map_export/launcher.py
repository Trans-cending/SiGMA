"""Launch the migrated complete-map export using explicit YAML configuration.

Run ``python -m sigma.map_export.cli -c FILE -o OUTPUT`` or call
``run_export(deployment_config, output_dir)``. Source geometry, streamed-level
placement, selection, and FBX/OBJ serialization remain in the original Unreal
exporter. This launcher replaces only host-specific shell paths, cleanup and
scheduler submission, while preserving completed and partial existing exports.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping

from .tools.resolve_map_asset import resolve_map_asset

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = Path(__file__).resolve().parent
ALLOWED_FIELDS = {"enabled", "map_name", "map_source", "format", "selection_mode", "rhi", "timeout_seconds", "minimum_loaded_level_count", "minimum_actor_count", "minimum_static_mesh_component_count", "minimum_output_size_bytes", "meshify_curves", "blender_bin", "carla_root", "unreal_editor_cmd", "fbx"}
FBX_FIELDS = {"ascii", "collision", "level_of_detail", "vertex_color", "force_front_x_axis"}


def validate_export_config(config: Mapping[str, Any]) -> list[str]:
    """Validate the map_export section using local configuration checks."""
    if not isinstance(config, Mapping):
        return ["stage3.map_export must be a mapping"]
    errors = [f"unknown stage3.map_export option: {key}" for key in config if key not in ALLOWED_FIELDS]
    for key in ("enabled", "meshify_curves"):
        if key in config and type(config[key]) is not bool:
            errors.append(f"stage3.map_export.{key} must be a YAML boolean")
    for key, choices in {"map_source": {"auto", "custom", "official"}, "format": {"fbx", "obj"}, "selection_mode": {"geometry", "world"}, "rhi": {"vulkan", "opengl4", "nullrhi"}}.items():
        if key in config and (not isinstance(config[key], str) or config[key] not in choices):
            errors.append(f"stage3.map_export.{key} must be one of {', '.join(sorted(choices))}")
    for key in ("map_name", "blender_bin", "carla_root", "unreal_editor_cmd"):
        if key in config and (not isinstance(config[key], str) or not config[key].strip()):
            errors.append(f"stage3.map_export.{key} must be a non-empty string")
    if "timeout_seconds" in config:
        try:
            value = float(config["timeout_seconds"])
            if not math.isfinite(value) or value <= 0:
                raise ValueError
        except (TypeError, ValueError):
            errors.append("stage3.map_export.timeout_seconds must be finite and positive")
    for key in ("minimum_loaded_level_count", "minimum_actor_count", "minimum_static_mesh_component_count", "minimum_output_size_bytes"):
        if key in config and (type(config[key]) is not int or config[key] < 1):
            errors.append(f"stage3.map_export.{key} must be a positive integer")
    if "fbx" in config:
        settings = config["fbx"]
        if not isinstance(settings, Mapping):
            errors.append("stage3.map_export.fbx must be a mapping")
        else:
            errors += [f"unknown stage3.map_export.fbx option: {key}" for key in settings if key not in FBX_FIELDS]
            errors += [f"stage3.map_export.fbx.{key} must be a YAML boolean" for key, value in settings.items() if type(value) is not bool]
    return errors


def _write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)


def _path(value: str, base: Path, *, executable: bool = False) -> Path:
    if executable and "/" not in str(value):
        found = shutil.which(str(value))
        if found:
            return Path(found).resolve()
    candidate = Path(value).expanduser()
    return (candidate if candidate.is_absolute() else base / candidate).resolve()


def _command(command: list[str], log: Path, env: dict[str, str], timeout: float) -> None:
    with log.open("w", encoding="utf-8") as stream:
        try:
            result = subprocess.run(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=timeout, check=False)
        except subprocess.TimeoutExpired:
            raise RuntimeError(f"Map export command timed out; see {log}") from None
    if result.returncode:
        raise RuntimeError(f"Map export command failed with exit {result.returncode}; see {log}")


def run_export(config: Mapping[str, Any], output_dir: str | Path) -> dict[str, Any]:
    """Export one map, returning ``static_scene`` for Stage III's Blender input.

    All output files stay below output_dir. Unreal reads the official CARLA
    project and serializes the map geometry using its existing editor setup.
    """
    stage3 = config.get("stage3", {})
    if not isinstance(stage3, Mapping):
        raise ValueError("stage3 must be a mapping")
    raw_section = stage3.get("map_export", config.get("map_export", {}))
    errors = validate_export_config(raw_section)
    if errors:
        raise ValueError("; ".join(errors))
    section = dict(raw_section)
    if section.get("enabled", False) is not True:
        raise ValueError("map_export.enabled must be true to run an export")
    stage1 = config.get("stage1", {})
    base = Path(str(config.get("_config_dir", "."))).resolve()
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = {"schema_version": "1.0", "status": "running", "site_id": str(config.get("site_id", ""))}
    _write(output / "manifest.json", manifest)
    try:
        root_value = section.get("carla_root", stage1.get("carla_root"))
        name = section.get("map_name", stage1.get("map_name"))
        if not root_value or not name:
            raise ValueError("Map export needs carla_root and map_name in map_export or stage1")
        carla_root = _path(str(root_value), base)
        project = carla_root / "Unreal/CarlaUE4/CarlaUE4.uproject"
        editor_value = section.get("unreal_editor_cmd")
        if editor_value:
            editor = _path(str(editor_value), base, executable=True)
        elif stage1.get("unreal_editor"):
            editor = _path(str(stage1["unreal_editor"]), base).with_name("UE4Editor-Cmd")
        else:
            raise ValueError("Map export needs unreal_editor_cmd or stage1.unreal_editor")
        if not project.is_file():
            raise FileNotFoundError(f"Matching official CARLA source project not found: {project}")
        if not editor.is_file():
            raise FileNotFoundError(f"Matching UE 4.26 editor command binary not found: {editor}")
        resolved = resolve_map_asset(carla_root, name, section.get("map_source", "auto"))
        export_format = section.get("format", "fbx")
        export_file = output / (str(name) + "." + export_format)
        if export_file.exists():
            raise FileExistsError(f"Existing export retained at {export_file}; choose a new output directory")
        report_path = output / "export_report.json"
        settings = {**resolved, "release_version": "1.0.5", "format": export_format, "selection_mode": section.get("selection_mode", "geometry"), "minimum_loaded_level_count": section.get("minimum_loaded_level_count", 1 + len(resolved["expected_sublevels"])), "minimum_actor_count": section.get("minimum_actor_count", 1), "minimum_static_mesh_component_count": section.get("minimum_static_mesh_component_count", 1), "minimum_output_size_bytes": section.get("minimum_output_size_bytes", 1048576), "fbx": {"ascii": False, "collision": False, "level_of_detail": False, "vertex_color": True, "force_front_x_axis": False, **dict(section.get("fbx", {}))}, "postprocess": {"meshify_curves": section.get("meshify_curves", True)}, "output_file": str(export_file), "report_file": str(report_path), "run_dir": str(output)}
        config_path = output / "export_config.json"
        _write(config_path, settings)
        env = dict(os.environ)
        env["PYTHONPATH"] = str(ROOT)
        env["MOSAIC_EXPORT_CONFIG"] = str(config_path)
        env["CARLA_ROOT"] = str(carla_root)
        rhi = section.get("rhi", "vulkan")
        timeout = float(section.get("timeout_seconds", 21600))
        ue_command = [str(editor), str(project), resolved["map_package"], "-ExecutePythonScript=" + str(PACKAGE / "ue/export_complete_map.py"), "-" + rhi, "-unattended", "-nop4", "-nosplash", "-NoSound", "-stdout", "-FullStdOutLogOutput", "-UTF8Output", "-Abslog=" + str(output / "unreal_export.log"), "-windowed", "-ResX=1920", "-ResY=1080", "-enablehighdpi"]
        if rhi != "nullrhi":
            xvfb = shutil.which("xvfb-run")
            if not xvfb:
                raise RuntimeError("Vulkan/OpenGL export requires xvfb-run as in the original exporter; install Xvfb or explicitly configure a supported nullrhi export")
            ue_command = [xvfb, "-a", "-s", "-screen 0 1920x1080x24 -nolisten tcp", "env", "SDL_VIDEODRIVER=x11"] + ue_command
        command = [sys.executable, "-m", "sigma.map_export.tools.supervise_ue_export", "--report", str(report_path), "--output", str(export_file), "--process-log", str(output / "editor_process.log"), "--hard-timeout-seconds", str(timeout), "--stream-mode", "key", "--"] + ue_command
        _command(command, output / "supervisor.log", env, timeout + 180)
        if export_format == "fbx" and section.get("meshify_curves", True):
            blender_settings = stage3.get("blender", {})
            blender = _path(str(section.get("blender_bin", blender_settings.get("executable", "blender"))), base, executable=True)
            if not blender.is_file():
                raise FileNotFoundError(f"Blender executable not found: {blender}; the completed Unreal export is retained")
            mesh_report = output / "blender_meshify_report.json"
            _command([str(blender), "--background", "--factory-startup", "--python-exit-code", "2", "--python", str(PACKAGE / "tools/blender_meshify_export.py"), "--", "--input", str(export_file), "--output", str(export_file), "--report", str(mesh_report)], output / "blender_meshify.log", env, timeout)
            meshify = json.loads(mesh_report.read_text(encoding="utf-8"))
            if meshify.get("success") is not True:
                raise RuntimeError(f"Blender mesh conversion did not succeed; see {mesh_report}")
            report = json.loads(report_path.read_text(encoding="utf-8"))
            report["meshify_postprocess"] = meshify
            report["export_result"]["output_size_bytes"] = export_file.stat().st_size
            _write(report_path, report)
        _command([sys.executable, "-m", "sigma.map_export.tools.validate_export", str(report_path)], output / "validation.log", env, 60)
        manifest.update(status="completed", static_scene=str(export_file), map_name=str(name), format=export_format, report=str(report_path), map_package=resolved["map_package"], expected_sublevels=resolved["expected_sublevels"], coordinates="Unreal source coordinates; Blender_xml_storage performs the source CARLA-to-radio transform")
        _write(output / "manifest.json", manifest)
        return manifest
    except Exception as exc:
        manifest.update(status="failed", error_type=type(exc).__name__, error=str(exc))
        _write(output / "manifest.json", manifest)
        raise
