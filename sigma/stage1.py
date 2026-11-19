"""Build site-grounded maps from the migrated deterministic map toolkit.

The public CLI calls ``run_stage1(config, output_dir, seed)``. Set
``mode: prepare`` to reconcile geographic inputs; ``mode: build`` also creates
the Unreal map through official CARLA 0.9.16 OSM2ODR,
Digital Twin reconstruction, geometry cleanup, and semantic verification.
Geographic preparation combines OSM roads and buildings with Overture enrichment.
Run: python -m sigma stage1 -c configs/ris.yaml -o outputs/ris --seed 1477
See docs/stage1.md for the real external build prerequisites and schema.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import signal
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping

ROOT = Path(__file__).resolve().parents[1]
MAP_ROOT = Path(__file__).resolve().parent / "map_construction"
MAP_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]{2,63}$")

CARLA_API_CHECK = """import json
import carla
from importlib import metadata
local = str(carla.Client('127.0.0.1', 2000).get_client_version())
try:
    distribution = metadata.version('carla')
except metadata.PackageNotFoundError:
    distribution = None
clean_release_commit = (distribution == '0.9.16' and local.startswith('294096e') and all(c in '0123456789abcdef' for c in local))
if (local != '0.9.16' and not clean_release_commit) or distribution not in (None, '0.9.16'):
    raise RuntimeError('Official CARLA 0.9.16 Python API required; client={!r}, package={!r}'.format(local, distribution))
print(json.dumps({'client': local, 'distribution': distribution, 'server_checked': False}))
"""


def validate_stage1_config(config: Mapping[str, Any]) -> list[str]:
    """Check configuration errors independently of CARLA and Unreal imports."""
    errors = []
    if not isinstance(config, Mapping):
        return ["stage1 must be a mapping"]
    allowed = {"map_name", "mode", "osm_file", "osm_url", "overture_geojson", "overture_land_cover_geojson", "overture_cli", "overture_release", "height_mode", "vegetation_mode", "ghs_country", "ghs_admin1", "ghs_csv", "promote_grass_pitches", "grass_pitch_way_ids", "missing_building_height_m", "carla_root", "unreal_editor", "carla_python", "xodr_projection_margin_m", "flat_terrain", "timeout_seconds", "max_dtt_tiles", "map_audit_routes", "editor_start_map"}
    for key in config:
        if key not in allowed:
            errors.append(f"unknown stage1 option: {key}")
    for key in ("osm_file", "osm_url", "overture_geojson", "overture_land_cover_geojson", "overture_cli", "overture_release", "ghs_country", "ghs_admin1", "ghs_csv", "carla_root", "unreal_editor", "carla_python", "editor_start_map"):
        if key in config and (not isinstance(config[key], str) or not config[key].strip()):
            errors.append(f"stage1.{key} must be a non-empty string")
    for key in ("flat_terrain", "promote_grass_pitches"):
        if key in config and type(config[key]) is not bool:
            errors.append(f"stage1.{key} must be a YAML boolean")
    if "max_dtt_tiles" in config and (type(config["max_dtt_tiles"]) is not int or config["max_dtt_tiles"] < 1):
        errors.append("stage1.max_dtt_tiles must be a positive integer")
    if "grass_pitch_way_ids" in config and not isinstance(config["grass_pitch_way_ids"], (list, tuple)):
        errors.append("stage1.grass_pitch_way_ids must be a sequence")
    if "map_audit_routes" in config:
        routes = config["map_audit_routes"]
        if not isinstance(routes, Mapping):
            errors.append("stage1.map_audit_routes must be a mapping")
        else:
            for name, route in routes.items():
                if not isinstance(route, Mapping):
                    errors.append(f"stage1.map_audit_routes.{name} must be a mapping")
                    continue
                for key, low, high in (("start_latitude", -90, 90), ("end_latitude", -90, 90), ("start_longitude", -180, 180), ("end_longitude", -180, 180)):
                    try:
                        value = float(route[key])
                        if not math.isfinite(value) or not low <= value <= high:
                            raise ValueError
                    except (TypeError, ValueError, KeyError):
                        errors.append(f"stage1.map_audit_routes.{name}.{key} must be finite WGS84 degrees")
    if not isinstance(config.get("map_name"), str) or not MAP_NAME.fullmatch(config["map_name"]):
        errors.append("stage1.map_name must contain 3–64 alphanumeric/underscore characters and start with a letter")
    if not isinstance(config.get("mode", "build"), str) or config.get("mode", "build") not in {"prepare", "build"}:
        errors.append("stage1.mode must be prepare or build")
    if bool(config.get("osm_file")) == bool(config.get("osm_url")):
        errors.append("stage1 requires exactly one of osm_file and osm_url")
    if not (config.get("overture_geojson") or config.get("overture_cli")):
        errors.append("stage1 requires overture_geojson or overture_cli for OSM–Overture reconciliation")
    if not isinstance(config.get("height_mode", "overture"), str) or config.get("height_mode", "overture") not in {"none", "overture", "overture-ghs"}:
        errors.append("stage1.height_mode must be none, overture, or overture-ghs")
    if not isinstance(config.get("vegetation_mode", "none"), str) or config.get("vegetation_mode", "none") not in {"none", "overture-conservative"}:
        errors.append("stage1.vegetation_mode must be none or overture-conservative")
    if config.get("height_mode") == "overture-ghs" and not (config.get("ghs_country") and config.get("ghs_admin1")):
        errors.append("overture-ghs height enrichment requires ghs_country and ghs_admin1")
    for field in ("xodr_projection_margin_m", "timeout_seconds", "missing_building_height_m"):
        if field in config:
            try:
                value = float(config[field])
                if not math.isfinite(value) or value <= 0:
                    raise ValueError
            except (TypeError, ValueError):
                errors.append(f"stage1.{field} must be finite and positive")
    return errors


def _write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def _run(command: list[str], log_path: Path, *, env: dict[str, str] | None = None, timeout: float = 14400) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            returncode = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise RuntimeError(f"Map command timed out after {timeout:g} seconds; see {log_path}") from None
    if returncode:
        raise RuntimeError(f"Map command failed with exit {returncode}; see {log_path}")


def _environment() -> dict[str, str]:
    env = {key: value for key, value in os.environ.items() if not key.startswith("MOSAIC_")}
    env["PYTHONPATH"] = str(ROOT)
    return env


def _tool(name: str, arguments: list[str], output: Path, *, python: str | None = None, env: dict[str, str] | None = None, timeout: float = 14400) -> None:
    _run([python or sys.executable, "-m", "sigma.map_construction.tools." + name] + arguments, output / (name + ".log"), env=env or _environment(), timeout=timeout)


def _require_report(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f"Required map report was not produced: {path}")
    report = json.loads(path.read_text(encoding="utf-8"))
    if report.get("success") is not True:
        raise RuntimeError(f"Map pass did not report success: {path}")
    return report


def _geometry_prepare(config: Mapping[str, Any], output: Path) -> tuple[Path, dict[str, Any]]:
    from sigma.map_construction.tools.fetch_and_sanitize_osm import fetch, sanitize_osm
    from sigma.map_construction.tools.enrich_osm_building_heights import load_osm
    from sigma.map_construction.tools.normalize_osm_generation_features import normalize

    inputs = output / "input"
    inputs.mkdir(parents=True, exist_ok=True)
    raw = Path(config["osm_file"]).read_bytes() if config.get("osm_file") else fetch(str(config["osm_url"]), 120)
    sanitized, changes = sanitize_osm(raw)
    raw_path, sanitized_path = inputs / "source.osm", inputs / "sanitized.osm"
    raw_path.write_bytes(raw)
    sanitized_path.write_bytes(sanitized)
    _write(output / "sanitize.json", {"success": True, "changes": changes, "source_sha256": hashlib.sha256(raw).hexdigest()})
    _, _, bbox = load_osm(sanitized_path)
    if not (-180 <= bbox[0] < bbox[2] <= 180 and -90 <= bbox[1] < bbox[3] <= 90):
        raise ValueError("OSM bounds must be ordered finite WGS84 longitude/latitude values")
    enriched = inputs / "reconciled.osm"
    arguments = ["--input-osm", str(sanitized_path), "--output-osm", str(enriched), "--report", str(output / "geographic_reconciliation.json"), "--cache-root", str(output / "source_cache"), "--overture-cli", str(config.get("overture_cli", "overturemaps")), "--overture-release", str(config.get("overture_release", "local" if config.get("overture_geojson") else "latest")), "--height-mode", str(config.get("height_mode", "overture")), "--footprint-mode", "overture-conservative", "--vegetation-mode", str(config.get("vegetation_mode", "none")), "--visualization-dir", str(output / "source_topviews")]
    for key, option in [("overture_geojson", "--overture-geojson"), ("overture_land_cover_geojson", "--overture-land-cover-geojson"), ("ghs_country", "--ghs-country"), ("ghs_admin1", "--ghs-admin1"), ("ghs_csv", "--ghs-csv")]:
        if config.get(key):
            arguments += [option, str(config[key])]
    _tool("enrich_osm_building_heights", arguments, output)
    _require_report(output / "geographic_reconciliation.json")
    prepared = inputs / "generation.osm"
    normalize(enriched, prepared, output / "normalization.json", promote_grass_pitches=bool(config.get("promote_grass_pitches", False)), grass_pitch_way_ids=tuple(str(i) for i in config.get("grass_pitch_way_ids", [])), promote_service_roads=False, missing_building_height_m=config.get("missing_building_height_m"))
    tree, buildings, _ = load_osm(prepared)
    root = tree.getroot()
    road_count = sum(bool(next((t.attrib.get("v") for t in way.findall("tag") if t.attrib.get("k") == "highway"), None)) for way in root.findall("way"))
    if not road_count:
        raise ValueError("OSM has no road ways; site reconstruction requires a road network")
    provenance = {"osm_sha256": hashlib.sha256(raw).hexdigest(), "overture_sha256": hashlib.sha256(Path(config["overture_geojson"]).read_bytes()).hexdigest() if config.get("overture_geojson") else None}
    return prepared, {"bbox": list(bbox), "building_count": len(buildings), "road_way_count": road_count, "provenance": provenance, "checks": {"geographic_reconciliation": "passed", "road_connectivity": "not_executed_requires_carla", "route_reachability": "not_executed_requires_loaded_map", "semantic_rendering": "not_executed_requires_unreal"}}


def _build(config: Mapping[str, Any], prepared: Path, output: Path, summary: dict[str, Any], seed: int) -> dict[str, Any]:
    carla_root = Path(str(config.get("carla_root", ""))).expanduser().resolve()
    editor = Path(str(config.get("unreal_editor", ""))).expanduser().resolve()
    project = carla_root / "Unreal/CarlaUE4/CarlaUE4.uproject"
    if not project.is_file() or not editor.is_file():
        raise RuntimeError("Stage I build requires carla_root for an official CARLA 0.9.16 source checkout and unreal_editor for its matching UE 4.26 editor. Geographic intermediates have been preserved.")
    python = str(config.get("carla_python", sys.executable))
    env = _environment()
    _run([python, "-c", CARLA_API_CHECK], output / "carla_api_version.log", env=env)
    min_lon, min_lat, max_lon, max_lat = summary["bbox"]
    stage_config = {"map_name": config["map_name"], "min_longitude": min_lon, "min_latitude": min_lat, "max_longitude": max_lon, "max_latitude": max_lat, "center_latitude": (min_lat + max_lat)/2, "center_longitude": (min_lon + max_lon)/2, "carla_root": str(carla_root), "map_audit_routes": dict(config.get("map_audit_routes", {})), "max_dtt_tiles": config.get("max_dtt_tiles")}
    config_path = output / "map_build_config.json"
    _write(config_path, stage_config)
    prepared_xodr = output / "input" / (str(config["map_name"]) + ".prepared.xodr")
    _tool("prepare_opendrive", ["--config", str(config_path), "--osm", str(prepared), "--output", str(prepared_xodr), "--report", str(output / "opendrive_prepare.json"), "--margin-m", str(config.get("xodr_projection_margin_m", 25.0))], output, python=python)
    stage_config = json.loads(config_path.read_text())
    name = str(config["map_name"])
    env.update({"MOSAIC_MAP_NAME": name, "MOSAIC_OSM_URL": str(config.get("osm_url", "local-file")), "MOSAIC_DTT_LOCAL_OSM_PATH": str(prepared), "MOSAIC_CENTER_LAT": str(stage_config["center_latitude"]), "MOSAIC_CENTER_LON": str(stage_config["center_longitude"]), "MOSAIC_MAX_LAT": str(max_lat), "MOSAIC_MAX_LON": str(max_lon), "MOSAIC_XODR_ORIGIN_LAT": str(stage_config["xodr_origin_latitude"]), "MOSAIC_XODR_ORIGIN_LON": str(stage_config["xodr_origin_longitude"]), "MOSAIC_XODR_END_LAT": str(stage_config["xodr_end_latitude"]), "MOSAIC_XODR_END_LON": str(stage_config["xodr_end_longitude"]), "MOSAIC_DTT_REPORT_JSON": str(output / "generate_save.json"), "MOSAIC_DTT_UE_LOG": str(output / "generate_save_unreal.log"), "MOSAIC_DTT_FLAT_TERRAIN": "1" if config.get("flat_terrain", True) else "0", "MOSAIC_RANDOM_SEED": str(seed)})
    timeout = float(config.get("timeout_seconds", 14400))
    def editor_pass(script: str, level: str, report: Path, *, render: bool = False) -> dict[str, Any]:
        report.parent.mkdir(parents=True, exist_ok=True)
        if render:
            command = [str(editor), str(project), level, "-unattended", "-nop4", "-nosplash", "-stdout", "-FullStdOutLogOutput", "-ExecutePythonScript=" + str(MAP_ROOT / "ue" / script), "-abslog=" + str(report.with_suffix(".unreal.log")), "-RenderOffscreen"]
            _run(command, report.with_suffix(".process.log"), env=env, timeout=timeout)
        else:
            _tool("supervise_ue4_report_pass", ["--editor", str(editor), "--uproject", str(project), "--level-package", level, "--python-script", str(MAP_ROOT / "ue" / script), "--report", str(report), "--ue-log", str(report.with_suffix(".unreal.log")), "--process-log", str(report.with_suffix(".process.log")), "--label", report.stem, "--timeout", str(int(timeout)) + "s"], report.parent / report.stem, env=env, timeout=timeout + 120)
        return _require_report(report)
    target_dir = carla_root / "Unreal/CarlaUE4/Content/CustomMaps" / name
    if target_dir.exists():
        raise FileExistsError(f"Map already exists at {target_dir}; use a new map_name. SiGMA never deletes existing map assets.")
    editor_pass("generate_save_map.py", str(config.get("editor_start_map", "/Game/Carla/Maps/Town10HD_Opt")), output / "generate_save.json", render=True)
    xodrs = sorted(target_dir.rglob("*.xodr"))
    if not xodrs:
        raise RuntimeError("DTT reported completion without a generated OpenDRIVE file")
    xodr = next((p for p in xodrs if p.stem == name), xodrs[0])
    _tool("repair_generated_opendrive", ["--map-name", name, "--xodr", str(xodr), "--report", str(output / "opendrive_repair.json")], output, python=python)
    _tool("audit_carla_map_alignment", ["--config", str(config_path), "--output-dir", str(output / "map_audit"), "--static-only", "--lane-samples-out", str(output / "driving_lane_samples.json")], output, python=python)
    env["MOSAIC_PRUNE_DRIVING_LANE_SAMPLES"] = str(output / "driving_lane_samples.json")
    levels = []
    content = carla_root / "Unreal/CarlaUE4/Content"
    for asset in sorted(target_dir.rglob("*.umap")):
        package = "/Game/" + asset.relative_to(content).with_suffix("").as_posix()
        label = asset.stem
        levels.append({"package": package, "label": label})
        report = output / "geometry_checks" / (label + ".json")
        env.update({"MOSAIC_PRUNE_TARGET_LEVEL": package, "MOSAIC_PRUNE_TARGET_LABEL": label, "MOSAIC_PRUNE_REPORT": str(report)})
        editor_pass("prune_generated_level.py", package, report)
    if not levels:
        raise RuntimeError("DTT did not save any map levels")
    level_manifest = output / "semantic_levels.json"
    _write(level_manifest, {"map_name": name, "levels": levels})
    for mode in ("retag", "verify"):
        report = output / ("semantic_" + mode + ".json")
        env.update({"MOSAIC_SEMANTIC_LEVEL_MANIFEST": str(level_manifest), "MOSAIC_SEMANTIC_PHASE_REPORT": str(report), "MOSAIC_SEMANTIC_LEVEL_SCRIPT": str(MAP_ROOT / "ue/semantic_retag_level.py"), "MOSAIC_SEMANTIC_REPORTS_DIR": str(output / ("semantic_" + mode)), "MOSAIC_RETAG_MODE": mode})
        editor_pass("semantic_retag_map.py", levels[0]["package"], report)
    shutil.copyfile(xodr, output / "map.xodr")
    return {"geometry_built": True, "xodr": str(output / "map.xodr"), "unreal_map_directory": str(target_dir), "levels": levels, "geo_reference": stage_config["geo_reference"], "cook_required": True, "checks": {"geographic_reconciliation": "passed", "road_connectivity": "passed_static_opendrive", "route_reachability": "not_executed_requires_loaded_map", "semantic_asset_verification": "passed", "semantic_rendering": "not_executed_requires_carla_rendering"}, "seed_scope": "traffic/CLI seed retained; external DTT procedural generator does not expose a verified random seed control"}


def run_stage1(config: Mapping[str, Any], output_dir: str | Path, seed: int = 42) -> dict[str, Any]:
    """Run geographic reconciliation and optionally the real Unreal map build.

    Every completed artifact stays in ``output_dir`` if a later step fails.
    ``geometry_built`` is true only after real Unreal passes succeed.
    """
    root_config = config
    config = dict(root_config["stage1"] if "stage1" in root_config else root_config)
    errors = validate_stage1_config(config)
    if errors:
        raise ValueError("; ".join(errors))
    base = Path(str(root_config.get("_config_dir", "."))).expanduser().resolve()
    for key in ("osm_file", "overture_geojson", "overture_land_cover_geojson", "ghs_csv", "carla_root", "unreal_editor", "carla_python"):
        if config.get(key):
            candidate = Path(str(config[key])).expanduser()
            config[key] = str((candidate if candidate.is_absolute() else base / candidate).resolve())
    if config.get("overture_cli") and "/" in str(config["overture_cli"]):
        candidate = Path(str(config["overture_cli"])).expanduser()
        config["overture_cli"] = str((candidate if candidate.is_absolute() else base / candidate).resolve())
    errors = validate_stage1_config(config)
    if errors:
        raise ValueError("; ".join(errors))
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = {"schema_version": "1.0", "stage": 1, "site_id": str(root_config.get("site_id", config["map_name"])), "map_name": config["map_name"], "mode": config.get("mode", "build"), "seed": int(seed), "geometry_built": False, "status": "running"}
    _write(output / "manifest.json", manifest)
    try:
        prepared, summary = _geometry_prepare(config, output)
        manifest.update(summary, prepared_osm=str(prepared), status="geographic_inputs_prepared")
        _write(output / "manifest.json", manifest)
        if manifest["mode"] == "build":
            manifest.update(_build(config, prepared, output, summary, seed))
            manifest["status"] = "map_built_requires_cook"
        _write(output / "manifest.json", manifest)
        return manifest
    except Exception as exc:
        manifest.update(status="failed", error=str(exc), error_type=type(exc).__name__)
        _write(output / "manifest.json", manifest)
        raise
