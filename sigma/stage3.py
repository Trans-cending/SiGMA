"""Stage III: frozen-state Blender XML, CSI/RIS, radio maps, and RCS radar.

Use ``python -m sigma stage3 -c configs/town03.yaml -o outputs/site``.
The old numerical packages bind settings at import; isolated subprocesses now
receive a validated YAML-derived environment. This also keeps the incompatible
communication (2.0.1) and radar (2.2.0) Sionna runtimes separate.
"""
from __future__ import annotations
import os
import math
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RADIO_ENV = {
    "frequency_hz": "FREQUENCY_HZ", "bandwidth_hz": "BANDWIDTH_HZ",
    "num_subcarriers": "N_SUBCARRIERS", "reconstruction": "CHANNEL_RECONSTRUCTION_MODE",
    "array_spacing_wavelength": "ARRAY_SPACING_WL", "antenna_pattern": "ANTENNA_PATTERN",
    "polarization": "POLARIZATION", "max_depth": "RT_MAX_DEPTH",
    "samples_per_src": "RT_SAMPLES_PER_SRC", "max_num_paths_per_src": "RT_MAX_NUM_PATHS_PER_SRC",
    "los": "RT_LOS", "specular_reflection": "RT_SPECULAR_REFLECTION",
    "diffuse_reflection": "RT_DIFFUSE_REFLECTION", "refraction": "RT_REFRACTION",
    "diffraction": "RT_DIFFRACTION", "edge_diffraction": "RT_EDGE_DIFFRACTION",
    "scattering_coefficient": "GLOBAL_SCATTERING_COEFFICIENT",
    "array_front_filter": "ARRAY_IF_FILTER", "rx_antenna_height_m": "RX_ANTENNA_HEIGHT_M",
    "material_extrapolate": "SIONNA_MATERIAL_EXTRAPOLATE", "variant": "MITSUBA_VARIANT",
}


def resolve_path(value, config):
    p = Path(value).expanduser()
    return p.resolve() if p.is_absolute() else (Path(config.get("_config_dir", ".")) / p).resolve()


def executable(value, config):
    if not value:
        return sys.executable
    return str(resolve_path(value, config)) if "/" in str(value) else str(value)


def clean_environment():
    """Prevent ambient settings from overriding explicit YAML or stale runs."""
    env = dict(os.environ)
    prefixes = ("DATASET_", "MOSAIC_", "XML_", "SIONNA_", "RADIO_", "RT_", "VIS_",
                "CHANNEL_", "ARRAY_", "MITSUBA_", "RX_ANTENNA_", "UAV_ANTENNA_",
                "TX_ANTENNA_", "PATH_DEDUP_", "PATH_GEOMETRY_", "BARTLETT_")
    exact = {"FREQUENCY_HZ", "BANDWIDTH_HZ", "N_SUBCARRIERS", "ANTENNA_PATTERN",
             "POLARIZATION", "GLOBAL_SCATTERING_COEFFICIENT", "SKIP_EXISTING_FRAMES",
             "STATIC_XML_PATH", "STAGE3_SUBDIR", "KEEP_WORK_XML", "IF_FILTER",
             "DYNAMIC_SUBDIR", "WORK_SUBDIR", "NUM_CAVS", "MERGE_SHAPES",
             "REMOVE_DUPLICATE_VERTICES", "MODEL_EXTRA_ROT_DEG", "DEVICE_DISPLAY_RADIUS_M"}
    for key in list(env):
        if key.startswith(prefixes) or key in exact:
            env.pop(key)
    env["PYTHONPATH"] = str(PROJECT_ROOT)
    env["PYTHONNOUSERSITE"] = "1"
    return env


def run_command(command, env, log):
    """Stream subprocess output and preserve it if any component fails."""
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("a", encoding="utf-8") as handle:
        with subprocess.Popen(command, cwd=PROJECT_ROOT, env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True) as process:
            for line in process.stdout:
                print(line, end="", flush=True)
                handle.write(line)
                handle.flush()
            code = process.wait()
    if code:
        raise RuntimeError(f"Stage III process exited {code}; reusable outputs retained; log: {log}")


def validate_stage3_config(cfg):
    errors = []
    if not isinstance(cfg, dict):
        return ["stage3 must be a mapping"]
    allowed = {"blender", "radio", "radar", "communication_python", "radar_python", "map_export"}
    errors += [f"unknown stage3 option: {x}" for x in sorted(set(cfg) - allowed)]
    for key in ("blender", "radio", "radar"):
        if not isinstance(cfg.get(key, {}), dict):
            errors.append(f"stage3.{key} must be a mapping")
    if errors:
        return errors
    radio = cfg.get("radio", {})
    for section_name in ("blender", "radio", "radar"):
        if "enabled" in cfg.get(section_name, {}) and type(cfg[section_name]["enabled"]) is not bool:
            errors.append(f"stage3.{section_name}.enabled must be boolean")
    errors += [f"unknown stage3.radio option: {x}" for x in sorted(
        set(radio) - set(RADIO_ENV) - {"enabled", "arrays", "radio_map", "skip_existing"})]
    if radio.get("reconstruction", "farfield") not in {"farfield", "nearfield"}:
        errors.append("stage3.radio.reconstruction must be farfield or nearfield")
    for key in ("frequency_hz", "bandwidth_hz", "num_subcarriers", "array_spacing_wavelength",
                "samples_per_src", "max_num_paths_per_src"):
        if key in radio and (type(radio[key]) not in (int, float) or not math.isfinite(radio[key]) or radio[key] <= 0):
            errors.append(f"stage3.radio.{key} must be positive")
    for key in ("num_subcarriers", "samples_per_src", "max_num_paths_per_src", "max_depth"):
        if key in radio and (type(radio[key]) is not int or radio[key] < (0 if key == "max_depth" else 1)):
            errors.append(f"stage3.radio.{key} must be an integer in range")
    for key in ("los", "specular_reflection", "diffuse_reflection", "refraction", "diffraction",
                "edge_diffraction", "array_front_filter", "material_extrapolate", "skip_existing"):
        if key in radio and type(radio[key]) is not bool:
            errors.append(f"stage3.radio.{key} must be boolean")
    if radio.get("polarization", "V") not in {"V", "H"}:
        errors.append("stage3.radio.polarization must be V or H")
    if "scattering_coefficient" in radio:
        value = radio["scattering_coefficient"]
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 1:
            errors.append("stage3.radio.scattering_coefficient must be finite in [0,1]")
    arrays = radio.get("arrays", [[1, 1, 4, 4]])
    if not isinstance(arrays, list) or not arrays:
        errors.append("stage3.radio.arrays must be a non-empty list")
        arrays = []
    for dims in arrays:
        if not isinstance(dims, list) or len(dims) != 4 or any(type(v) is not int or v < 1 for v in dims):
            errors.append("stage3.radio.arrays requires [rx_cols,rx_rows,tx_cols,tx_rows] positive integers")
    rm = radio.get("radio_map", {})
    if not isinstance(rm, dict):
        errors.append("stage3.radio.radio_map must be a mapping")
    else:
        errors += [f"unknown radio_map option: {x}" for x in sorted(set(rm) - {
            "enabled", "center", "size", "orientation", "cell_size", "samples_per_tx", "max_depth"})]
        if "enabled" in rm and type(rm["enabled"]) is not bool:
            errors.append("radio_map.enabled must be boolean")
        if rm.get("enabled"):
            for key, size in (("center", 3), ("size", 2), ("orientation", 3), ("cell_size", 2)):
                if key not in rm and key in {"orientation", "cell_size"}:
                    continue
                value = rm.get(key)
                if (not isinstance(value, list) or len(value) != size or
                    any(type(x) not in (int, float) or not math.isfinite(x) for x in value) or
                    (key in {"size", "cell_size"} and any(x <= 0 for x in value))):
                    errors.append(f"radio_map.{key} requires {size} finite coordinates/dimensions")
    blender = cfg.get("blender", {})
    export = cfg.get("map_export", {})
    if not isinstance(export, dict):
        errors.append("stage3.map_export must be a mapping")
        export = {}
    elif export:
        from .map_export import validate_export_config
        errors += validate_export_config(export)
        if export.get("enabled") and export.get("format", "fbx") != "fbx":
            errors.append("Stage III Blender conversion requires map_export.format: fbx")
    errors += [f"unknown stage3.blender option: {x}" for x in sorted(set(blender) - {
        "enabled", "executable", "static_scene", "actor_assets", "crop", "crop_margin_m"})]
    if blender.get("enabled", True) and ((not blender.get("static_scene") and not export.get("enabled")) or not blender.get("actor_assets")):
        errors.append("enabled stage3.blender requires static_scene (or enabled map_export) and actor_assets")
    errors += [f"unknown stage3.radar option: {x}" for x in sorted(set(cfg.get("radar", {})) - {"enabled"})]
    return errors


def run_stage3(config, output_dir: Path, seed: int):
    import yaml
    cfg = config["stage3"]
    errors = validate_stage3_config(cfg)
    if errors:
        raise ValueError("; ".join(errors))
    output_dir = Path(output_dir).resolve()
    if not (output_dir / "stage1_multimodal").is_dir():
        raise FileNotFoundError(f"Stage II world states are missing: {output_dir / 'stage1_multimodal'}")
    env = clean_environment()
    env.update(DATASET_ROOT=str(output_dir), RT_SEED=str(seed), SIGMA_SEED=str(seed), VIS_ENABLE="0",
               ARRAY_CONFIGS="1,1,4,4", RADIO_UAV_IDS="none", RADIO_STATIC_RX_IDS="none")
    blender = cfg.get("blender", {})
    if blender.get("enabled", True):
        if cfg.get("map_export", {}).get("enabled"):
            from .map_export import run_export
            exported = run_export(config, output_dir / "map_export")
            scene = Path(exported["static_scene"])
        else:
            scene = resolve_path(blender["static_scene"], config)
        assets = resolve_path(blender["actor_assets"], config)
        if not scene.is_file() or not assets.is_dir():
            raise FileNotFoundError(f"Blender scene or actor asset library missing: {scene}, {assets}")
        env.update(SIONNA_FBX_ROOT=str(assets), XML_TOWN_KEY=output_dir.name,
                   XML_SHARED_MESH_ABS_PATHS="0",
                   XML_STATIC_IF_CROP=str(int(blender.get("crop", True))),
                   XML_STATIC_REQUIRE_CROP="1", XML_STATIC_BBOX_EXTEND=str(blender.get("crop_margin_m", 100)))
        command = [executable(blender.get("executable", "blender"), config), "-b", "--python-exit-code", "2"]
        if scene.suffix.lower() == ".blend":
            command.append(str(scene))
        elif scene.suffix.lower() == ".fbx":
            env["XML_STATIC_FBX_PATH"] = str(scene)
        else:
            raise ValueError("static_scene must be a rebuilt/exported .fbx or .blend")
        command += ["--python", str(PROJECT_ROOT / "Blender_xml_storage/xml_export.py"), "--", str(output_dir)]
        for mode in ("static", "dynamic"):
            env["XML_EXPORT_MODE"] = mode
            run_command(command, env, output_dir / "logs" / f"blender_{mode}.log")
    if not (output_dir / "stage2_xml").is_dir():
        raise FileNotFoundError("Stage III requires frozen-state stage2_xml (run Blender conversion first)")
    radio = cfg.get("radio", {})
    for key, variable in RADIO_ENV.items():
        if key in radio:
            env[variable] = str(int(radio[key])) if isinstance(radio[key], bool) else str(radio[key])
    env["ARRAY_CONFIGS"] = ";".join(",".join(map(str, dims)) for dims in radio.get("arrays", [[1, 1, 4, 4]]))
    env["SKIP_EXISTING_FRAMES"] = str(int(radio.get("skip_existing", True)))
    env["SIGMA_RADIO_MAP_YAML"] = yaml.safe_dump(radio.get("radio_map", {}))
    # Cache identity covers dimensions, materials, physics, seed and geometry
    # so each output matches the complete generation configuration.
    inputs = []
    for directory, pattern in (("stage1_multimodal", "*.yaml"), ("stage2_xml", "*")):
        for path in sorted((output_dir / directory).rglob(pattern)):
            if path.is_file():
                digest = hashlib.sha256()
                with path.open("rb") as handle:
                    for block in iter(lambda: handle.read(1024 * 1024), b""):
                        digest.update(block)
                inputs.append((str(path.relative_to(output_dir)), digest.hexdigest()))
    signature = hashlib.sha256(json.dumps({"config": cfg, "seed": seed, "inputs": inputs},
                                         sort_keys=True).encode()).hexdigest()
    signature_file = output_dir / "stage3_radio/runtime_signature.json"
    if signature_file.is_file():
        previous = json.loads(signature_file.read_text())
        if previous.get("sha256") != signature:
            raise RuntimeError("Stage III physics/seed/input geometry changed; use a new output dataset "
                               "or explicitly archive stage3_radio before recomputing")
    elif signature_file.parent.exists() and any(signature_file.parent.iterdir()):
        raise RuntimeError("Existing stage3_radio lacks a verified runtime signature; archive it before recomputing")
    signature_file.parent.mkdir(parents=True, exist_ok=True)
    signature_file.write_text(json.dumps({"sha256": signature, "seed": seed}, indent=2) + "\n")
    if radio.get("enabled", True):
        run_command([executable(cfg.get("communication_python"), config), "-m",
                     "Sionna_channel_storage.sionna_ofdm_dataset_generator"], env,
                    output_dir / "logs/radio.log")
    if cfg.get("radar", {}).get("enabled", False):
        run_command([executable(cfg.get("radar_python"), config), "-m",
                     "Radar_channel_storage.sionna_rcs_dataset_generator",
                     "--dataset-root", str(output_dir)], env, output_dir / "logs/radar.log")
    return {"dataset_root": str(output_dir), "radio": radio.get("enabled", True),
            "radar": cfg.get("radar", {}).get("enabled", False)}
