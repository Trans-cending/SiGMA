#!/usr/bin/env python3
"""Post-process canonical Stage1/Stage2 data with Sionna RT 2.2 RCS.

This entry point intentionally lives outside the communication Stage3 process.
Run via ``python -m sigma stage3 -c configs/town03.yaml`` with a separate
radar_python runtime containing sionna-rt==2.2.0. Outputs include the shared
site/frame/world-frame/trajectory/time identity from the saved scene state.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import yaml


PLATFORM_ROOT = Path(__file__).resolve().parents[1]
if str(PLATFORM_ROOT) not in sys.path:
    sys.path.insert(0, str(PLATFORM_ROOT))

from CARLA_sensor_storage.radar.profiles import get_radar_profile
from CARLA_sensor_storage.radar.writer import RadarIQWriter
from mosaic_core.rf_materials import material_config_hash
from Radar_channel_storage.config import (
    normalize_sionna_rcs_config, radar_profile_overrides,
)
from Radar_channel_storage.fmcw import RadarPathData, synthesize_fmcw_iq
from Radar_channel_storage.scene_inputs import (
    build_vehicle_target_specs, radar_sensor_pose, read_yaml,
    reusable_target_mesh,
)
from Radar_channel_storage.sionna220 import (
    extract_radar_paths, import_sionna_rcs_api, require_sionna_rt_220,
    require_scene_frequency,
)


FRAME_RE = re.compile(r"(\d{6})$")


def _frame_index(path: Path) -> int:
    match = FRAME_RE.search(path.stem)
    if not match:
        raise ValueError(f"cannot parse six-digit frame index from {path.name}")
    return int(match.group(1))


def _atomic_npz(path: Path, **payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + f".tmp.{os.getpid()}")
    with tmp.open("wb") as handle:
        np.savez_compressed(handle, **payload)
    os.replace(tmp, path)


def _static_safe_xml(dataset_root: Path, trajectory_dir: Path, work_dir: Path) -> Path:
    # Import only after DATASET_ROOT is set by main(), because the established
    # Stage3 helpers intentionally bind their dataset root at module import.
    from Sionna_channel_storage.scene_manager import (
        clone_root_with_abs_mesh, resolve_static_xml, sanitize_xml_for_sionna,
    )
    del dataset_root
    source = resolve_static_xml(trajectory_dir)
    intermediate = work_dir / f"{trajectory_dir.name}_radar_static.xml"
    safe = work_dir / f"{trajectory_dir.name}_radar_static_safe.xml"
    root = clone_root_with_abs_mesh(source)
    try:
        ET.indent(root, space="    ")
    except Exception:
        pass
    ET.ElementTree(root).write(intermediate, encoding="utf-8", xml_declaration=True)
    return sanitize_xml_for_sionna(intermediate, safe)


def _radar_entries(trajectory_dir: Path) -> list[tuple[str, str, dict, dict]]:
    result = []
    for calibration_path in sorted((trajectory_dir / "calibration").glob("*.yaml")):
        calibration = read_yaml(calibration_path)
        node_id = str(calibration.get("node_id") or calibration_path.stem)
        for sensor_name, entry in sorted(dict(calibration.get("sensors") or {}).items()):
            cfg = dict(entry.get("config") or {})
            backend = str(cfg.get("radar_backend", cfg.get("backend", "sionna_rcs")))
            backend = backend.strip().lower().replace("-", "_")
            if str(entry.get("type", cfg.get("type", ""))).lower() != "radar":
                continue
            if backend == "sionna_rcs" and bool(cfg.get("enabled", True)):
                result.append((node_id, str(sensor_name), entry, calibration))
    return result


def _rotation_matrix_sionna(euler: np.ndarray) -> np.ndarray:
    alpha, beta, gamma = [float(v) for v in euler]
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta), np.sin(beta)
    cg, sg = np.cos(gamma), np.sin(gamma)
    rz = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]], dtype=np.float64)
    ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]], dtype=np.float64)
    rx = np.array([[1, 0, 0], [0, cg, -sg], [0, sg, cg]], dtype=np.float64)
    return rz @ ry @ rx


def _active_actor_ids(
    specs, radar_position, radar_orientation, max_range_m,
    horizontal_fov_deg, vertical_fov_deg,
):
    rotation = _rotation_matrix_sionna(radar_orientation)
    forward, right, up = rotation[:, 0], rotation[:, 1], rotation[:, 2]
    active = set()
    for spec in specs:
        delta = np.asarray(spec.position_sionna_m) - radar_position
        distance = float(np.linalg.norm(delta))
        ahead = float(np.dot(delta, forward))
        lateral = float(np.dot(delta, right))
        vertical = float(np.dot(delta, up))
        azimuth = np.degrees(np.arctan2(lateral, ahead))
        elevation = np.degrees(np.arctan2(vertical, np.hypot(ahead, lateral)))
        if (ahead > 0.0 and distance <= max_range_m
                and abs(azimuth) <= 0.5 * horizontal_fov_deg
                and abs(elevation) <= 0.5 * vertical_fov_deg):
            active.add(int(spec.actor_id))
    return active


def _filter_paths(data: RadarPathData, details: dict, actor_by_target_index: np.ndarray,
                  active_actor_ids: set[int], keep_background: bool) -> tuple[RadarPathData, dict]:
    target_index = np.asarray(details["sensing_target_index"], dtype=np.int32)
    actor_id = np.full(target_index.shape, -1, dtype=np.int64)
    valid_target = (target_index >= 0) & (target_index < actor_by_target_index.size)
    actor_id[valid_target] = actor_by_target_index[target_index[valid_target]]
    keep = np.asarray([int(value) in active_actor_ids for value in actor_id], dtype=bool)
    if keep_background:
        keep |= target_index < 0
    filtered = RadarPathData(
        coefficient=np.asarray(data.coefficient)[:, keep],
        delay_s=np.asarray(data.delay_s)[keep],
        doppler_hz=np.asarray(data.doppler_hz)[keep],
        target_id=actor_id[keep],
    )
    filtered_details = {}
    for key, value in details.items():
        array = np.asarray(value)
        filtered_details[key] = array[keep] if array.shape[:1] == keep.shape else array
    filtered_details["target_actor_id"] = actor_id[keep]
    return filtered, filtered_details


def _empty_paths(num_rx: int) -> tuple[RadarPathData, dict[str, np.ndarray]]:
    return RadarPathData(
        coefficient=np.zeros((num_rx, 0), dtype=np.complex64),
        delay_s=np.zeros((0,), dtype=np.float32),
        doppler_hz=np.zeros((0,), dtype=np.float32),
        target_id=np.zeros((0,), dtype=np.int64),
    ), {
        "theta_t_rad": np.zeros((0,), dtype=np.float32),
        "phi_t_rad": np.zeros((0,), dtype=np.float32),
        "theta_r_rad": np.zeros((0,), dtype=np.float32),
        "phi_r_rad": np.zeros((0,), dtype=np.float32),
        "interaction_types": np.zeros((0, 0), dtype=np.uint8),
        "object_ids": np.zeros((0, 0), dtype=np.uint32),
        "primitive_ids": np.zeros((0, 0), dtype=np.uint32),
        "interaction_vertices_m": np.zeros((0, 0, 3), dtype=np.float32),
        "sensing_target_index": np.zeros((0,), dtype=np.int32),
        "target_actor_id": np.zeros((0,), dtype=np.int64),
    }


def _path_output(dataset_root: Path, trajectory_dir: Path, node_id: str,
                 sensor_name: str, frame_idx: int) -> Path:
    return (
        dataset_root / "stage3_radio" / trajectory_dir.name / "radar_paths"
        / node_id / sensor_name / f"p_{frame_idx:06d}.npz"
    )


def _iq_output(trajectory_dir: Path, node_id: str, entry: dict, frame_idx: int) -> Path:
    prefix = str(entry.get("file_prefix") or "radar")
    return trajectory_dir / "nodes" / node_id / "radar" / f"{prefix}_{frame_idx:06d}.npz"


def _complete(path: Path, expected_hash: str, expected_shape: tuple[int, int, int]) -> bool:
    if not path.is_file() or path.stat().st_size <= 0:
        return False
    try:
        with np.load(path, allow_pickle=False) as data:
            return (
                "iq" in data
                and tuple(data["iq"].shape) == tuple(expected_shape)
                and data["iq"].dtype == np.complex64
                and str(np.asarray(data["radar_backend"]).reshape(())) == "sionna_rcs"
                and str(np.asarray(data["sionna_rcs_config_hash"]).reshape(())) == expected_hash
            )
    except Exception:
        return False


def _write_paths(path: Path, data: RadarPathData, details: dict, *, profile,
                 cfg, frame_idx: int, trajectory_id: int, node_id: str,
                 sensor_name: str, target_specs) -> None:
    target_table = np.asarray([
        (int(spec.actor_id), str(spec.blueprint_id)) for spec in target_specs
    ], dtype=[("actor_id", "<i8"), ("blueprint_id", "U96")])
    payload = {
        "coefficient": np.asarray(data.coefficient, dtype=np.complex64),
        "delay_s": np.asarray(data.delay_s, dtype=np.float32),
        "doppler_hz": np.asarray(data.doppler_hz, dtype=np.float32),
        "target_actor_id": np.asarray(data.target_id, dtype=np.int64),
        "target_table": target_table,
        "frequency_hz": np.float64(profile.start_frequency_hz),
        "frame_id": np.int64(frame_idx),
        "trajectory_id": np.int64(trajectory_id),
        "node_id": np.asarray(node_id),
        "sensor_name": np.asarray(sensor_name),
        "axis_order": np.asarray("coefficient[rx,path]"),
        "sionna_rt_version": np.asarray("2.2.0"),
        "sionna_rcs_config_hash": np.asarray(cfg.config_hash),
        "sionna_rcs_config_json": np.asarray(json.dumps(cfg.__dict__, sort_keys=True)),
        "material_config_hash": np.asarray(material_config_hash()),
    }
    payload.update({key: np.asarray(value) for key, value in details.items()})
    _atomic_npz(path, **payload)


def process_frame(*, dataset_root: Path, trajectory_dir: Path, world_path: Path,
                  scene, radar_entry, api, mesh_cache: Path, force: bool) -> bool:
    (node_id, sensor_name, entry, calibration) = radar_entry
    cfg_raw = dict(entry.get("config") or {})
    rcs_options = dict(cfg_raw.get("sionna_rcs") or {})
    rcs_options.setdefault("seed", int(os.environ.get("SIGMA_SEED", "17")))
    cfg_raw["sionna_rcs"] = rcs_options
    cfg = normalize_sionna_rcs_config(cfg_raw)
    profile = get_radar_profile(
        cfg_raw.get("radar_profile", "deepsense_s9"),
        radar_profile_overrides(cfg_raw),
    )
    frame_idx = _frame_index(world_path)
    iq_path = _iq_output(trajectory_dir, node_id, entry, frame_idx)
    expected_shape = (
        int(profile.num_rx), int(profile.num_adc_samples), int(profile.num_chirps)
    )
    if not force and _complete(iq_path, cfg.config_hash, expected_shape):
        return False

    dynamic_xml = (
        dataset_root / "stage2_xml" / trajectory_dir.name / "dynamic"
        / f"dynamic_{frame_idx:06d}.xml"
    )
    if not dynamic_xml.is_file():
        raise FileNotFoundError(dynamic_xml)
    world = read_yaml(world_path)
    specs = build_vehicle_target_specs(world, dynamic_xml)
    radar_position, radar_orientation = radar_sensor_pose(
        world, calibration, sensor_name
    )
    radar_velocity = np.zeros(3, dtype=np.float64)
    node_kind = "cav" if node_id.startswith("cav_") else "rsu"
    node = ((world.get("nodes") or {}).get(node_kind) or {}).get(node_id) or {}
    if node_kind == "cav" and node.get("velocity") is not None:
        from mosaic_core.coordinates import carla_to_sionna_xyz
        radar_velocity = carla_to_sionna_xyz(node["velocity"], dtype=np.float64)

    max_range = float(cfg_raw.get("range", 160.0)) + float(cfg.target_margin_m)
    fov = float(profile.horizontal_fov_deg)
    active_actor_ids = _active_actor_ids(
        specs, radar_position, radar_orientation, max_range, fov,
        float(cfg_raw.get("vertical_fov", 10.0)),
    )

    (load_scene, PlanarArray, Transmitter, Receiver, PathSolver,
     RCSSolver, TR38901SensingTarget) = api
    del load_scene
    require_scene_frequency(scene, float(profile.start_frequency_hz))
    scene.tx_array = PlanarArray(
        num_rows=1, num_cols=1, vertical_spacing=0.5,
        horizontal_spacing=0.5, pattern="iso", polarization="V",
    )
    scene.rx_array = PlanarArray(
        num_rows=1, num_cols=int(profile.num_rx), vertical_spacing=0.5,
        horizontal_spacing=float(profile.rx_spacing_wavelength),
        pattern="iso", polarization="V",
    )
    tx = Transmitter("mosaic-radar-tx", radar_position,
                     orientation=radar_orientation, velocity=radar_velocity)
    rx = Receiver("mosaic-radar-rx", radar_position,
                  orientation=radar_orientation, velocity=radar_velocity)
    scene.add([tx, rx])

    targets = []
    target_specs = []
    # Restrict radar targets to vehicles within the configured range.
    # All in-range vehicles are still instantiated, including those just
    # outside FoV, so they remain absorbers/occluders. Their paths are filtered
    # only after solving.
    for spec in specs:
        if np.linalg.norm(spec.position_sionna_m - radar_position) > max_range:
            continue
        mesh = reusable_target_mesh(spec, mesh_cache)
        target = TR38901SensingTarget(
            name=f"vehicle-{spec.actor_id}", object_type=cfg.object_type,
            model_type=int(cfg.model_type), fname=str(mesh),
            position=spec.position_sionna_m,
            orientation=spec.orientation_sionna_rad,
            velocity=spec.velocity_sionna_m_s,
            random_sigma_s=bool(cfg.random_sigma_s),
            random_phases=bool(cfg.random_phases),
            random_xpr=bool(cfg.random_xpr),
        )
        targets.append(target)
        target_specs.append(spec)

    try:
        if targets:
            scene.add(targets)
            rcs_paths = RCSSolver(deterministic=bool(cfg.deterministic))(
                scene, max_depth=int(cfg.max_depth),
                buffer_size_per_sp=int(cfg.buffer_size_per_sp),
                samples_per_sp=int(cfg.samples_per_sp),
                synthetic_array=True, los=bool(cfg.los),
                specular_reflection=bool(cfg.specular_reflection),
                refraction=bool(cfg.refraction), seed=int(cfg.seed) + frame_idx,
            )
            paths = rcs_paths
            if cfg.include_background_paths:
                background = PathSolver(deterministic=bool(cfg.deterministic))(
                    scene, max_depth=int(cfg.background_max_depth),
                    max_num_paths_per_src=int(cfg.buffer_size_per_sp),
                    samples_per_src=int(cfg.background_samples_per_src),
                    synthetic_array=True, los=False,
                    specular_reflection=True, diffuse_reflection=False,
                    refraction=False, diffraction=False,
                    edge_diffraction=False, seed=int(cfg.seed) + frame_idx,
                )
                paths = rcs_paths.concat(background)
            path_data, details = extract_radar_paths(paths, num_rx=profile.num_rx)
            actor_ids = np.asarray([s.actor_id for s in target_specs], dtype=np.int64)
            path_data, details = _filter_paths(
                path_data, details, actor_ids, active_actor_ids,
                keep_background=bool(cfg.include_background_paths),
            )
        else:
            path_data, details = _empty_paths(profile.num_rx)

        iq = synthesize_fmcw_iq(path_data, profile)
        path_path = _path_output(
            dataset_root, trajectory_dir, node_id, sensor_name, frame_idx
        )
        _write_paths(
            path_path, path_data, details, profile=profile, cfg=cfg,
            frame_idx=frame_idx, trajectory_id=int(world.get("trajectory_id", trajectory_dir.name)),
            node_id=node_id, sensor_name=sensor_name, target_specs=target_specs,
        )
        RadarIQWriter(profile).write(
            str(iq_path), iq, material_config_hash=material_config_hash(),
            frame_id=frame_idx, timestamp=float(world.get("time_s", 0.0)),
            trajectory_id=int(world.get("trajectory_id", trajectory_dir.name)),
            radar_backend="sionna_rcs",
            extra_metadata={
                "sionna_rt_version": "2.2.0",
                "sionna_rcs_config_hash": cfg.config_hash,
                "radar_path_file": os.path.relpath(path_path, trajectory_dir),
                "num_radar_paths": int(path_data.delay_s.size),
                "num_sensing_targets": int(len(target_specs)),
                "target_actor_ids": np.asarray(sorted(active_actor_ids), dtype=np.int64),
            },
        )
        for output_file in (path_path, iq_path):
            with np.load(output_file, allow_pickle=False) as saved:
                payload = {key: saved[key] for key in saved.files}
            payload.update(site_id=np.asarray(str(world["site_id"])),
                           frame_idx=np.asarray(frame_idx), world_frame=np.asarray(world["world_frame"]),
                           time_s=np.asarray(world["time_s"], np.float64))
            _atomic_npz(output_file, **payload)
        return True
    finally:
        if targets:
            from .sionna220 import remove_sensing_targets
            remove_sensing_targets(scene, targets)
        scene.remove(["mosaic-radar-tx", "mosaic-radar-rx"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--trajectory", action="append", default=[])
    parser.add_argument("--frame", type=int, action="append", default=[])
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()

    dataset_root = args.dataset_root.resolve()
    stage1_root = dataset_root / "stage1_multimodal"
    if not stage1_root.is_dir() or not (dataset_root / "stage2_xml").is_dir():
        raise RuntimeError(
            f"dataset root must contain stage1_multimodal and stage2_xml: {dataset_root}"
        )
    os.environ["DATASET_ROOT"] = str(dataset_root)
    require_sionna_rt_220()
    api = import_sionna_rcs_api()
    load_scene = api[0]

    selected_trajectories = set(args.trajectory)
    selected_frames = set(args.frame)
    trajectories = [
        path for path in sorted(stage1_root.iterdir())
        if path.is_dir() and re.fullmatch(r"\d{6}", path.name)
        and (not selected_trajectories or path.name in selected_trajectories)
    ]
    converted = skipped = 0
    configured_entries = 0
    mesh_cache = dataset_root / "stage3_radio" / "_radar_mesh_cache"
    with tempfile.TemporaryDirectory(prefix="mosaic_sionna_rcs_") as temp:
        work_dir = Path(temp)
        for trajectory_dir in trajectories:
            entries = _radar_entries(trajectory_dir)
            if not entries:
                continue
            configured_entries += len(entries)
            safe_xml = _static_safe_xml(dataset_root, trajectory_dir, work_dir)
            scene = load_scene(str(safe_xml), merge_shapes=False)
            from Sionna_channel_storage.scene_manager import set_scene_frequency_with_material_report
            # Every configured Sionna radar in one trajectory must share the
            # same carrier, otherwise a separate scene load is required.
            frequencies = {
                float(get_radar_profile(
                    dict(entry.get("config") or {}).get("radar_profile", "deepsense_s9"),
                    radar_profile_overrides(dict(entry.get("config") or {})),
                ).start_frequency_hz)
                for _, _, entry, _ in entries
            }
            if len(frequencies) != 1:
                raise ValueError(
                    f"{trajectory_dir.name}: multiple sionna_rcs carrier frequencies are unsupported: {frequencies}"
                )
            set_scene_frequency_with_material_report(scene, frequencies.pop())
            world_paths = sorted((trajectory_dir / "world_state").glob("world_state_*.yaml"))
            for world_path in world_paths:
                frame_idx = _frame_index(world_path)
                if selected_frames and frame_idx not in selected_frames:
                    continue
                for entry in entries:
                    changed = process_frame(
                        dataset_root=dataset_root, trajectory_dir=trajectory_dir,
                        world_path=world_path, scene=scene, radar_entry=entry,
                        api=api, mesh_cache=mesh_cache, force=bool(args.force),
                    )
                    converted += int(changed)
                    skipped += int(not changed)
                    print(
                        f"[SIONNA_RCS] trajectory={trajectory_dir.name} frame={frame_idx:06d} "
                        f"sensor={entry[0]}/{entry[1]} status={'converted' if changed else 'cached'}",
                        flush=True,
                    )
                    if args.limit and converted >= args.limit:
                        print(json.dumps({"converted": converted, "cached": skipped}))
                        return
    if not configured_entries or not (converted + skipped):
        raise RuntimeError("No Sionna RCS radar calibration/frames found; enable virtual sionna_rcs radar in Stage II YAML")
    print(json.dumps({"converted": converted, "cached": skipped}))


if __name__ == "__main__":
    main()
