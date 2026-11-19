#!/usr/bin/env python3
"""Run the migrated Sionna RT 2.2 RCS smoke in a configured CPU/GPU runtime.

PYTHONPATH=. MITSUBA_VARIANT=llvm_ad_mono_polarized python tests/radar_backend_smoke.py
The numerical scene is a test fixture, independent of live CARLA acquisition.
Source: mosaic_platform_v101/Radar_channel_storage/smoke_sionna220.py.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile

import numpy as np
import trimesh

from CARLA_sensor_storage.radar.profiles import get_radar_profile
from Radar_channel_storage.fmcw import synthesize_fmcw_iq
from Radar_channel_storage.sionna220 import (
    extract_radar_paths, import_sionna_rcs_api, require_sionna_rt_220,
)


def main() -> None:
    import mitsuba as mi
    if os.environ.get("MITSUBA_VARIANT"):
        mi.set_variant(os.environ["MITSUBA_VARIANT"])
    version = require_sionna_rt_220()
    (load_scene, PlanarArray, Transmitter, Receiver, PathSolver,
     RCSSolver, TR38901SensingTarget) = import_sionna_rcs_api()
    profile = get_radar_profile()
    scene = load_scene()
    scene.frequency = profile.start_frequency_hz
    scene.tx_array = PlanarArray(
        num_rows=1, num_cols=1, vertical_spacing=0.5,
        horizontal_spacing=0.5, pattern="iso", polarization="V")
    scene.rx_array = PlanarArray(
        num_rows=1, num_cols=profile.num_rx, vertical_spacing=0.5,
        horizontal_spacing=profile.rx_spacing_wavelength,
        pattern="iso", polarization="V")
    scene.add([
        Transmitter("radar-tx", [0, 0, 1]),
        Receiver("radar-rx", [0, 0, 1]),
    ])
    with tempfile.TemporaryDirectory(prefix="sigma_rcs_smoke_") as temp:
        mesh_path = Path(temp) / "vehicle_box.ply"
        trimesh.creation.box(extents=[4.5, 1.8, 1.5]).export(
            mesh_path, file_type="ply")
        targets = [
            TR38901SensingTarget(
                name="vehicle-101", object_type="vehicle-multi-sp",
                fname=str(mesh_path), position=[20, 0, 1], velocity=[-5, 0, 0],
                random_sigma_s=False, random_phases=False, random_xpr=False),
            TR38901SensingTarget(
                name="vehicle-202", object_type="vehicle-multi-sp",
                fname=str(mesh_path), position=[30, 5, 1], velocity=[0, -2, 0],
                random_sigma_s=False, random_phases=False, random_xpr=False),
        ]
        scene.add(targets)
        paths = RCSSolver(deterministic=True)(
            scene, max_depth=1, buffer_size_per_sp=4096,
            samples_per_sp=4096, synthetic_array=True, los=True,
            specular_reflection=False, refraction=False, seed=17)
        # Exercise the same official optional-background composition API used
        # by the dataset worker. The empty smoke scene can legitimately yield
        # zero background paths; concat and extraction must still be valid.
        background = PathSolver(deterministic=True)(
            scene, max_depth=1, max_num_paths_per_src=4096,
            samples_per_src=4096, synthetic_array=True, los=False,
            specular_reflection=True, diffuse_reflection=False,
            refraction=False, diffraction=False, edge_diffraction=False,
            seed=17,
        )
        combined = paths.concat(background)
        data, details = extract_radar_paths(paths, num_rx=profile.num_rx)
        combined_data, combined_details = extract_radar_paths(
            combined, num_rx=profile.num_rx)
        if combined_data.delay_s.size < data.delay_s.size:
            raise AssertionError("Paths.concat lost RCS paths")
        combined_target_indices = np.asarray(
            combined_details["sensing_target_index"], dtype=np.int32)
        background_path_count = int(np.count_nonzero(combined_target_indices < 0))
        if data.delay_s.size == 0:
            raise AssertionError("official RCSSolver returned no target paths")
        indices = set(np.asarray(details["sensing_target_index"], int).tolist())
        if not indices.issubset({0, 1}) or not indices:
            raise AssertionError(f"unexpected sensing target indices {indices}")
        iq = synthesize_fmcw_iq(data, profile)
        if iq.shape != (4, 256, 128) or iq.dtype != np.complex64:
            raise AssertionError(f"bad IQ contract {iq.shape}/{iq.dtype}")
        min_range = float(np.min(data.delay_s) * 299_792_458.0 / 2.0)
        # Vehicle scattering points span the mesh. The nearest round-trip path
        # must be close to the first target's nominal 20 m range.
        if not 15.0 <= min_range <= 25.0:
            raise AssertionError(f"unexpected minimum target range {min_range}")
        report = {
            "sionna_rt_version": version,
            "num_paths": int(data.delay_s.size),
            "target_indices": sorted(indices),
            "min_range_m": min_range,
            "doppler_hz_min": float(np.min(data.doppler_hz)),
            "doppler_hz_max": float(np.max(data.doppler_hz)),
            "iq_shape": list(iq.shape),
            "iq_dtype": str(iq.dtype),
            "mesh_loaded": True,
            "multi_vehicle_targets": 2,
            "official_paths_concat": True,
            "background_path_count_in_empty_scene": background_path_count,
        }
    report_path = Path(os.environ.get(
        "SIONNA_RCS_SMOKE_REPORT",
        "/tmp/sigma_rcs_smoke_report.json"))
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    print("SIONNA_RCS_220_SMOKE=PASS")


if __name__ == "__main__":
    main()
