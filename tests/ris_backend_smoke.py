"""Execute real RIS component generation and saved rephasing on Sionna CPU.

Run in the pinned communication environment with LLVM configured::

    python tests/ris_backend_smoke.py --output /tmp/sigma-ris-backend-smoke

This creates constructed empty-scene fixtures. Both far-field reconstruction
and physical explicit-array modes use asymmetric K=3, R=2, T=4, N_RIS=6 axes.
The fixture geometry lies in the small panel's far field in both array modes.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from backend_smoke import fixture
from sigma.stage3 import run_stage3
from Sionna_channel_storage.ris_channel import cascade_channel


def prepare(root: Path) -> dict:
    fixture(root, distance=10)
    ris = {
        "enabled": True, "frequency_hz": 28e9,
        "nodes": [{"id": "ris_001", "latitude": 0, "longitude": 0, "altitude": 3,
                   "rotation": {"pitch": 0, "yaw": -90, "roll": 0},
                   "panel": {"num_rows": 2, "num_cols": 3, "element_spacing_lambda": 0.5}}],
        "channel": {"mode": "effective", "save_full_component_channels": True,
                    "save_path_parameters": True, "phase_mode": "zero",
                    "reflection_amplitude": 0.8, "require_far_field": True},
    }
    sensing = root / "stage1_multimodal"
    (sensing / "sensor_config.yaml").write_text(yaml.safe_dump({"ris": ris}))
    metadata = {"nodes": {"ris_001": {
        "location_carla_m": [5, 2, 3],
        "rotation_carla_deg": {"pitch": 0, "yaw": -90, "roll": 0},
    }}}
    (sensing / "ris_metadata.yaml").write_text(yaml.safe_dump(metadata))
    return ris


def state_identity(data) -> None:
    assert data["site_id"].item() == "backend_fixture"
    assert int(data["trajectory_id"]) == 1
    assert int(data["frame_idx"]) == 0
    assert int(data["world_frame"]) == 125
    assert float(data["time_s"]) == 6.25


def inspect(root: Path, mode: str) -> dict:
    radio = root / "stage3_radio/000001"
    components = list((radio / "ris/channels").rglob("ris_000000.npz"))
    assert len(components) == 1, components
    source = components[0]
    with np.load(source, allow_pickle=False) as data:
        state_identity(data)
        expected = {"h_direct": (3, 2, 4), "h_br": (3, 6, 4),
                    "h_ru": (3, 2, 6), "phi": (3, 6),
                    "h_cascade": (3, 2, 4), "h_effective": (3, 2, 4)}
        for key, shape in expected.items():
            assert data[key].shape == shape, (key, data[key].shape)
            assert np.isfinite(data[key]).all(), key
            assert np.any(np.abs(data[key]) > 0), key
        metadata = json.loads(data["metadata_json"].item())
        assert metadata["array_mode"] == mode
        assert metadata["br_has_valid_path"] and metadata["ru_has_valid_path"]
        assert metadata["panel_projected_aperture_coupling"]["front_side_valid"]
        assert metadata["far_field"]["valid"]
        np.testing.assert_allclose(data["h_cascade"],
                                   cascade_channel(data["h_br"], data["h_ru"], data["phi"]), rtol=1e-5)
        np.testing.assert_allclose(data["h_effective"], data["h_direct"] + data["h_cascade"], rtol=1e-5)
        direct = np.asarray(data["h_direct"])
        coupling = metadata["panel_projected_aperture_coupling"]["field_amplitude_factor"]
    channels = list((radio / "channels").rglob("h_000000.npz"))
    assert len(channels) == 1, channels
    with np.load(channels[0], allow_pickle=False) as data:
        state_identity(data)
        np.testing.assert_array_equal(data["h"], direct)
    path_types = set()
    explicit_types = set()
    for leg in ("rsu_to_ris", "ris_to_cav"):
        for path in (radio / "ris/paths" / leg).rglob("*.npz"):
            with np.load(path, allow_pickle=False) as data:
                state_identity(data)
                if "coefficient" in data:
                    path_types.add(leg)
                    assert data["coefficient"].size > 0
                    assert np.isfinite(data["coefficient"]).all()
                    assert np.isfinite(data["delay_s"]).all()
                if "a" in data:
                    explicit_types.add(leg)
                    assert data["synthetic_array"].item() is False
                    assert data["a"].shape[:2] == ((6, 4) if leg == "rsu_to_ris" else (2, 6))
                    assert data["a"].shape == data["tau_s"].shape
                    assert np.any(data["tau_s"] >= 0)
    assert path_types == {"rsu_to_ris", "ris_to_cav"}
    if mode == "nearfield":
        assert explicit_types == path_types
    rephased = root / "rephased/ris_000000.npz"
    command = [sys.executable, "-m", "Sionna_channel_storage.rephase_ris", "--input", str(source),
               "--output", str(rephased), "--phase-mode", "focus", "--reflection-amplitude", "0.6"]
    subprocess.run(command, cwd=Path(__file__).resolve().parents[1], check=True)
    with np.load(rephased, allow_pickle=False) as data:
        state_identity(data)
        np.testing.assert_allclose(np.abs(data["phi"]), 0.6 * coupling, rtol=1e-6)
        np.testing.assert_allclose(data["h_cascade"],
                                   cascade_channel(data["h_br"], data["h_ru"], data["phi"]), rtol=1e-5)
        np.testing.assert_allclose(data["h_effective"], direct + data["h_cascade"], rtol=1e-5)
        assert data["source_ris_config_hash"].item() == data["ris_config_hash"].item()
        assert json.loads(data["metadata_json"].item())["ray_tracing_reused"] is True
    return {"mode": mode, "status": "passed", "component_shape": [3, 6, 4],
            "user_component_shape": [3, 2, 6], "direct_shape": [3, 2, 4],
            "explicit_component_paths": sorted(explicit_types), "rephase": "passed",
            "shared_state_identity": "passed"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("Choose a new output directory; previous results are preserved")
    reports = []
    for mode in ("farfield", "nearfield"):
        root = args.output / mode
        ris = prepare(root)
        config = {"ris": ris, "stage3": {"blender": {"enabled": False}, "radio": {
            "reconstruction": mode, "arrays": [[2, 1, 2, 2]], "frequency_hz": 28e9,
            "bandwidth_hz": 10e6, "num_subcarriers": 3, "max_depth": 1,
            "samples_per_src": 128, "max_num_paths_per_src": 128,
            "refraction": False, "array_front_filter": False, "rx_antenna_height_m": 0,
            "variant": "llvm_ad_mono_polarized"}}}
        run_stage3(config, root, 42)
        reports.append(inspect(root, mode))
    (args.output / "ris_backend_results.json").write_text(json.dumps(reports, indent=2) + "\n")
    print(json.dumps(reports, indent=2))


if __name__ == "__main__":
    main()
