"""Execute the installed communication backend on a tiny constructed scene.

Run ``python tests/backend_smoke.py --output /tmp/sigma-backend-smoke`` in the
communication environment (Sionna RT 2.0.1, configured LLVM/CUDA libraries).
This numerical backend integration check uses constructed empty-scene XML and
state fixtures.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sigma.stage3 import run_stage3


def fixture(root, distance=10):
    root.mkdir(parents=True, exist_ok=True)
    trajectory = root / "stage1_multimodal/000001/world_state"
    trajectory.mkdir(parents=True)
    state = {"site_id": "backend_fixture", "trajectory_id": 1, "frame_idx": 0,
             "world_frame": 125, "time_s": 6.25,
             "nodes": {"rsu": {"rsu_001": {"node_id": "rsu_001", "node_type": "rsu", "index": 0,
                        "location": [0, 0, 3], "rotation": {"pitch": 0, "yaw": 0, "roll": 0}}},
                       "cav": {"cav_001": {"node_id": "cav_001", "node_type": "cav", "index": 0,
                        "actor_id": 101, "location": [distance, 0, 3],
                        "rotation": {"pitch": 0, "yaw": 180, "roll": 0}}}},
             "scene": {"vehicles": {"101": {"center": [0, 0, 0], "extent": [0, 0, 0]}}}}
    (trajectory / "world_state_000000.yaml").write_text(yaml.safe_dump(state))
    static = root / "stage2_xml/static"
    dynamic = root / "stage2_xml/000001/dynamic"
    static.mkdir(parents=True)
    dynamic.mkdir(parents=True)
    xml = '<scene version="3.0.0"></scene>'
    (static / (root.name + ".xml")).write_text(xml)
    (dynamic / "dynamic_000000.xml").write_text(xml)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reports = []
    for mode in ("farfield", "nearfield"):
        root = args.output / mode
        fixture(root, distance=0.02 if mode == "nearfield" else 10)
        config = {"stage3": {"blender": {"enabled": False}, "radio": {
            "reconstruction": mode, "arrays": [[2, 1, 4, 2]], "frequency_hz": 28e9,
            "bandwidth_hz": 10e6, "num_subcarriers": 3, "max_depth": 1,
            "samples_per_src": 128, "max_num_paths_per_src": 128,
            "refraction": False, "array_front_filter": False, "rx_antenna_height_m": 0,
            "variant": "llvm_ad_mono_polarized",
            "radio_map": {"enabled": True, "center": [5, 0, 1], "size": [10, 6],
                          "cell_size": [2, 2], "samples_per_tx": 1000, "max_depth": 1}}}}
        run_stage3(config, root, 42)
        products = list((root / "stage3_radio/000001").rglob("*.npz"))
        types = set()
        for path in products:
            with np.load(path, allow_pickle=False) as data:
                assert str(data["site_id"]) == "backend_fixture"
                assert int(data["world_frame"]) == 125
                assert int(data["trajectory_id"]) == 1
                assert float(data["time_s"]) == 6.25
                if "h" in data:
                    types.add("h")
                    assert data["h"].shape == (3, 2, 8)
                    assert np.isfinite(data["h"]).all() and np.any(abs(data["h"]) > 0)
                if "tau_s" in data:
                    types.add("explicit")
                    valid = data["tau_s"][data["tau_s"] >= 0]
                    assert valid.size and np.ptp(valid) > 0
                if "path_gain" in data:
                    types.add("radio_map")
                    assert np.isfinite(data["path_gain"]).all() and np.any(data["path_gain"] > 0)
        assert {"h", "radio_map"}.issubset(types)
        if mode == "nearfield":
            assert "explicit" in types
        run_stage3(config, root, 42)  # Resume identical frozen state.
        try:
            run_stage3(config, root, 43)
        except RuntimeError as exc:
            assert "changed" in str(exc)
        else:
            raise AssertionError("Changed seed reused cached physics")
        reports.append({"mode": mode, "products": len(products), "status": "passed"})
    print(json.dumps(reports))


if __name__ == "__main__":
    main()
