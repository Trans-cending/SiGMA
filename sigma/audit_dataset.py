"""Audit saved sensor/CSI/RIS/Radar frame identities independently of simulators.

Run ``python -m sigma.audit_dataset outputs/site``. Missing state files, stale
sensor callbacks, missing payloads and inconsistent NPZ identities are errors.
CARLA/Blender static mesh alignment is checked separately on the real map.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import re
import numpy as np
import yaml


def audit(root):
    root = Path(root).resolve()
    errors, states = [], {}
    for trajectory in sorted((root / "stage1_multimodal").glob("[0-9]" * 6)):
        for path in sorted((trajectory / "world_state").glob("world_state_*.yaml")):
            try:
                state = yaml.safe_load(path.read_text())
                frame = int(re.search(r"(\d{6})$", path.stem)[1])
                assert int(state["trajectory_id"]) == int(trajectory.name)
                assert int(state["frame_idx"]) == frame
                assert np.isfinite(state["time_s"])
                if "timestamp" in state:
                    assert np.isclose(state["time_s"], state["timestamp"], rtol=0, atol=1e-6)
                for sensor in state.get("sensors", {}).values():
                    assert int(sensor["sensor_frame"]) == int(state["world_frame"])
                    assert np.isclose(sensor["sensor_timestamp"], state["time_s"], rtol=0, atol=1e-5)
                    assert (trajectory / sensor["file"]).is_file(), sensor["file"]
                states[(int(trajectory.name), frame)] = state
            except (AssertionError, KeyError, ValueError, TypeError, OSError, OverflowError, yaml.YAMLError) as exc:
                errors.append(f"{path.relative_to(root)}: invalid state/sensor identity: {exc}")
    if not states:
        errors.append("No canonical world_state files found")
    products = list((root / "stage3_radio").rglob("*.npz"))
    products += list((root / "stage1_multimodal").rglob("*.npz"))  # Radar IQ in node directories.
    for path in products:
        try:
            with np.load(path, allow_pickle=False) as data:
                state = states[(int(data["trajectory_id"]), int(data["frame_idx"]))]
                assert str(data["site_id"]) == str(state["site_id"])
                assert int(data["world_frame"]) == int(state["world_frame"])
                assert np.isclose(float(data["time_s"]), state["time_s"], rtol=0, atol=1e-6)
        except (AssertionError, KeyError, ValueError, TypeError, OSError, OverflowError) as exc:
            errors.append(f"{path.relative_to(root)}: inconsistent product identity: {exc}")
    return {"ok": not errors, "world_states": len(states), "radio_products": len(products), "errors": errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_root", type=Path)
    args = parser.parse_args()
    result = audit(args.dataset_root)
    print(json.dumps(result, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
