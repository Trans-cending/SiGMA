"""Cross-modal dataset identity auditing against canonical CARLA elapsed time."""
import json
import sys

import numpy as np
import pytest
import yaml

from sigma.audit_dataset import audit, main


def dataset_fixture(root):
    trajectory = root / "stage1_multimodal/000001"
    state_file = trajectory / "world_state/world_state_000025.yaml"
    state_file.parent.mkdir(parents=True)
    image = trajectory / "nodes/cav_001/rgb/front_000025.png"
    image.parent.mkdir(parents=True)
    image.write_bytes(b"fixture-presence-only")
    state = {"site_id": "site3", "trajectory_id": 1, "frame_idx": 25,
             "world_frame": 125, "time_s": 6.25, "timestamp": 6.25,
             "sensors": {"rgb": {"sensor_frame": 125, "sensor_timestamp": 6.25,
                                  "file": str(image.relative_to(trajectory))}}}
    state_file.write_text(yaml.safe_dump(state))
    identity = {"site_id": "site3", "trajectory_id": 1, "frame_idx": 25,
                "world_frame": 125, "time_s": 6.25}
    channel = root / "stage3_radio/000001/channels/h001/rc/rsu_to_cav/h_000025.npz"
    channel.parent.mkdir(parents=True)
    np.savez(channel, **identity, h=np.ones((3, 2, 4), np.complex64))
    ris = root / "stage3_radio/000001/ris/channels/h001/rc/ris_000025.npz"
    ris.parent.mkdir(parents=True)
    np.savez(ris, **identity, h_cascade=np.ones((3, 2, 4), np.complex64))
    radar = trajectory / "nodes/rsu_001/radar/iq_000025.npz"
    radar.parent.mkdir(parents=True)
    np.savez(radar, **identity, iq=np.ones((2, 4, 8), np.complex64))
    return state_file, state, channel, identity, image


def test_correct_sensor_csi_ris_radar_shared_identity(tmp_path):
    dataset_fixture(tmp_path)
    result = audit(tmp_path)
    assert result == {"ok": True, "world_states": 1, "radio_products": 3, "errors": []}


@pytest.mark.parametrize("field,value", [("frame_idx", 24), ("world_frame", 124),
                                         ("time_s", 0.0), ("site_id", "another_site")])
def test_wrong_product_frame_or_time_fails(tmp_path, field, value):
    _, _, channel, identity, _ = dataset_fixture(tmp_path)
    np.savez(channel, **{**identity, field: value}, h=np.ones((3, 2, 4), np.complex64))
    result = audit(tmp_path)
    assert result["ok"] is False
    assert any("inconsistent product identity" in error and channel.name in error for error in result["errors"])


@pytest.mark.parametrize("field,value", [("sensor_frame", 124), ("sensor_timestamp", 0.0)])
def test_stale_sensor_callback_fails(tmp_path, field, value):
    state_file, state, _, _, _ = dataset_fixture(tmp_path)
    state["sensors"]["rgb"][field] = value
    state_file.write_text(yaml.safe_dump(state))
    result = audit(tmp_path)
    assert result["ok"] is False
    assert any("invalid state/sensor identity" in error for error in result["errors"])


def test_relative_recording_time_cannot_replace_canonical_elapsed_time(tmp_path):
    state_file, state, _, _, _ = dataset_fixture(tmp_path)
    state["time_s"] = 0.0
    state_file.write_text(yaml.safe_dump(state))
    assert audit(tmp_path)["ok"] is False


def test_missing_payload_and_no_state_are_errors(tmp_path):
    _, _, _, _, image = dataset_fixture(tmp_path)
    image.unlink()
    assert audit(tmp_path)["ok"] is False
    empty = audit(tmp_path / "missing")
    assert empty["ok"] is False
    assert "No canonical world_state files found" in empty["errors"]


def test_audit_cli_failure_exit_and_json_report(tmp_path, monkeypatch, capsys):
    _, _, channel, identity, _ = dataset_fixture(tmp_path)
    np.savez(channel, **{**identity, "time_s": 0.0})
    monkeypatch.setattr(sys, "argv", ["sigma.audit_dataset", str(tmp_path)])
    assert main() == 1
    assert json.loads(capsys.readouterr().out)["ok"] is False


def test_nonscalar_product_identity_is_reported_as_error(tmp_path):
    _, _, channel, identity, _ = dataset_fixture(tmp_path)
    np.savez(channel, **{**identity, "trajectory_id": np.array([1, 2])})
    result = audit(tmp_path)
    assert result["ok"] is False
    assert any("inconsistent product identity" in error for error in result["errors"])
