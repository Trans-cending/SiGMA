"""Saved 1024-element RIS rephasing must preserve physical/state metadata."""
import json
import sys

import numpy as np
import pytest

from Sionna_channel_storage import rephase_ris


def saved_components(path, *, valid=True):
    np.savez(path, h_br=np.ones((1, 1024, 16), np.complex64),
             h_ru=np.ones((1, 1, 1024), np.complex64),
             h_direct=np.full((1, 1, 16), 3, np.complex64),
             ris_config_hash=np.asarray("source_hash"),
             site_id=np.asarray("site3"), trajectory_id=np.asarray("000001"),
             frame_idx=np.asarray(25), world_frame=np.asarray(87), time_s=np.asarray(1.25),
             metadata_json=np.asarray(json.dumps({"br_has_valid_path": valid,
                 "ru_has_valid_path": True,
                 "panel_projected_aperture_coupling": {"field_amplitude_factor": 0.5}})))


def run_rephase(monkeypatch, source, output, *, amplitude="0.8"):
    monkeypatch.setattr(sys, "argv", ["rephase_ris", "--input", str(source), "--output", str(output),
                                     "--phase-mode", "focus", "--reflection-amplitude", amplitude])
    rephase_ris.main()


def test_1024_rephase_applies_coupling_and_can_rephase_again(tmp_path, monkeypatch):
    source, first, second = [tmp_path / f"{name}.npz" for name in ("source", "first", "second")]
    saved_components(source)
    run_rephase(monkeypatch, source, first)
    with np.load(first) as data:
        assert data["phi"].shape == (1, 1024)
        np.testing.assert_allclose(data["h_cascade"], np.full((1, 1, 16), 1024 * 0.8 * 0.5), rtol=1e-6)
        np.testing.assert_allclose(data["h_effective"], data["h_cascade"] + 3)
        assert data["site_id"].item() == "site3"
        assert data["trajectory_id"].item() == "000001"
        assert data["frame_idx"].item() == 25
        assert data["world_frame"].item() == 87
        assert data["time_s"].item() == 1.25
        assert data["ris_config_hash"].item() == "source_hash"
        assert len(data["phase_configuration_hash"].item()) == 64
    run_rephase(monkeypatch, first, second, amplitude="0.5")
    with np.load(second) as data:
        np.testing.assert_allclose(data["h_cascade"], np.full((1, 1, 16), 1024 * 0.5 * 0.5), rtol=1e-6)


def test_rephase_invalid_component_leg_keeps_only_direct_channel(tmp_path, monkeypatch):
    source, output = tmp_path / "source.npz", tmp_path / "new.npz"
    saved_components(source, valid=False)
    run_rephase(monkeypatch, source, output)
    with np.load(output) as data:
        assert np.count_nonzero(data["h_cascade"]) == 0
        np.testing.assert_array_equal(data["h_effective"], data["h_direct"])


def test_rephase_refuses_overwrite_and_invalid_amplitude(tmp_path, monkeypatch):
    source, output = tmp_path / "source.npz", tmp_path / "new.npz"
    saved_components(source)
    run_rephase(monkeypatch, source, output)
    with pytest.raises(FileExistsError):
        run_rephase(monkeypatch, source, output)
    with pytest.raises(ValueError, match="finite"):
        run_rephase(monkeypatch, source, tmp_path / "bad.npz", amplitude="nan")
    assert not (tmp_path / "bad.npz").exists()
