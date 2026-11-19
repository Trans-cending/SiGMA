#!/usr/bin/env python3
"""Simulator-free regression tests for optional RIS support."""
from __future__ import annotations

from pathlib import Path
import tempfile

import numpy as np

from mosaic_core.ris import (
    element_world_coordinates, estimate_component_storage_bytes,
    normalize_ris_config, panel_geometry, ris_config_hash,
)
from Sionna_channel_storage.ris_channel import (
    audit_far_field, effective_channel, panel_coupling_audit,
    require_far_field, synthesize_ris,
)
from Sionna_channel_storage.ris_writer import (
    load_ris_paths, ris_file_complete, ris_path_complete,
    save_ris_channels, save_ris_paths,
)
from Sionna_channel_storage.ris_pipeline import endpoint_receiver_doppler_hz


def config(*, amplitude=0.8, phase_mode="focus"):
    return normalize_ris_config({
        "enabled": True,
        "frequency_hz": 28e9,
        "nodes": [{
            "id": "ris_001", "latitude": 33.0, "longitude": -111.0,
            "altitude": 4.0,
            "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
            "panel": {"num_rows": 2, "num_cols": 3,
                      "element_spacing_lambda": 0.5, "thickness_m": 0.02},
        }],
        "channel": {"mode": "effective", "reflection_amplitude": amplitude,
                    "phase_mode": phase_mode},
    })


def test_disabled_is_backward_compatible():
    cfg = normalize_ris_config(None)
    assert cfg["enabled"] is False
    assert cfg["channel"]["mode"] == "off"


def test_geometry_and_coordinates():
    cfg = config()
    geom = panel_geometry(cfg)
    assert geom.num_elements == 6
    assert np.isclose(geom.boundary_width_m, 3 * 0.5 * geom.wavelength_m)
    assert np.isclose(geom.center_span_width_m, 2 * 0.5 * geom.wavelength_m)
    coords = element_world_coordinates([10, 20, 30], cfg["nodes"][0]["rotation"], geom)
    assert coords.shape == (6, 3)
    assert np.allclose(np.mean(coords, axis=0), [10, 20, 30])
    assert np.isclose(np.ptp(coords[:, 1]), geom.center_span_width_m)
    assert np.isclose(np.ptp(coords[:, 2]), geom.center_span_height_m)


def test_cascade_matches_reference_and_zero_amplitude():
    rng = np.random.default_rng(9)
    h_br = (rng.normal(size=(4, 6, 3)) + 1j*rng.normal(size=(4, 6, 3))).astype(np.complex64)
    h_ru = (rng.normal(size=(4, 2, 6)) + 1j*rng.normal(size=(4, 2, 6))).astype(np.complex64)
    direct = (rng.normal(size=(4, 2, 3)) + 1j*rng.normal(size=(4, 2, 3))).astype(np.complex64)
    products = synthesize_ris(direct, h_br, h_ru, config()["channel"])
    reference = np.empty_like(direct)
    for k in range(4):
        reference[k] = h_ru[k] @ np.diag(products["phi"][k]) @ h_br[k]
    assert np.allclose(products["h_cascade"], reference, rtol=2e-5, atol=2e-5)
    zero = synthesize_ris(direct, h_br, h_ru, config(amplitude=0.0)["channel"])
    assert np.array_equal(zero["h_effective"], direct)
    assert not np.any(zero["h_cascade"])


def test_far_field_and_panel_front_side():
    audit = audit_far_field([10, 0, 0], [0, 0, 0], [10, 5, 0], 2.0)
    require_far_field(audit)
    near = audit_far_field([1, 0, 0], [0, 0, 0], [10, 0, 0], 2.0)
    try:
        require_far_field(near)
    except ValueError:
        pass
    else:
        raise AssertionError("near-field configuration did not fail")
    front = panel_coupling_audit([10, 0, 0], [0, 0, 0], [10, 5, 0], [1, 0, 0])
    assert front.front_side_valid and 0.0 < front.field_amplitude_factor <= 1.0
    back = panel_coupling_audit([-10, 0, 0], [0, 0, 0], [10, 0, 0], [1, 0, 0])
    assert not back.front_side_valid and back.field_amplitude_factor == 0.0


def test_endpoint_receiver_doppler():
    frequency_hz = 30e9
    wavelength_m = 299_792_458.0 / frequency_hz
    toward = endpoint_receiver_doppler_hz(
        [np.pi / 2], [0.0], [10.0, 0.0, 0.0], frequency_hz)
    orthogonal = endpoint_receiver_doppler_hz(
        [np.pi / 2], [0.0], [0.0, 10.0, 0.0], frequency_hz)
    assert np.allclose(toward, [10.0 / wavelength_m], rtol=1e-6)
    assert np.allclose(orthogonal, [0.0], atol=1e-5)


def test_persistence_rebuild_and_hash_invalidation():
    cfg = config()
    cfg_hash = ris_config_hash(cfg)
    rng = np.random.default_rng(3)
    h_br = (rng.normal(size=(2, 6, 2)) + 1j*rng.normal(size=(2, 6, 2))).astype(np.complex64)
    h_ru = (rng.normal(size=(2, 1, 6)) + 1j*rng.normal(size=(2, 1, 6))).astype(np.complex64)
    direct = np.ones((2, 1, 2), np.complex64)
    products = synthesize_ris(direct, h_br, h_ru, cfg["channel"])
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        channel_path = root / "ris.npz"
        save_ris_channels(
            channel_path, config_hash=cfg_hash, h_direct=direct,
            h_br=h_br, h_ru=h_ru, phi=products["phi"],
            h_cascade=products["h_cascade"], h_effective=products["h_effective"],
            metadata={"mode": "effective"}, save_full_component_channels=True,
        )
        assert ris_file_complete(
            channel_path, config_hash=cfg_hash, mode="effective",
            save_full_component_channels=True)
        assert not ris_file_complete(channel_path, config_hash="different", mode="effective")
        with np.load(channel_path) as data:
            rebuilt, cascaded = effective_channel(
                data["h_direct"], data["h_br"], data["h_ru"], data["phi"])
            assert np.allclose(rebuilt, data["h_effective"])
            assert np.allclose(cascaded, data["h_cascade"])

        path_file = root / "leg.npz"
        save_ris_paths(path_file, {
            "coefficient": np.ones(1, np.complex64),
            "delay_s": np.zeros(1, np.float32),
            "doppler_hz": np.zeros(1, np.float32),
            "aod_rad": np.zeros((1, 2), np.float32),
            "aoa_rad": np.zeros((1, 2), np.float32),
        }, config_hash=cfg_hash)
        assert ris_path_complete(path_file, config_hash=cfg_hash)
        loaded = load_ris_paths(path_file)
        assert loaded["rx_users"][0]["a"].shape == (1,)

    assert estimate_component_storage_bytes(
        num_subcarriers=4, num_ris_elements=6, num_bs=2, num_ue=1) == 576


if __name__ == "__main__":
    test_disabled_is_backward_compatible()
    test_geometry_and_coordinates()
    test_cascade_matches_reference_and_zero_amplitude()
    test_far_field_and_panel_front_side()
    test_endpoint_receiver_doppler()
    test_persistence_rebuild_and_hash_invalidation()
    print("RIS_TESTS=PASS")
