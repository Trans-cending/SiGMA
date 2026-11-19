#!/usr/bin/env python3
"""Simulator-independent CPU tests for Sionna-RCS adapters."""
from __future__ import annotations

from pathlib import Path
import tempfile
from types import SimpleNamespace

import numpy as np
import yaml

from CARLA_sensor_storage.radar.profiles import get_radar_profile
from CARLA_sensor_storage.radar.writer import RadarIQWriter
from mosaic_core.rf_materials import material_config_hash
from Radar_channel_storage.config import normalize_sionna_rcs_config
from Radar_channel_storage.fmcw import (
    RadarPathData, expected_doppler_bin, expected_range_bin,
    synthesize_fmcw_iq,
)
from Radar_channel_storage.radar_maps import compute_radar_maps
from Radar_channel_storage.scene_inputs import build_vehicle_target_specs
from Radar_channel_storage.sionna_rcs_dataset_generator import _active_actor_ids


def test_config_is_reproducible_and_strict():
    cfg = normalize_sionna_rcs_config({"sionna_rcs": {"seed": 11}})
    assert cfg.seed == 11 and cfg.object_type == "vehicle-multi-sp"
    assert cfg.config_hash == normalize_sionna_rcs_config(
        {"sionna_rcs": {"seed": 11}}
    ).config_hash
    try:
        normalize_sionna_rcs_config({"sionna_rcs": {"unknown": 1}})
    except ValueError:
        pass
    else:
        raise AssertionError("unknown Sionna-RCS setting was accepted")


def test_single_target_range_and_doppler_bins():
    profile = get_radar_profile()
    # Choose an exactly representable positive range bin.
    range_bin = 23
    delay = (
        range_bin * profile.adc_sampling_rate_hz
        / (profile.chirp_slope_hz_s * profile.num_adc_samples)
    )
    doppler_bin = 7
    doppler = doppler_bin / (profile.chirp_repetition_s * profile.num_chirps)
    paths = RadarPathData(
        coefficient=np.ones((profile.num_rx, 1), np.complex64),
        delay_s=np.asarray([delay], np.float32),
        doppler_hz=np.asarray([doppler], np.float32),
        target_id=np.asarray([42], np.int64),
    )
    iq = synthesize_fmcw_iq(paths, profile)
    assert iq.shape == (4, 256, 128) and iq.dtype == np.complex64
    range_spectrum = np.abs(np.fft.fft(iq[0, :, 0]))
    doppler_spectrum = np.abs(np.fft.fft(iq[0, 0, :]))
    assert int(np.argmax(range_spectrum)) == range_bin
    assert int(np.argmax(doppler_spectrum)) == doppler_bin
    assert np.isclose(expected_range_bin(delay * 299_792_458.0 / 2.0, profile), range_bin)
    assert np.isclose(expected_doppler_bin(doppler, profile), doppler_bin)


def test_multiple_targets_and_writer_contract():
    profile = get_radar_profile()
    bins = np.asarray([11, 41], dtype=np.float64)
    delays = bins * profile.adc_sampling_rate_hz / (
        profile.chirp_slope_hz_s * profile.num_adc_samples
    )
    paths = RadarPathData(
        coefficient=np.ones((profile.num_rx, 2), np.complex64),
        delay_s=delays.astype(np.float32),
        doppler_hz=np.zeros(2, np.float32),
        target_id=np.asarray([1, 2], np.int64),
    )
    iq = synthesize_fmcw_iq(paths, profile)
    peaks = set(np.argsort(np.abs(np.fft.fft(iq[0, :, 0])))[-2:].tolist())
    assert peaks == {11, 41}
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp) / "radar_000001.npz"
        RadarIQWriter(profile).write(
            str(path), iq, material_config_hash=material_config_hash(),
            frame_id=1, timestamp=0.1, trajectory_id=2,
            radar_backend="sionna_rcs",
            extra_metadata={"sionna_rt_version": "2.2.0"},
        )
        with np.load(path) as data:
            assert data["iq"].shape == (4, 256, 128)
            assert data["iq"].dtype == np.complex64
            assert str(data["radar_backend"]) == "sionna_rcs"
            assert str(data["sionna_rt_version"]) == "2.2.0"


def test_range_doppler_and_angle_doppler_maps():
    profile = get_radar_profile()
    range_bin = 29
    doppler_bin = -9
    angle_deg = 24.0
    delay = range_bin * profile.adc_sampling_rate_hz / (
        profile.chirp_slope_hz_s * profile.num_adc_samples
    )
    doppler = doppler_bin / (profile.chirp_repetition_s * profile.num_chirps)
    element = np.arange(profile.num_rx, dtype=np.float64)
    coefficient = np.exp(
        1j * 2.0 * np.pi * profile.rx_spacing_wavelength
        * np.sin(np.radians(angle_deg)) * element
    )[:, None].astype(np.complex64)
    iq = synthesize_fmcw_iq(RadarPathData(
        coefficient=coefficient,
        delay_s=np.asarray([delay], np.float32),
        doppler_hz=np.asarray([doppler], np.float32),
        target_id=np.asarray([7], np.int64),
    ), profile)
    maps = compute_radar_maps(
        iq,
        chirp_slope_hz_s=profile.chirp_slope_hz_s,
        adc_sampling_rate_hz=profile.adc_sampling_rate_hz,
        chirp_repetition_s=profile.chirp_repetition_s,
        start_frequency_hz=profile.start_frequency_hz,
        rx_spacing_wavelength=profile.rx_spacing_wavelength,
        angle_bins=361,
    )
    r_idx, d_idx = np.unravel_index(
        np.argmax(maps.range_doppler_power), maps.range_doppler_power.shape)
    a_idx, ad_idx = np.unravel_index(
        np.argmax(maps.angle_doppler_power), maps.angle_doppler_power.shape)
    assert r_idx == range_bin
    assert d_idx == (profile.num_chirps // 2 + doppler_bin)
    assert ad_idx == d_idx
    assert abs(float(maps.angle_axis_deg[a_idx]) - angle_deg) < 1.0
    assert np.isclose(np.sum(maps.angle_probability), 1.0)

    # Analytic complex IQ retains the upper half of the positive beat-bin
    # convention. Targets beyond bin 128 remain visible in the map.
    far_bin = 181
    far_delay = far_bin * profile.adc_sampling_rate_hz / (
        profile.chirp_slope_hz_s * profile.num_adc_samples
    )
    far_iq = synthesize_fmcw_iq(RadarPathData(
        coefficient=np.ones((profile.num_rx, 1), np.complex64),
        delay_s=np.asarray([far_delay], np.float32),
        doppler_hz=np.zeros(1, np.float32),
    ), profile)
    far_maps = compute_radar_maps(
        far_iq,
        chirp_slope_hz_s=profile.chirp_slope_hz_s,
        adc_sampling_rate_hz=profile.adc_sampling_rate_hz,
        chirp_repetition_s=profile.chirp_repetition_s,
        start_frequency_hz=profile.start_frequency_hz,
        rx_spacing_wavelength=profile.rx_spacing_wavelength,
    )
    assert far_maps.range_doppler_power.shape == (256, 128)
    assert int(np.unravel_index(
        np.argmax(far_maps.range_doppler_power), far_maps.range_doppler_power.shape
    )[0]) == far_bin


def test_world_state_vehicle_id_mesh_and_velocity_mapping():
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        mesh = root / "car.ply"
        mesh.write_bytes(b"ply\n")
        dynamic = root / "dynamic_000001.xml"
        dynamic.write_text(
            '<scene><shape type="ply" id="mesh-Vehicle_7__body">'
            f'<string name="filename" value="{mesh}"/>'
            '</shape></scene>', encoding="utf-8")
        world = {
            "nodes": {"cav": {}},
            "scene": {"vehicles": {7: {
                "bp_id": "vehicle.test", "location": [1, -2, 3],
                "center": [0, 0, 1], "extent": [2, 1, 0.5],
                "angle": [0, 90, 0], "velocity": [4, -5, 0],
            }}},
        }
        specs = build_vehicle_target_specs(world, dynamic)
        assert len(specs) == 1 and specs[0].actor_id == 7
        assert np.allclose(specs[0].velocity_sionna_m_s, [4, 5, 0])
        assert np.allclose(specs[0].dimensions_m, [4, 2, 1])


def test_horizontal_and_vertical_fov_filtering():
    specs = [
        SimpleNamespace(actor_id=1, position_sionna_m=np.asarray([20.0, 0.0, 0.0])),
        SimpleNamespace(actor_id=2, position_sionna_m=np.asarray([20.0, 20.0, 0.0])),
        SimpleNamespace(actor_id=3, position_sionna_m=np.asarray([20.0, 0.0, 10.0])),
    ]
    active = _active_actor_ids(
        specs, np.zeros(3), np.zeros(3), 100.0,
        horizontal_fov_deg=60.0, vertical_fov_deg=20.0,
    )
    assert active == {1}


if __name__ == "__main__":
    test_config_is_reproducible_and_strict()
    test_single_target_range_and_doppler_bins()
    test_multiple_targets_and_writer_contract()
    test_range_doppler_and_angle_doppler_maps()
    test_world_state_vehicle_id_mesh_and_velocity_mapping()
    test_horizontal_and_vertical_fov_filtering()
    print("SIONNA_RCS_ADAPTER_TESTS=PASS")
