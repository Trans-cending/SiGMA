"""Reject undefined IQ inputs and controls from removed radar backends."""
import numpy as np
import pytest
from CARLA_sensor_storage.radar.profiles import get_radar_profile
from Radar_channel_storage.config import radar_profile_overrides, normalize_sionna_rcs_config
from Radar_channel_storage.fmcw import RadarPathData, synthesize_fmcw_iq


def test_invalid_coefficients_fail_before_producing_iq():
    data = RadarPathData(np.full((4, 1), complex(float("nan"), 0)), np.array([1e-7]), np.array([0.]))
    with pytest.raises(ValueError, match="coefficients"):
        synthesize_fmcw_iq(data, get_radar_profile())


@pytest.mark.parametrize("settings", [{"adc_sampling_rate_hz": float("inf")},
                                     {"idle_time_s": float("nan")}, {"num_rx": True}, {"num_chirps": 1.5}])
def test_invalid_waveform_parameters_are_rejected(settings):
    with pytest.raises(ValueError):
        get_radar_profile(overrides=settings)


def test_removed_noise_controls_and_string_flags_cannot_silently_change_experiment():
    with pytest.raises(ValueError, match="Removed"):
        radar_profile_overrides({"noise_enabled": True})
    with pytest.raises(ValueError, match="boolean"):
        normalize_sionna_rcs_config({"sionna_rcs": {"los": "false"}})
