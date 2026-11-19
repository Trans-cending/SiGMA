"""Validated FMCW waveform parameters for Stage III Sionna RCS synthesis."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Any, Dict


@dataclass(frozen=True)
class DeepSenseRadarProfile:
    name: str = "deepsense_s9"
    version: str = "2"
    start_frequency_hz: float = 77e9
    bandwidth_hz: float = 768.768e6
    chirp_slope_hz_s: float = 15.015e12
    adc_sampling_rate_hz: float = 5e6
    ramp_end_time_s: float = 60e-6
    idle_time_s: float = 5e-6
    num_adc_samples: int = 256
    num_chirps: int = 128
    num_rx: int = 4
    rx_spacing_wavelength: float = 0.5
    horizontal_fov_deg: float = 120.0
    output_dtype: str = "complex64"
    output_layout: str = "rx_sample_chirp"

    @property
    def wavelength_m(self) -> float:
        return 299792458.0 / float(self.start_frequency_hz)

    @property
    def rx_spacing_m(self) -> float:
        return self.wavelength_m * float(self.rx_spacing_wavelength)

    @property
    def adc_duration_s(self) -> float:
        return float(self.num_adc_samples) / float(self.adc_sampling_rate_hz)

    @property
    def ramp_duration_s(self) -> float:
        return float(self.ramp_end_time_s)

    @property
    def chirp_repetition_s(self) -> float:
        return float(self.ramp_end_time_s) + float(self.idle_time_s)

    @property
    def sampled_bandwidth_hz(self) -> float:
        return float(self.chirp_slope_hz_s) * self.adc_duration_s

    def validate(self) -> "DeepSenseRadarProfile":
        positive = {
            "start_frequency_hz": self.start_frequency_hz,
            "bandwidth_hz": self.bandwidth_hz,
            "chirp_slope_hz_s": self.chirp_slope_hz_s,
            "adc_sampling_rate_hz": self.adc_sampling_rate_hz,
            "ramp_end_time_s": self.ramp_end_time_s,
            "num_adc_samples": self.num_adc_samples,
            "num_chirps": self.num_chirps,
            "num_rx": self.num_rx,
            "rx_spacing_wavelength": self.rx_spacing_wavelength,
        }
        invalid = [key for key, value in positive.items() if isinstance(value, bool)
                   or not math.isfinite(float(value)) or float(value) <= 0.0]
        if invalid:
            raise ValueError("Radar profile values must be positive: " + ", ".join(invalid))
        for key in ("num_adc_samples", "num_chirps", "num_rx"):
            if type(getattr(self, key)) is not int:
                raise ValueError(f"{key} must be a positive integer")
        if not math.isfinite(float(self.idle_time_s)) or float(self.idle_time_s) < 0.0:
            raise ValueError("idle_time_s must be >= 0")
        if not 0.0 < float(self.horizontal_fov_deg) <= 180.0:
            raise ValueError("horizontal_fov_deg must be in (0, 180]")
        if self.output_dtype != "complex64" or self.output_layout != "rx_sample_chirp":
            raise ValueError("Canonical radar output must be complex64 [rx,sample,chirp]")
        if self.adc_duration_s > self.ramp_duration_s:
            raise ValueError(
                "ADC acquisition duration exceeds full chirp ramp duration: "
                f"T_adc={self.adc_duration_s}, T_ramp={self.ramp_duration_s}"
            )
        swept_bandwidth_hz = float(self.chirp_slope_hz_s) * self.ramp_duration_s
        if float(self.bandwidth_hz) > swept_bandwidth_hz:
            raise ValueError(
                "Radar bandwidth exceeds the chirp sweep during ramp_end_time_s: "
                f"bandwidth={self.bandwidth_hz}, swept={swept_bandwidth_hz}"
            )
        return self


_PROFILES = {"deepsense_s9": DeepSenseRadarProfile()}


def get_radar_profile(name: str = "deepsense_s9", overrides: Dict[str, Any] = None) -> DeepSenseRadarProfile:
    key = str(name or "deepsense_s9").strip().lower()
    if key not in _PROFILES:
        raise ValueError(f"Unknown radar_profile={name!r}; available={sorted(_PROFILES)}")
    profile = _PROFILES[key]
    values = {k: v for k, v in dict(overrides or {}).items() if v is not None}
    if values:
        allowed = set(profile.__dataclass_fields__) - {"name", "version"}
        unknown = sorted(set(values) - allowed)
        if unknown:
            raise ValueError("Unknown radar profile override(s): " + ", ".join(unknown))
        profile = replace(profile, **values)
    return profile.validate()
