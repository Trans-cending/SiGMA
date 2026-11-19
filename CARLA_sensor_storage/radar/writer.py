"""Canonical complex64 FMCW IQ writer for the Stage III Sionna RCS backend."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict

import numpy as np


class RadarIQWriter:
    def __init__(self, profile):
        self.profile = profile

    def write(
        self,
        path: str,
        iq: np.ndarray,
        *,
        material_config_hash: str,
        frame_id: int,
        timestamp: float,
        trajectory_id: int,
        radar_backend: str = "sionna_rcs",
        extra_metadata: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        expected = (
            int(self.profile.num_rx),
            int(self.profile.num_adc_samples),
            int(self.profile.num_chirps),
        )
        if iq.shape != expected:
            raise ValueError(f"Radar IQ shape must be {expected}, got {iq.shape}")
        if iq.dtype != np.complex64:
            raise TypeError(f"Radar IQ dtype must be complex64, got {iq.dtype}")
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(out.suffix + f".tmp.{os.getpid()}")
        payload = dict(
                iq=iq,
                start_frequency_hz=np.float64(self.profile.start_frequency_hz),
                bandwidth_hz=np.float64(self.profile.bandwidth_hz),
                chirp_slope_hz_s=np.float64(self.profile.chirp_slope_hz_s),
                adc_sampling_rate_hz=np.float64(self.profile.adc_sampling_rate_hz),
                ramp_end_time_s=np.float64(self.profile.ramp_end_time_s),
                idle_time_s=np.float64(self.profile.idle_time_s),
                chirp_repetition_s=np.float64(self.profile.chirp_repetition_s),
                sampled_bandwidth_hz=np.float64(self.profile.sampled_bandwidth_hz),
                num_adc_samples=np.int32(self.profile.num_adc_samples),
                num_chirps=np.int32(self.profile.num_chirps),
                num_rx=np.int32(self.profile.num_rx),
                rx_spacing_m=np.float64(self.profile.rx_spacing_m),
                rx_spacing_wavelength=np.float64(self.profile.rx_spacing_wavelength),
                horizontal_fov_deg=np.float64(self.profile.horizontal_fov_deg),
                radar_profile=np.asarray(self.profile.name),
                radar_profile_version=np.asarray(self.profile.version),
                material_config_hash=np.asarray(str(material_config_hash)),
                frame_id=np.int64(frame_id),
                timestamp=np.float64(timestamp),
                trajectory_id=np.int64(trajectory_id),
                radar_backend=np.asarray(str(radar_backend)),
            )
        for key, value in dict(extra_metadata or {}).items():
            if key == "iq" or key in payload:
                raise ValueError(f"extra radar metadata collides with reserved key {key!r}")
            payload[str(key)] = np.asarray(value)
        with open(tmp, "wb") as handle:
            np.savez_compressed(handle, **payload)
        os.replace(tmp, out)
        return {
            "radar_backend": str(radar_backend),
            "radar_profile": self.profile.name,
            "radar_profile_version": self.profile.version,
            "iq_shape": list(expected),
            "iq_dtype": "complex64",
            "material_config_hash": str(material_config_hash),
        }
