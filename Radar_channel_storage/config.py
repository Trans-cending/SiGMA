"""Strict RCS and FMCW settings for the isolated Stage III radar worker.

Waveform overrides cover parameters actually used by synthesize_fmcw_iq.
Old voxel/scatterer/noise controls belonging to the removed backend are rejected.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class SionnaRCSConfig:
    max_depth: int = 2
    buffer_size_per_sp: int = 100_000
    samples_per_sp: int = 100_000
    synthetic_array: bool = True
    los: bool = True
    specular_reflection: bool = True
    refraction: bool = False
    include_background_paths: bool = False
    background_max_depth: int = 2
    background_samples_per_src: int = 100_000
    seed: int = 0
    deterministic: bool = True
    object_type: str = "vehicle-multi-sp"
    model_type: int = 2
    random_sigma_s: bool = False
    random_phases: bool = False
    random_xpr: bool = False
    target_margin_m: float = 2.0
    output_schema_version: int = 1

    def validate(self) -> "SionnaRCSConfig":
        for name in ("synthetic_array", "los", "specular_reflection", "refraction",
                     "include_background_paths", "deterministic", "random_sigma_s",
                     "random_phases", "random_xpr"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"sionna_rcs.{name} must be boolean")
        for name in (
            "max_depth", "buffer_size_per_sp", "samples_per_sp",
            "background_max_depth", "background_samples_per_src",
        ):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"sionna_rcs.{name} must be >= 1")
        if type(self.seed) is not int or self.seed < 0:
            raise ValueError("sionna_rcs.seed must be >= 0")
        if self.object_type != "vehicle-multi-sp":
            raise ValueError(
                "The first sionna_rcs release supports only "
                "object_type='vehicle-multi-sp'"
            )
        if int(self.model_type) not in (1, 2):
            raise ValueError("sionna_rcs.model_type must be 1 or 2")
        if not math.isfinite(float(self.target_margin_m)) or float(self.target_margin_m) < 0.0:
            raise ValueError("sionna_rcs.target_margin_m must be >= 0")
        if not bool(self.synthetic_array):
            raise ValueError(
                "sionna_rcs currently requires synthetic_array=true so one "
                "geometric path set is expanded to the canonical Rx array"
            )
        return self

    @property
    def config_hash(self) -> str:
        payload = json.dumps(asdict(self), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def normalize_sionna_rcs_config(sensor_config: Mapping[str, Any]) -> SionnaRCSConfig:
    raw = dict(sensor_config.get("sionna_rcs") or {})
    allowed = set(SionnaRCSConfig.__dataclass_fields__)
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError("Unknown sionna_rcs setting(s): " + ", ".join(unknown))
    return SionnaRCSConfig(**raw).validate()


def radar_profile_overrides(sensor_config: Mapping[str, Any]) -> dict[str, Any]:
    unsupported = {"noise_enabled", "noise_stddev", "max_scatterers", "angular_voxel_deg", "range_voxel_m"} & set(sensor_config)
    if unsupported:
        raise ValueError("Removed radar backend option(s): " + ", ".join(sorted(unsupported)))
    keys = {
        "start_frequency_hz", "bandwidth_hz", "chirp_slope_hz_s",
        "adc_sampling_rate_hz", "ramp_end_time_s", "idle_time_s",
        "num_adc_samples", "num_chirps", "num_rx",
        "rx_spacing_wavelength", "horizontal_fov_deg",
    }
    out = {key: sensor_config[key] for key in keys if key in sensor_config}
    if "horizontal_fov" in sensor_config:
        out["horizontal_fov_deg"] = sensor_config["horizontal_fov"]
    return out
