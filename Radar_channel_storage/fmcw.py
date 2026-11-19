"""Convert Sionna RCS path parameters into canonical FMCW raw IQ.

Sionna supplies propagation/RCS coefficients, delays, Doppler, and antenna
responses. This module applies waveform timing to those coefficients,
preserving the supplied antenna phases and Doppler.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class RadarPathData:
    # Baseband coefficients at t=0, including carrier-delay phase and the
    # configured Tx/Rx array response. Shape [N_rx,N_paths].
    coefficient: np.ndarray
    delay_s: np.ndarray
    doppler_hz: np.ndarray
    target_id: np.ndarray | None = None

    def validate(self, num_rx: int) -> "RadarPathData":
        coefficient = np.asarray(self.coefficient)
        delay = np.asarray(self.delay_s)
        doppler = np.asarray(self.doppler_hz)
        if coefficient.ndim != 2 or coefficient.shape[0] != int(num_rx):
            raise ValueError(
                f"coefficient must be [N_rx,N_paths] with N_rx={num_rx}; got {coefficient.shape}"
            )
        if not np.all(np.isfinite(coefficient)):
            raise ValueError("path coefficients must be finite")
        if self.target_id is not None and np.asarray(self.target_id).shape != (coefficient.shape[1],):
            raise ValueError("target_id must have one entry per path")
        if delay.shape != (coefficient.shape[1],) or doppler.shape != delay.shape:
            raise ValueError(
                f"delay/doppler must be [N_paths]; got {delay.shape}/{doppler.shape}"
            )
        if np.any(delay < 0.0) or not np.all(np.isfinite(delay)):
            raise ValueError("path delays must be finite and non-negative")
        if not np.all(np.isfinite(doppler)):
            raise ValueError("path Doppler values must be finite")
        return self


def synthesize_fmcw_iq(paths: RadarPathData, profile) -> np.ndarray:
    """Synthesize dechirped IQ with shape [N_rx,N_adc,N_chirps].

    The input coefficient is the baseband-equivalent Sionna coefficient from
    ``Paths.cir(normalize_delays=False)`` at time zero, so carrier phase and
    array phase are already present.  We add only FMCW beat phase and Sionna's
    path Doppler evolution (including fast-time evolution).
    """
    profile = profile.validate()
    paths.validate(profile.num_rx)
    coefficient = np.asarray(paths.coefficient, dtype=np.complex128)
    delay = np.asarray(paths.delay_s, dtype=np.float64)
    doppler = np.asarray(paths.doppler_hz, dtype=np.float64)
    shape = (int(profile.num_rx), int(profile.num_adc_samples), int(profile.num_chirps))
    if delay.size == 0:
        return np.zeros(shape, dtype=np.complex64)

    fast_time = np.arange(profile.num_adc_samples, dtype=np.float64) / float(
        profile.adc_sampling_rate_hz)
    slow_time = np.arange(profile.num_chirps, dtype=np.float64) * float(
        profile.chirp_repetition_s)
    # Keep the source positive-range-FFT convention.  The
    # Sionna CIR coefficient already contains exp(-j*2*pi*fc*tau) and the
    # physical Rx array phase, so neither is applied here a second time.  Only
    # the FMCW beat term and Sionna's official Doppler evolution are added.
    # This convention
    # preserves existing downstream range-bin interpretation.
    beat = np.exp(
        1j * 2.0 * math.pi
        * float(profile.chirp_slope_hz_s)
        * delay[:, None]
        * fast_time[None, :]
        - 1j * math.pi * float(profile.chirp_slope_hz_s) * delay[:, None] ** 2
    )
    absolute_time = fast_time[None, :, None] + slow_time[None, None, :]
    doppler_response = np.exp(
        1j * 2.0 * math.pi * doppler[:, None, None] * absolute_time
    )
    iq = np.einsum(
        "rp,pn,pnm->rnm", coefficient, beat, doppler_response, optimize=True
    )
    if iq.shape != shape:
        raise RuntimeError(f"Sionna FMCW synthesis returned {iq.shape}, expected {shape}")
    return iq.astype(np.complex64)


def expected_range_bin(range_m: float, profile) -> float:
    delay = 2.0 * float(range_m) / 299_792_458.0
    return (
        float(profile.chirp_slope_hz_s) * delay
        / float(profile.adc_sampling_rate_hz)
        * int(profile.num_adc_samples)
    )


def expected_doppler_bin(doppler_hz: float, profile) -> float:
    return (
        float(doppler_hz)
        * float(profile.chirp_repetition_s)
        * int(profile.num_chirps)
    )
