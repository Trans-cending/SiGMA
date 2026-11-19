#!/usr/bin/env python3
"""Deterministic Range--Doppler and Angle--Doppler maps from canonical IQ.

The canonical SiGMA IQ layout is ``[rx, adc_sample, chirp]``. This module
applies windowed FFTs to the generated complex IQ while preserving the source
propagation paths. Angle processing assumes the horizontal
ULA used by the canonical DeepSense radar profile.
"""
from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
from pathlib import Path

import numpy as np


SPEED_OF_LIGHT_M_S = 299_792_458.0


@dataclass(frozen=True)
class RadarMaps:
    range_doppler_power: np.ndarray       # [range, doppler]
    angle_doppler_power: np.ndarray       # [angle, doppler]
    range_axis_m: np.ndarray              # [range]
    doppler_axis_hz: np.ndarray           # [doppler]
    angle_axis_deg: np.ndarray            # [angle]
    selected_range_bin: int

    @property
    def angle_probability(self) -> np.ndarray:
        score = np.sum(self.angle_doppler_power, axis=1, dtype=np.float64)
        total = float(np.sum(score))
        if total <= 0.0:
            return np.full(score.shape, 1.0 / max(score.size, 1), np.float64)
        return score / total


def _hann(length: int, enabled: bool) -> np.ndarray:
    return np.hanning(length).astype(np.float64) if enabled else np.ones(length)


def compute_radar_maps(
    iq: np.ndarray,
    *,
    chirp_slope_hz_s: float,
    adc_sampling_rate_hz: float,
    chirp_repetition_s: float,
    start_frequency_hz: float,
    rx_spacing_wavelength: float,
    angle_bins: int = 181,
    selected_range_bin: int | None = None,
    window: bool = True,
) -> RadarMaps:
    """Return physical axes and power maps for ``iq[rx,adc,chirp]``.

    Analytic complex IQ uses the full range-bin interval ``[0,N_adc)`` with
    the stored positive-beat convention. Doppler and spatial FFTs are centered
    with ``fftshift``. The angle axis is the ULA broadside azimuth convention
    ``sin(angle)=u``.
    """
    value = np.asarray(iq)
    if value.ndim != 3 or not np.iscomplexobj(value):
        raise ValueError(f"iq must be complex [rx,adc,chirp], got {value.shape}/{value.dtype}")
    n_rx, n_adc, n_chirps = value.shape
    if min(n_rx, n_adc, n_chirps, int(angle_bins)) <= 0:
        raise ValueError("all IQ dimensions and angle_bins must be positive")
    if min(chirp_slope_hz_s, adc_sampling_rate_hz, chirp_repetition_s,
           start_frequency_hz, rx_spacing_wavelength) <= 0.0:
        raise ValueError("radar sampling/carrier/spacing values must be positive")

    range_fft = np.fft.fft(
        value * _hann(n_adc, window)[None, :, None], axis=1
    )
    doppler_fft = np.fft.fftshift(
        np.fft.fft(
            range_fft * _hann(n_chirps, window)[None, None, :], axis=2
        ), axes=2,
    )
    rd_power = np.sum(np.abs(doppler_fft) ** 2, axis=0, dtype=np.float64)

    if selected_range_bin is None:
        selected_range_bin = int(np.argmax(np.sum(rd_power, axis=1)))
    selected_range_bin = int(selected_range_bin)
    if not 0 <= selected_range_bin < rd_power.shape[0]:
        raise ValueError(
            f"selected_range_bin={selected_range_bin} outside [0,{rd_power.shape[0]})"
        )

    # Preserve slow-time phase, then transform the horizontal receive ULA.
    range_gate = range_fft[:, selected_range_bin, :]
    angle_doppler = np.fft.fftshift(
        np.fft.fft(
            np.fft.fftshift(
                np.fft.fft(
                    range_gate * _hann(n_chirps, window)[None, :], axis=1
                ), axes=1,
            # Keep the full physical aperture.  np.hanning(4) is
            # [0, 0.75, 0.75, 0] and would discard half of the canonical
            # four-element DeepSense receive ULA, unnecessarily degrading the
            # angular feature used by downstream beam prediction.
            ) * np.ones((n_rx, 1), dtype=np.float64),
            n=int(angle_bins), axis=0,
        ), axes=0,
    )
    ad_power = np.abs(angle_doppler) ** 2

    beat_hz = np.arange(rd_power.shape[0], dtype=np.float64) * (
        float(adc_sampling_rate_hz) / n_adc
    )
    range_axis_m = beat_hz * SPEED_OF_LIGHT_M_S / (
        2.0 * float(chirp_slope_hz_s)
    )
    doppler_axis_hz = np.fft.fftshift(
        np.fft.fftfreq(n_chirps, d=float(chirp_repetition_s))
    )
    spatial_frequency = np.fft.fftshift(np.fft.fftfreq(int(angle_bins)))
    direction_cosine = np.clip(
        spatial_frequency / float(rx_spacing_wavelength), -1.0, 1.0
    )
    angle_axis_deg = np.degrees(np.arcsin(direction_cosine))
    return RadarMaps(
        range_doppler_power=rd_power.astype(np.float32),
        angle_doppler_power=ad_power.astype(np.float32),
        range_axis_m=range_axis_m.astype(np.float32),
        doppler_axis_hz=doppler_axis_hz.astype(np.float32),
        angle_axis_deg=angle_axis_deg.astype(np.float32),
        selected_range_bin=selected_range_bin,
    )


def load_iq_and_compute(
    path: Path, *, angle_bins: int = 181,
    selected_range_bin: int | None = None,
) -> RadarMaps:
    with np.load(path, allow_pickle=False) as data:
        required = {
            "iq", "chirp_slope_hz_s", "adc_sampling_rate_hz",
            "chirp_repetition_s", "start_frequency_hz", "rx_spacing_wavelength",
        }
        missing = sorted(required - set(data.files))
        if missing:
            raise ValueError(f"{path} lacks required radar fields: {missing}")
        return compute_radar_maps(
            data["iq"],
            chirp_slope_hz_s=float(data["chirp_slope_hz_s"]),
            adc_sampling_rate_hz=float(data["adc_sampling_rate_hz"]),
            chirp_repetition_s=float(data["chirp_repetition_s"]),
            start_frequency_hz=float(data["start_frequency_hz"]),
            rx_spacing_wavelength=float(data["rx_spacing_wavelength"]),
            angle_bins=angle_bins,
            selected_range_bin=selected_range_bin,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--angle-bins", type=int, default=181)
    parser.add_argument(
        "--range-bin", type=int,
        help="range gate used for Angle--Doppler; default selects the strongest gate",
    )
    args = parser.parse_args()
    maps = load_iq_and_compute(
        args.input, angle_bins=args.angle_bins, selected_range_bin=args.range_bin)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        range_doppler_power=maps.range_doppler_power,
        angle_doppler_power=maps.angle_doppler_power,
        range_axis_m=maps.range_axis_m,
        doppler_axis_hz=maps.doppler_axis_hz,
        angle_axis_deg=maps.angle_axis_deg,
        angle_probability=maps.angle_probability.astype(np.float32),
        selected_range_bin=np.int32(maps.selected_range_bin),
        source_iq=np.asarray(str(args.input.resolve())),
    )
    peak_range, peak_doppler = np.unravel_index(
        int(np.argmax(maps.range_doppler_power)), maps.range_doppler_power.shape
    )
    peak_angle, peak_ad_doppler = np.unravel_index(
        int(np.argmax(maps.angle_doppler_power)), maps.angle_doppler_power.shape
    )
    report = {
        "input": str(args.input.resolve()),
        "output": str(args.output.resolve()),
        "range_doppler_shape": list(maps.range_doppler_power.shape),
        "angle_doppler_shape": list(maps.angle_doppler_power.shape),
        "selected_range_bin": maps.selected_range_bin,
        "peak_range_m": float(maps.range_axis_m[peak_range]),
        "peak_doppler_hz": float(maps.doppler_axis_hz[peak_doppler]),
        "peak_angle_deg": float(maps.angle_axis_deg[peak_angle]),
        "peak_angle_doppler_hz": float(maps.doppler_axis_hz[peak_ad_doppler]),
        "angle_probability_sum": float(np.sum(maps.angle_probability)),
        "beam_prediction_note": (
            "angle_probability is a deterministic angular feature for a downstream "
            "beam predictor; four Rx elements do not provide a 64-beam ground-truth label"
        ),
    }
    report_path = args.report or args.output.with_suffix(".json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
