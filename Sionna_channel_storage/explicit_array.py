"""Trace every physical antenna pair for near-field CSI.

The migrated center-path reconstruction is retained for far-field mode. This
adapter configures actual PlanarArrays and calls the same PathSolver with
synthetic_array=False. CIR coefficients already contain carrier/array phase.
Run through ``python -m sigma stage3 -c configs/town03.yaml``.
"""
from pathlib import Path
import numpy as np

from . import sionna_config as settings
from .raytracing import create_scene_for_ray_tracing, import_sionna_objects
from .channel_processor import subcarrier_frequency_offsets


def cir_to_ofdm(coefficients, delays, frequencies):
    """Convert one physical link [R,T,P] CIR to [1,R,T,K] CSI."""
    a = np.asarray(coefficients)
    tau = np.asarray(delays)
    if a.ndim != 3 or tau.shape != a.shape:
        raise ValueError(f"Explicit CIR requires matching [R,T,P]: {a.shape}/{tau.shape}")
    valid = np.isfinite(tau) & (tau >= 0) & np.isfinite(a)
    a = np.where(valid, a, 0).astype(np.complex128)
    tau = np.where(valid, tau, 0).astype(np.float64)
    output = np.empty((*a.shape[:2], len(frequencies)), np.complex64)
    for k, frequency in enumerate(frequencies):
        output[..., k] = np.sum(a * np.exp(-2j * np.pi * tau * frequency), axis=-1)
    return output[None]


def trace_explicit_channel(xml: Path, pose: dict, cfg: dict, path_output: Path):
    scene, _, solver_class = create_scene_for_ray_tracing(xml, pose)
    _, array_class, *_ = import_sionna_objects()
    for side in ("tx", "rx"):
        setattr(scene, f"{side}_array", array_class(
            num_rows=int(cfg[f"{side}_rows"]), num_cols=int(cfg[f"{side}_cols"]),
            vertical_spacing=settings.ARRAY_SPACING_WL,
            horizontal_spacing=settings.ARRAY_SPACING_WL,
            pattern=settings.ANTENNA_PATTERN, polarization=settings.POLARIZATION))
    paths = solver_class()(
        scene=scene, max_depth=settings.RT_MAX_DEPTH,
        max_num_paths_per_src=settings.RT_MAX_NUM_PATHS_PER_SRC,
        samples_per_src=settings.RT_SAMPLES_PER_SRC, synthetic_array=False,
        los=settings.RT_LOS, specular_reflection=settings.RT_SPECULAR_REFLECTION,
        diffuse_reflection=settings.RT_DIFFUSE_REFLECTION, refraction=settings.RT_REFRACTION,
        diffraction=settings.RT_DIFFRACTION, edge_diffraction=settings.RT_EDGE_DIFFRACTION,
        seed=settings.RT_SEED)
    a, tau = paths.cir(normalize_delays=False, out_type="numpy")
    a, tau = np.asarray(a), np.asarray(tau)
    # Sionna physical layout: [rx,rx_ant,tx,tx_ant,path,time].
    if a.ndim != 6 or a.shape[0] != 1 or a.shape[2] != 1 or a.shape[-1] != 1:
        raise ValueError(f"Unexpected physical-array CIR shape {a.shape}")
    if tau.shape != a.shape[:-1]:
        raise ValueError(f"Physical-array delay shape {tau.shape} differs from CIR {a.shape}")
    coefficient, delay = a[0, :, 0, :, :, 0], tau[0, :, 0, :, :]
    front_mask = np.ones(coefficient.shape, dtype=bool)
    if settings.ARRAY_IF_FILTER:
        for side, count in (("tx", coefficient.shape[1]), ("rx", coefficient.shape[0])):
            if count == 1:
                continue
            theta = np.asarray(getattr(paths, f"theta_{'t' if side == 'tx' else 'r'}"))[0, :, 0, :, :]
            phi = np.asarray(getattr(paths, f"phi_{'t' if side == 'tx' else 'r'}"))[0, :, 0, :, :]
            forward = np.asarray(pose[f"{side}_forward"], dtype=float)
            dot = (np.sin(theta) * np.cos(phi) * forward[0]
                   + np.sin(theta) * np.sin(phi) * forward[1] + np.cos(theta) * forward[2])
            front_mask &= dot >= -1e-7
        coefficient = np.where(front_mask, coefficient, 0)
    path_output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path_output, a=coefficient, tau_s=delay,
                       synthetic_array=False, front_mask=front_mask,
                       array_front_filter=settings.ARRAY_IF_FILTER,
                       axis_order="rx_antenna,tx_antenna,path")
    return cir_to_ofdm(coefficient, delay, subcarrier_frequency_offsets(cfg))
