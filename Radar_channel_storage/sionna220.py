"""Pinned Sionna RT 2.2 RCS API with explicitly configured Mitsuba backend.

Run through sigma stage3; radio.variant selects the shared CPU/GPU variant.
"""
from __future__ import annotations

from importlib import metadata
import os
from typing import Any

import numpy as np

from .fmcw import RadarPathData


REQUIRED_SIONNA_RT_VERSION = "2.2.0"


def require_sionna_rt_220() -> str:
    try:
        version = metadata.version("sionna-rt")
    except metadata.PackageNotFoundError as exc:
        raise RuntimeError(
            "The sionna_rcs radar backend requires its isolated sionna-rt==2.2.0 environment"
        ) from exc
    if version != REQUIRED_SIONNA_RT_VERSION:
        raise RuntimeError(
            f"sionna_rcs requires sionna-rt=={REQUIRED_SIONNA_RT_VERSION}; found {version}. "
            "Do not run it in the communication Stage3 environment."
        )
    return version


def import_sionna_rcs_api():
    require_sionna_rt_220()
    import mitsuba as mi
    variant = os.environ.get("MITSUBA_VARIANT", "").strip()
    if variant and mi.variant() != variant:
        mi.set_variant(variant)
    from sionna.rt import PathSolver, PlanarArray, Receiver, Transmitter, load_scene
    from sionna.rt.rcs import RCSSolver, TR38901SensingTarget
    return (
        load_scene, PlanarArray, Transmitter, Receiver, PathSolver,
        RCSSolver, TR38901SensingTarget,
    )


def tensor_numpy(value, dtype=None) -> np.ndarray:
    if hasattr(value, "numpy"):
        value = value.numpy()
    else:
        try:
            import drjit as dr
            value = dr.detach(value)
        except Exception:
            pass
    out = np.asarray(value)
    return out.astype(dtype, copy=False) if dtype is not None else out


def require_scene_frequency(scene, frequency_hz: float) -> None:
    """Validate the frequency of a reused scene while retaining material state.

    Frequency is set once before adding sensing targets. Sionna 2.2 retains
    their absorber material after removal; reassigning frequency would call
    an unsupported frequency_update method on that material on frame two.
    """
    current = tensor_numpy(scene.frequency, np.float64).reshape(-1)
    if current.size != 1 or not np.isclose(current[0], frequency_hz, rtol=1e-6, atol=1.0):
        raise ValueError(f"Radar scene/profile frequency mismatch: {current} vs {frequency_hz}; load a new scene")


def remove_sensing_targets(scene, targets) -> None:
    """Remove temporary geometry AND its unused, auto-created materials.

    Scene.remove(objects) retains materials in Sionna 2.2. A later frame
    recreating the same actor otherwise collides with '<actor>-material'.
    Materials referenced by scene geometry remain in the scene.
    """
    materials = {target.radio_material.name: target.radio_material for target in targets}
    scene.remove(targets)
    for name, material in materials.items():
        if scene.get(name) is material and not material.is_used:
            scene.remove(name)


def extract_radar_paths(paths, *, num_rx: int) -> tuple[RadarPathData, dict[str, np.ndarray]]:
    """Extract one monostatic/bistatic link from official Sionna 2.2 Paths.

    RCSSolver's ``cir`` applies carrier-delay and array phases once; FMCW
    synthesis preserves those phases.
    """
    a, tau = paths.cir(normalize_delays=False, out_type="numpy")
    a = np.asarray(a)
    tau = np.asarray(tau)
    doppler = tensor_numpy(paths.doppler, np.float32)
    # Expected physical-array layout: [rx,rx_ant,tx,tx_ant,path,time].
    if a.ndim != 6 or a.shape[0] != 1 or a.shape[2] != 1 or a.shape[3] != 1:
        raise ValueError(f"unexpected RCS CIR layout {a.shape}; expected [1,Nrx,1,1,P,1]")
    coeff = a[0, :, 0, 0, :, 0]
    if coeff.shape[0] != int(num_rx):
        raise ValueError(f"RCS path N_rx mismatch: {coeff.shape[0]} != {num_rx}")
    if tau.ndim == 5:
        delay = tau[0, 0, 0, 0, :]
    elif tau.ndim == 3:
        delay = tau[0, 0, :]
    else:
        raise ValueError(f"unexpected RCS delay layout {tau.shape}")
    if doppler.ndim == 5:
        doppler_vec = doppler[0, 0, 0, 0, :]
    elif doppler.ndim == 3:
        doppler_vec = doppler[0, 0, :]
    else:
        raise ValueError(f"unexpected RCS Doppler layout {doppler.shape}")
    valid = np.isfinite(delay) & (delay >= 0.0) & np.isfinite(doppler_vec)
    coeff = coeff[:, valid]
    data = RadarPathData(
        coefficient=coeff.astype(np.complex64),
        delay_s=np.asarray(delay[valid], np.float32),
        doppler_hz=np.asarray(doppler_vec[valid], np.float32),
    )
    def path_vector(value, dtype):
        array = tensor_numpy(value, dtype)
        if array.shape[-1] != valid.size:
            raise ValueError(
                f"path component has path axis {array.shape[-1]}, expected {valid.size}"
            )
        return array.reshape(-1, array.shape[-1])[0, valid]

    interactions = tensor_numpy(paths.interactions, np.uint32)
    objects = tensor_numpy(paths.objects, np.uint32)
    primitives = tensor_numpy(paths.primitives, np.uint32)
    vertices = tensor_numpy(paths.vertices, np.float32)
    if interactions.shape[-1] != valid.size or objects.shape != interactions.shape:
        raise ValueError(
            f"unexpected path-component shapes interactions={interactions.shape} objects={objects.shape}"
        )

    # Collapse endpoint/antenna dimensions. With synthetic arrays all copies
    # describe the same geometric path, so the first one is canonical.
    depth = int(interactions.shape[0])
    interactions_2d = interactions.reshape(depth, -1, valid.size)[:, 0, :][:, valid].T
    objects_2d = objects.reshape(depth, -1, valid.size)[:, 0, :][:, valid].T
    primitives_2d = primitives.reshape(depth, -1, valid.size)[:, 0, :][:, valid].T
    if vertices.shape[0] != depth or vertices.shape[-2] != valid.size or vertices.shape[-1] != 3:
        raise ValueError(f"unexpected RCS vertices layout {vertices.shape}")
    vertices_3d = vertices.reshape(depth, -1, valid.size, 3)[:, 0, :, :][:, valid, :]
    vertices_3d = np.moveaxis(vertices_3d, 0, 1)

    from sionna.rt.constants import InteractionType
    sensing_code = int(InteractionType.SENSING)
    target_index = np.full(int(np.sum(valid)), -1, dtype=np.int32)
    for p in range(target_index.size):
        hit = np.flatnonzero(interactions_2d[p] == sensing_code)
        if hit.size:
            target_index[p] = int(objects_2d[p, hit[0]])

    details = {
        "theta_t_rad": path_vector(paths.theta_t, np.float32),
        "phi_t_rad": path_vector(paths.phi_t, np.float32),
        "theta_r_rad": path_vector(paths.theta_r, np.float32),
        "phi_r_rad": path_vector(paths.phi_r, np.float32),
        "interaction_types": interactions_2d.astype(np.uint8, copy=False),
        "object_ids": objects_2d,
        "primitive_ids": primitives_2d,
        "interaction_vertices_m": vertices_3d,
        "sensing_target_index": target_index,
    }
    return data, details
