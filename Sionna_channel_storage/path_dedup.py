# -*- coding: utf-8 -*-
from __future__ import annotations

import numpy as np

from .sionna_config import (
    PATH_GEOMETRY_DEDUP_ENABLED,
    PATH_DEDUP_ANGLE_TOL_RAD,
    PATH_DEDUP_COEFF_MAG_REL_TOL,
    PATH_DEDUP_COEFF_PHASE_TOL_RAD,
    PATH_DEDUP_DELAY_TOL_S,
    PATH_DEDUP_VERTEX_SCALE_M,
)

# Sionna RT 2.0.1 InteractionType.SPECULAR. Keep this lightweight module free
# from a heavy Sionna import during tests/tooling.
_SPECULAR_INTERACTION = np.uint32(1)
DEDUP_REASON_NONE = np.uint8(0)
DEDUP_REASON_EXACT_SPECULAR_CHAIN = np.uint8(1)


def _pairwise_absolute(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    return np.abs(values[:, None] - values[None, :])


def _pairwise_wrapped(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    return np.abs(np.angle(np.exp(1j * (values[:, None] - values[None, :]))))


def _positive_scale(name: str, value: float) -> float:
    value = float(value)
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and > 0, got {value}")
    return value


def _specular_chain_key(
    idx: int,
    interactions: np.ndarray,
    objects: np.ndarray,
    primitives: np.ndarray,
) -> tuple[tuple[int, int, int, int], ...] | None:
    """Return the ordered pure-specular interaction/object/primitive chain.

    LoS uses an empty interaction chain and remains individually retained.
    Diffuse, diffraction, refraction, and mixed-mechanism paths are also left
    untouched. The depth index prevents sparse interaction layouts from
    aliasing accidentally.
    """
    row = np.asarray(interactions[:, idx], dtype=np.uint32).reshape(-1)
    active = np.flatnonzero(row != 0)
    if active.size == 0:
        return None
    if not np.all(row[active] == _SPECULAR_INTERACTION):
        return None
    return tuple(
        (
            int(depth),
            int(row[depth]),
            int(objects[depth, idx]),
            int(primitives[depth, idx]),
        )
        for depth in active
    )


def _specular_chain_medoid(
    indices: list[int],
    interactions: np.ndarray,
    vertices: np.ndarray,
    tau: np.ndarray,
    theta_r: np.ndarray,
    phi_r: np.ndarray,
    theta_t: np.ndarray,
    phi_t: np.ndarray,
    coeff: np.ndarray,
) -> tuple[int, np.ndarray]:
    """Choose the most central numeric realization of one exact chain."""
    idx = np.asarray(indices, dtype=np.int64)
    if idx.size == 1:
        return int(idx[0]), np.zeros((1,), dtype=np.float64)

    delay_scale = _positive_scale("PATH_DEDUP_DELAY_TOL_S", PATH_DEDUP_DELAY_TOL_S)
    angle_scale = _positive_scale("PATH_DEDUP_ANGLE_TOL_RAD", PATH_DEDUP_ANGLE_TOL_RAD)
    mag_scale = _positive_scale(
        "PATH_DEDUP_COEFF_MAG_REL_TOL", PATH_DEDUP_COEFF_MAG_REL_TOL
    )
    phase_scale = _positive_scale(
        "PATH_DEDUP_COEFF_PHASE_TOL_RAD", PATH_DEDUP_COEFF_PHASE_TOL_RAD
    )
    vertex_scale = _positive_scale(
        "PATH_DEDUP_VERTEX_SCALE_M", PATH_DEDUP_VERTEX_SCALE_M
    )

    components = [
        _pairwise_absolute(tau[idx]) / delay_scale,
        _pairwise_absolute(theta_r[idx]) / angle_scale,
        _pairwise_wrapped(phi_r[idx]) / angle_scale,
        _pairwise_absolute(theta_t[idx]) / angle_scale,
        _pairwise_wrapped(phi_t[idx]) / angle_scale,
    ]

    chain_coeff = np.asarray(coeff[idx], dtype=np.complex128)
    magnitude = np.abs(chain_coeff)
    magnitude_denominator = np.maximum(
        np.maximum(magnitude[:, None], magnitude[None, :]), 1e-30
    )
    components.append(
        _pairwise_absolute(magnitude) / magnitude_denominator / mag_scale
    )
    phase_distance = _pairwise_wrapped(np.angle(chain_coeff))
    phase_distance[
        (magnitude[:, None] <= 1e-30) | (magnitude[None, :] <= 1e-30)
    ] = 0.0
    components.append(phase_distance / phase_scale)

    active_depths = np.flatnonzero(interactions[:, idx[0]] != 0)
    active_vertices = vertices[active_depths][:, idx, :]
    vertex_delta = (
        active_vertices[:, :, None, :] - active_vertices[:, None, :, :]
    )
    vertex_distance = np.linalg.norm(vertex_delta, axis=-1).mean(axis=0)
    components.append(vertex_distance / vertex_scale)

    pair_distance = np.sqrt(
        np.mean(np.square(np.stack(components, axis=0)), axis=0)
    )
    scores = pair_distance.sum(axis=1) / float(idx.size - 1)
    return int(idx[int(np.argmin(scores))]), scores


def geometry_dedup_metadata(
    *,
    interactions: np.ndarray,
    vertices: np.ndarray,
    objects: np.ndarray,
    primitives: np.ndarray,
    tau: np.ndarray,
    theta_r: np.ndarray,
    phi_r: np.ndarray,
    theta_t: np.ndarray,
    phi_t: np.ndarray,
    coeff: np.ndarray,
) -> dict:
    """Group repeated pure-specular exact chains while retaining raw paths.

    The returned dedup_keep mask is intended for channel reconstruction.
    Raw paths remain present in storage together with duplicate metadata.
    Only pure specular-reflection paths are eligible. For a fixed Tx/Rx pair,
    an exact ordered interaction-type/object-id/primitive-id chain identifies
    one physical image-method solution, so every repeated realization of that
    chain belongs to one group. Numeric delay, local AoA/AoD, passband complex
    coefficient, and interaction vertices select a deterministic medoid; they
    preserve the identity group. Diffuse Monte-Carlo paths, LOS, refraction,
    diffraction, mixed-mechanism paths, and different primitive chains are
    always retained.
    """
    coeff = np.asarray(coeff, dtype=np.complex64).reshape(-1)
    n = int(coeff.shape[0])
    keep = np.ones((n,), dtype=bool)
    duplicate_of = np.full((n,), -1, dtype=np.int32)
    group_size = np.ones((n,), dtype=np.uint16)
    dedup_reason = np.full((n,), DEDUP_REASON_NONE, dtype=np.uint8)
    medoid_score = np.zeros((n,), dtype=np.float32)
    if (not PATH_GEOMETRY_DEDUP_ENABLED) or n < 2:
        return {
            "dedup_keep": keep,
            "duplicate_of": duplicate_of,
            "duplicate_group_size": group_size,
            "dedup_reason": dedup_reason,
            "dedup_medoid_score": medoid_score,
        }

    interactions = np.asarray(interactions, dtype=np.uint32)
    vertices = np.asarray(vertices, dtype=np.float32)
    objects = np.asarray(objects, dtype=np.uint32)
    primitives = np.asarray(primitives, dtype=np.uint32)
    tau = np.asarray(tau, dtype=np.float64).reshape(n)
    theta_r = np.asarray(theta_r, dtype=np.float64).reshape(n)
    phi_r = np.asarray(phi_r, dtype=np.float64).reshape(n)
    theta_t = np.asarray(theta_t, dtype=np.float64).reshape(n)
    phi_t = np.asarray(phi_t, dtype=np.float64).reshape(n)
    if interactions.ndim != 2 or interactions.shape[1] != n:
        raise ValueError(
            f"dedup interactions must be [depth,num_paths], got {interactions.shape}"
        )
    if objects.shape != interactions.shape or primitives.shape != interactions.shape:
        raise ValueError(
            f"dedup object/primitive shape mismatch: interactions={interactions.shape}, "
            f"objects={objects.shape}, primitives={primitives.shape}"
        )
    if vertices.shape != (interactions.shape[0], n, 3):
        raise ValueError(
            f"dedup vertices must be [depth,num_paths,3], got {vertices.shape} "
            f"for interactions={interactions.shape}"
        )

    buckets: dict[tuple[tuple[int, int, int, int], ...], list[int]] = {}
    for idx in range(n):
        key = _specular_chain_key(idx, interactions, objects, primitives)
        if key is not None:
            buckets.setdefault(key, []).append(idx)

    for indices in buckets.values():
        if len(indices) < 2:
            continue
        representative, scores = _specular_chain_medoid(
            indices,
            interactions,
            vertices,
            tau,
            theta_r,
            phi_r,
            theta_t,
            phi_t,
            coeff,
        )
        size = np.uint16(min(len(indices), np.iinfo(np.uint16).max))
        for position, idx in enumerate(indices):
            group_size[idx] = size
            medoid_score[idx] = np.float32(scores[position])
            if idx != representative:
                keep[idx] = False
                duplicate_of[idx] = representative
                dedup_reason[idx] = DEDUP_REASON_EXACT_SPECULAR_CHAIN

    return {
        "dedup_keep": keep,
        "duplicate_of": duplicate_of,
        "duplicate_group_size": group_size,
        "dedup_reason": dedup_reason,
        "dedup_medoid_score": medoid_score,
    }


__all__ = [k for k in globals() if not k.startswith("__")]
