# -*- coding: utf-8 -*-
from __future__ import annotations

from .sionna_config import *
from .raytracing import global_angles_to_local_angles, tensor_to_numpy
from .path_dedup import geometry_dedup_metadata


def _as_numpy(value, *, dtype=None) -> np.ndarray:
    arr = np.asarray(tensor_to_numpy(value))
    return arr.astype(dtype, copy=False) if dtype is not None else arr


def _slice_path_link(arr, tx_idx: int, rx_idx: int, name: str) -> np.ndarray:
    """Slice a Sionna RT 2.0.1 per-path tensor to one Tx/Rx link.

    Official 2.0.1 layouts are either
      [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]
    or, for synthetic-array angle/delay tensors,
      [num_rx, num_tx, num_paths].

    Stage3 traces one spatial element with one polarization port (V or H), so
    the physical-array antenna indices are both zero. Dual-polarized VH/cross
    tensors require a port-aware channel representation; the current scalar
    reconstructed channel stores spatial elements only.
    """
    arr = _as_numpy(arr)
    if arr.ndim == 5:
        if arr.shape[1] != 1 or arr.shape[3] != 1:
            raise ValueError(
                f"{name}: scalar Stage3 reconstruction expects one polarization port "
                f"per 1x1 tracing element, got shape={arr.shape}. Use POLARIZATION=V or H; "
                "dual-polarization VH/cross requires port-aware channel reconstruction."
            )
        return arr[rx_idx, 0, tx_idx, 0, :]
    if arr.ndim == 3:
        return arr[rx_idx, tx_idx, :]
    raise ValueError(
        f"{name}: unexpected Sionna RT 2.0.1 path tensor shape={arr.shape}; "
        "expected 5D physical-array or 3D synthetic-array layout"
    )


def _slice_interaction_link(arr, tx_idx: int, rx_idx: int, name: str) -> np.ndarray:
    """Slice interactions/objects/primitives to [max_depth, num_paths]."""
    arr = _as_numpy(arr)
    if arr.ndim == 6:
        if arr.shape[2] != 1 or arr.shape[4] != 1:
            raise ValueError(
                f"{name}: scalar Stage3 reconstruction expects one polarization port "
                f"per 1x1 tracing element, got shape={arr.shape}. Use POLARIZATION=V or H; "
                "dual-polarization VH/cross requires port-aware channel reconstruction."
            )
        return arr[:, rx_idx, 0, tx_idx, 0, :]
    if arr.ndim == 4:
        return arr[:, rx_idx, tx_idx, :]
    raise ValueError(
        f"{name}: unexpected Sionna RT 2.0.1 interaction tensor shape={arr.shape}; "
        "expected 6D physical-array or 4D synthetic-array layout"
    )


def _slice_vertices_link(arr, tx_idx: int, rx_idx: int) -> np.ndarray:
    """Slice Paths.vertices to [max_depth, num_paths, 3]."""
    arr = _as_numpy(arr, dtype=np.float32)
    if arr.ndim == 7:
        if arr.shape[2] != 1 or arr.shape[4] != 1 or arr.shape[-1] != 3:
            raise ValueError(
                "vertices: scalar Stage3 reconstruction expects one polarization port "
                f"per 1x1 tracing element and xyz coordinates, got shape={arr.shape}. "
                "Use POLARIZATION=V or H; dual-polarization VH/cross requires port-aware "
                "channel reconstruction."
            )
        return arr[:, rx_idx, 0, tx_idx, 0, :, :]
    if arr.ndim == 5:
        if arr.shape[-1] != 3:
            raise ValueError(f"vertices: expected xyz coordinate axis, got shape={arr.shape}")
        return arr[:, rx_idx, tx_idx, :, :]
    raise ValueError(
        f"vertices: unexpected Sionna RT 2.0.1 tensor shape={arr.shape}; "
        "expected 7D physical-array or 5D synthetic-array layout"
    )


def _path_coefficient_tensor(paths) -> tuple[np.ndarray, np.ndarray]:
    """Return the official Sionna RT 2.0.1 real/imag Paths.a tensors."""
    raw = paths.a
    if not isinstance(raw, tuple) or len(raw) != 2:
        raise TypeError(
            "Sionna RT 2.0.1 Paths.a must be a (real, imag) tensor tuple"
        )
    return _as_numpy(raw[0], dtype=np.float32), _as_numpy(raw[1], dtype=np.float32)


def _baseband_from_passband(a: np.ndarray, tau: np.ndarray) -> np.ndarray:
    """Match Paths.cir(normalize_delays=False) per-path phase convention."""
    tau64 = np.asarray(tau, dtype=np.float64)
    tau_phase = np.where(tau64 < 0.0, 0.0, tau64)
    phase = np.exp(-1j * 2.0 * np.pi * float(FREQUENCY_HZ) * tau_phase)
    return (np.asarray(a, dtype=np.complex128) * phase).astype(np.complex64)


def _empty_rx_path_data() -> dict:
    zf = np.zeros((0,), dtype=np.float32)
    zc = np.zeros((0,), dtype=np.complex64)
    zpts = np.zeros((0, 3), dtype=np.float32)
    return {
        "a": zc,
        "tau": zf,
        "theta_r": zf,
        "phi_r": zf,
        "glob_theta_r": zf,
        "glob_phi_r": zf,
        "theta_t": zf,
        "phi_t": zf,
        "glob_theta_t": zf,
        "glob_phi_t": zf,
        "first_interaction_pos": zpts,
        "last_interaction_pos": zpts,
        "interaction_types": np.zeros((0, 0), dtype=np.uint8),
        "object_ids": np.zeros((0, 0), dtype=np.uint32),
        "primitive_ids": np.zeros((0, 0), dtype=np.uint32),
        "interaction_vertices_m": np.zeros((0, 0, 3), dtype=np.float32),
        "dedup_keep": np.zeros((0,), dtype=bool),
        "duplicate_of": np.zeros((0,), dtype=np.int32),
        "duplicate_group_size": np.zeros((0,), dtype=np.uint16),
        "dedup_reason": np.zeros((0,), dtype=np.uint8),
        "dedup_medoid_score": np.zeros((0,), dtype=np.float32),
        "path_type": np.zeros((0,), dtype=np.uint8),
        "los": np.zeros((0,), dtype=bool),
    }


def _extract_interaction_data(paths, tx_idx: int, rx_idx: int, n_paths: int):
    """Return Sionna RT 2.0.1 interaction geometry for one Tx/Rx link."""
    interactions = _slice_interaction_link(
        paths.interactions, tx_idx, rx_idx, "interactions"
    ).astype(np.uint32, copy=False)
    objects = _slice_interaction_link(
        paths.objects, tx_idx, rx_idx, "objects"
    ).astype(np.uint32, copy=False)
    primitives = _slice_interaction_link(
        paths.primitives, tx_idx, rx_idx, "primitives"
    ).astype(np.uint32, copy=False)
    if interactions.shape[-1] != n_paths:
        raise ValueError(
            f"interactions path count mismatch: {interactions.shape[-1]} != {n_paths}"
        )
    if objects.shape != interactions.shape or primitives.shape != interactions.shape:
        raise ValueError(
            f"interaction metadata shape mismatch: interactions={interactions.shape}, "
            f"objects={objects.shape}, primitives={primitives.shape}"
        )

    if interactions.shape[0] == 0:
        path_type = np.zeros((n_paths,), dtype=np.uint8)
    else:
        path_type = np.bitwise_or.reduce(interactions, axis=0).astype(np.uint8)
    los = path_type == 0

    first = np.full((n_paths, 3), np.nan, dtype=np.float32)
    last = np.full((n_paths, 3), np.nan, dtype=np.float32)
    vertices = _slice_vertices_link(paths.vertices, tx_idx, rx_idx)
    if vertices.shape[:2] != interactions.shape:
        raise ValueError(
            f"vertices/interactions shape mismatch: vertices={vertices.shape}, "
            f"interactions={interactions.shape}"
        )

    for p in range(n_paths):
        depth_idx = np.flatnonzero(interactions[:, p] != 0)
        if depth_idx.size:
            first[p] = vertices[depth_idx[0], p]
            last[p] = vertices[depth_idx[-1], p]
    return path_type, los, first, last, interactions, objects, primitives, vertices


def _extract_single_link_paths(paths, tx_idx: int, rx_idx: int, pose_info: dict) -> dict:
    """Extract one link using only the documented Sionna RT 2.0.1 Paths layout."""
    num_rx = int(pose_info.get("num_users", NUM_CAVS))
    num_tx = int(pose_info.get("num_tx", 1))
    if not (0 <= rx_idx < num_rx and 0 <= tx_idx < num_tx):
        raise IndexError(
            f"link index out of range: tx={tx_idx}/{num_tx}, rx={rx_idx}/{num_rx}"
        )

    rx_fwd = np.asarray(
        pose_info.get("rx_forward_all", pose_info["rx_forward"]), dtype=np.float32
    ).reshape(-1, 3)
    tx_fwd = np.asarray(
        pose_info.get("tx_forward_all", pose_info["tx_forward"]), dtype=np.float32
    ).reshape(-1, 3)

    glob_theta_r = _slice_path_link(paths.theta_r, tx_idx, rx_idx, "theta_r").astype(np.float32)
    glob_phi_r = _slice_path_link(paths.phi_r, tx_idx, rx_idx, "phi_r").astype(np.float32)
    glob_theta_t = _slice_path_link(paths.theta_t, tx_idx, rx_idx, "theta_t").astype(np.float32)
    glob_phi_t = _slice_path_link(paths.phi_t, tx_idx, rx_idx, "phi_t").astype(np.float32)
    tau = _slice_path_link(paths.tau, tx_idx, rx_idx, "tau").astype(np.float32)

    n_paths = int(glob_theta_r.shape[-1])
    if n_paths == 0:
        return _empty_rx_path_data()
    for name, vec in (
        ("phi_r", glob_phi_r),
        ("theta_t", glob_theta_t),
        ("phi_t", glob_phi_t),
        ("tau", tau),
    ):
        if vec.shape != (n_paths,):
            raise ValueError(f"{name}: expected ({n_paths},), got {vec.shape}")

    a_real, a_imag = _path_coefficient_tensor(paths)
    a_real = _slice_path_link(a_real, tx_idx, rx_idx, "a.real").astype(np.float32)
    a_imag = _slice_path_link(a_imag, tx_idx, rx_idx, "a.imag").astype(np.float32)
    if a_real.shape != (n_paths,) or a_imag.shape != (n_paths,):
        raise ValueError(
            f"Paths.a path count mismatch: real={a_real.shape}, imag={a_imag.shape}, n={n_paths}"
        )
    a_passband = (
        a_real.astype(np.complex64) + 1j * a_imag.astype(np.complex64)
    ).astype(np.complex64)
    a = _baseband_from_passband(a_passband, tau)

    valid = _slice_path_link(paths.valid, tx_idx, rx_idx, "valid").astype(bool)
    if valid.shape != (n_paths,):
        raise ValueError(f"valid: expected ({n_paths},), got {valid.shape}")

    (
        path_type, los, first_pts, last_pts,
        interaction_types, object_ids, primitive_ids, interaction_vertices,
    ) = _extract_interaction_data(paths, tx_idx, rx_idx, n_paths)

    theta_r, phi_r = global_angles_to_local_angles(
        glob_theta_r, glob_phi_r, rx_fwd[rx_idx]
    )
    theta_t, phi_t = global_angles_to_local_angles(
        glob_theta_t, glob_phi_t, tx_fwd[tx_idx]
    )

    mask = (
        valid
        & np.isfinite(a.real)
        & np.isfinite(a.imag)
        & np.isfinite(tau)
        & (tau >= 0.0)
        & np.isfinite(theta_r)
        & np.isfinite(phi_r)
        & np.isfinite(theta_t)
        & np.isfinite(phi_t)
        & np.isfinite(glob_theta_r)
        & np.isfinite(glob_phi_r)
        & np.isfinite(glob_theta_t)
        & np.isfinite(glob_phi_t)
    )
    a_out = a[mask].astype(np.complex64)
    a_passband_out = a_passband[mask].astype(np.complex64)
    tau_out = tau[mask].astype(np.float32)
    theta_r_out = theta_r[mask].astype(np.float32)
    phi_r_out = phi_r[mask].astype(np.float32)
    theta_t_out = theta_t[mask].astype(np.float32)
    phi_t_out = phi_t[mask].astype(np.float32)
    interactions_out = interaction_types[:, mask].astype(np.uint8)
    vertices_out = interaction_vertices[:, mask, :].astype(np.float32)
    dedup = geometry_dedup_metadata(
        interactions=interactions_out,
        vertices=vertices_out,
        objects=object_ids[:, mask],
        primitives=primitive_ids[:, mask],
        tau=tau_out,
        theta_r=theta_r_out,
        phi_r=phi_r_out,
        theta_t=theta_t_out,
        phi_t=phi_t_out,
        # Use the passband coefficient to apply the propagation delay phase once.
        coeff=a_passband_out,
    )
    return {
        "a": a_out,
        "tau": tau_out,
        "theta_r": theta_r_out,
        "phi_r": phi_r_out,
        "glob_theta_r": glob_theta_r[mask].astype(np.float32),
        "glob_phi_r": glob_phi_r[mask].astype(np.float32),
        "theta_t": theta_t_out,
        "phi_t": phi_t_out,
        "glob_theta_t": glob_theta_t[mask].astype(np.float32),
        "glob_phi_t": glob_phi_t[mask].astype(np.float32),
        "first_interaction_pos": first_pts[mask].astype(np.float32),
        "last_interaction_pos": last_pts[mask].astype(np.float32),
        # Preserve all raw valid RT paths and their full interaction metadata.
        # dedup_keep is consumed only by channel reconstruction.
        "interaction_types": interactions_out.T.copy(),
        "object_ids": object_ids[:, mask].T.astype(np.uint32),
        "primitive_ids": primitive_ids[:, mask].T.astype(np.uint32),
        "interaction_vertices_m": np.transpose(vertices_out, (1, 0, 2)).copy(),
        "dedup_keep": dedup["dedup_keep"].astype(bool),
        "duplicate_of": dedup["duplicate_of"].astype(np.int32),
        "duplicate_group_size": dedup["duplicate_group_size"].astype(np.uint16),
        "dedup_reason": dedup["dedup_reason"].astype(np.uint8),
        "dedup_medoid_score": dedup["dedup_medoid_score"].astype(np.float32),
        "path_type": path_type[mask].astype(np.uint8),
        "los": los[mask].astype(bool),
    }


def _extract_single_rx_paths(paths, rx_idx: int, pose_info: dict) -> dict:
    return _extract_single_link_paths(paths, tx_idx=0, rx_idx=rx_idx, pose_info=pose_info)


def extract_paths_data(paths, pose_info: dict) -> dict:
    """Extract path data for each requested link from batched Sionna devices."""
    link_specs = list(pose_info.get("link_specs") or [])
    if link_specs:
        link_users = [
            _extract_single_link_paths(
                paths,
                tx_idx=int(spec["tx_idx"]),
                rx_idx=int(spec["rx_idx"]),
                pose_info=pose_info,
            )
            for spec in link_specs
        ]
        first = link_users[0] if link_users else _empty_rx_path_data()
        return {
            "theta_t": first["theta_t"].astype(np.float32),
            "phi_t": first["phi_t"].astype(np.float32),
            "glob_theta_t": first["glob_theta_t"].astype(np.float32),
            "glob_phi_t": first["glob_phi_t"].astype(np.float32),
            "rx_users": link_users,
            "link_users": link_users,
            "num_users": int(pose_info.get("num_users", 0)),
            "num_links": len(link_users),
        }

    num_users = int(pose_info.get("num_users", NUM_CAVS))
    rx_users = [_extract_single_rx_paths(paths, rx_i, pose_info) for rx_i in range(num_users)]
    first = rx_users[0] if rx_users else _empty_rx_path_data()
    return {
        "theta_t": first["theta_t"].astype(np.float32),
        "phi_t": first["phi_t"].astype(np.float32),
        "glob_theta_t": first["glob_theta_t"].astype(np.float32),
        "glob_phi_t": first["glob_phi_t"].astype(np.float32),
        "rx_users": rx_users,
        "num_users": num_users,
    }


__all__ = [k for k in globals() if not k.startswith("__")]
