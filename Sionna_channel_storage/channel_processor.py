# -*- coding: utf-8 -*-
from .sionna_config import *

from .raytracing import *

from .path_extractor import _empty_rx_path_data

_SUBCARRIER_FREQUENCY_OFFSETS: dict[tuple[float, float, int], np.ndarray] = {}

def subcarrier_frequency_offsets(cfg: dict | None = None) -> np.ndarray:
    if cfg is None:
        bandwidth = float(BANDWIDTH_HZ)
        n_sc = int(N_SUBCARRIERS)
    else:
        bandwidth = float(cfg.get("bandwidth_hz", BANDWIDTH_HZ))
        n_sc = int(cfg.get("num_subcarriers", N_SUBCARRIERS))
    key = (bandwidth, float(cfg.get("center_frequency_hz", FREQUENCY_HZ)) if cfg else float(FREQUENCY_HZ), n_sc)
    arr = _SUBCARRIER_FREQUENCY_OFFSETS.get(key)
    if arr is None:
        spacing = bandwidth / float(n_sc)
        idx = np.arange(n_sc, dtype=np.float64) - (n_sc - 1) / 2.0
        arr = idx * spacing
        _SUBCARRIER_FREQUENCY_OFFSETS[key] = arr
    return arr

def synthesize_ofdm_channel(paths_data: dict, cfg: dict, pose_info: dict) -> np.ndarray:
    """Return H with shape (1, n_rx_ant, n_tx_ant, n_subcarriers)."""
    n_rx = int(cfg["rx_rows"] * cfg["rx_cols"])
    n_tx = int(cfg["tx_rows"] * cfg["tx_cols"])
    n_sc = int(cfg.get("num_subcarriers", N_SUBCARRIERS))
    rx_if_filter = bool(ARRAY_IF_FILTER and n_rx > 1)
    tx_if_filter = bool(ARRAY_IF_FILTER and n_tx > 1)
    h_freq = np.zeros((1, n_rx, n_tx, n_sc), dtype=np.complex64)

    rx_data = paths_data.get("rx_users", [_empty_rx_path_data()])[0]
    a = rx_data["a"].astype(np.complex128)
    tau = rx_data["tau"].astype(np.float64)
    theta_r = rx_data["theta_r"].astype(np.float64)
    phi_r = rx_data["phi_r"].astype(np.float64)
    theta_t = rx_data.get("theta_t", paths_data["theta_t"]).astype(np.float64)
    phi_t = rx_data.get("phi_t", paths_data["phi_t"]).astype(np.float64)

    # Keep the raw path dataset intact while preventing exact/near-exact RT
    # geometry duplicates from being coherently counted multiple times in H.
    # Legacy path files use all paths when this mask is absent.
    dedup_keep = np.asarray(
        rx_data.get("dedup_keep", np.ones((a.shape[0],), dtype=bool)),
        dtype=bool,
    ).reshape(-1)
    if dedup_keep.shape != (a.shape[0],):
        raise ValueError(
            f"dedup_keep shape mismatch: expected {(a.shape[0],)}, got {dedup_keep.shape}"
        )
    if not np.all(dedup_keep):
        a = a[dedup_keep]
        tau = tau[dedup_keep]
        theta_r = theta_r[dedup_keep]
        phi_r = phi_r[dedup_keep]
        theta_t = theta_t[dedup_keep]
        phi_t = phi_t[dedup_keep]

    if a.size == 0 or tau.size == 0 or theta_r.size == 0 or theta_t.size == 0:
        return h_freq

    for k, f in enumerate(subcarrier_frequency_offsets(cfg)):
        a_k = a * np.exp(-1j * 2.0 * np.pi * tau * float(f))
        if CHANNEL_RECONSTRUCTION_MODE == "nearfield":
            if reconstruct_nearfield_H_upa_squeeze is None:
                raise ImportError("CHANNEL_RECONSTRUCTION_MODE=nearfield requires reconstruct_nearfield_H_upa_squeeze in utils/data_utils.py")
            H = reconstruct_nearfield_H_upa_squeeze(
                a_k, theta_r, phi_r, theta_t, phi_t,
                cfg["rx_rows"], cfg["rx_cols"], cfg["tx_rows"], cfg["tx_cols"],
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                fc=float(cfg.get("center_frequency_hz", FREQUENCY_HZ)),
                d_ratio=ARRAY_SPACING_WL,
                if_filter=ARRAY_IF_FILTER,
                rx_if_filter=rx_if_filter,
                tx_if_filter=tx_if_filter,
                tx_pos=pose_info["tx_pos"],
                rx_pos=np.asarray(pose_info.get("rx_pos_all", pose_info["rx_pos"])).reshape(-1, 3)[0],
                tx_forward=pose_info["tx_forward"],
                rx_forward=np.asarray(pose_info.get("rx_forward_all", pose_info["rx_forward"])).reshape(-1, 3)[0],
                first_interaction_pos=(
                    np.asarray(rx_data.get("first_interaction_pos"))[dedup_keep]
                    if rx_data.get("first_interaction_pos") is not None
                    else None
                ),
                last_interaction_pos=(
                    np.asarray(rx_data.get("last_interaction_pos"))[dedup_keep]
                    if rx_data.get("last_interaction_pos") is not None
                    else None
                ),
            )
        else:
            H = reconstruct_farfield_H_upa_squeeze(
                a_k, theta_r, phi_r, theta_t, phi_t,
                cfg["rx_rows"], cfg["rx_cols"], cfg["tx_rows"], cfg["tx_cols"],
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                fc=float(cfg.get("center_frequency_hz", FREQUENCY_HZ)),
                d_ratio=ARRAY_SPACING_WL,
                if_filter=ARRAY_IF_FILTER,
                rx_if_filter=rx_if_filter,
                tx_if_filter=tx_if_filter,
            )
        h_freq[0, :, :, k] = H.astype(np.complex64)
    return h_freq

def bartlett_spectrum_from_vector(
    h_vec: np.ndarray,
    rows: int,
    cols: int,
    *,
    if_filter: bool | None = None,
):
    """Return a Bartlett spectrum consistent with Stage3 array IF filtering.

    A multi-element array with ARRAY_IF_FILTER enabled is physically restricted
    to local +Z (front hemisphere).  A 1x1 element remains omnidirectional and
    therefore keeps the full sphere.  The input channel vector is already
    reconstructed from filtered propagation paths; the mask here only removes
    the ULA/UPA front-back steering ambiguity from beam visualization/peak search.
    """
    A = generate_gt_bartlett_codebook(
        num_rows=rows,
        num_cols=cols,
        img_h=BARTLETT_IMG_H,
        img_w=BARTLETT_IMG_W,
        d_ratio=ARRAY_SPACING_WL,
    )
    A = np.asarray(A)
    h = np.asarray(h_vec).reshape(-1)
    P = np.abs(np.conj(A).T @ h) ** 2
    P = P.reshape(BARTLETT_IMG_H, BARTLETT_IMG_W).astype(np.float64, copy=False)

    if if_filter is None:
        if_filter = bool(ARRAY_IF_FILTER and int(rows) * int(cols) > 1)
    if if_filter:
        front = generate_gt_bartlett_front_mask(BARTLETT_IMG_H, BARTLETT_IMG_W)
        P = np.where(front, P, np.nan)

    finite = np.isfinite(P)
    if not np.any(finite):
        return np.full(P.shape, np.nan, dtype=np.float32)
    peak = float(np.nanmax(P))
    if peak > 0.0:
        P = P / peak
    floor_lin = 10 ** (BARTLETT_DB_FLOOR / 10.0)
    P_db = 10.0 * np.log10(np.maximum(P, floor_lin))
    return P_db.astype(np.float32)


__all__=[k for k in globals() if not k.startswith("__")]
