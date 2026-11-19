"""Far-field RIS component reconstruction and cascaded-channel algebra."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True)
class FarFieldAudit:
    bs_ris_distance_m: float
    ris_ue_distance_m: float
    fraunhofer_distance_m: float
    bs_ris_valid: bool
    ris_ue_valid: bool

    @property
    def valid(self) -> bool:
        return self.bs_ris_valid and self.ris_ue_valid

    def as_dict(self) -> dict[str, Any]:
        return {
            "bs_ris_distance_m": self.bs_ris_distance_m,
            "ris_ue_distance_m": self.ris_ue_distance_m,
            "fraunhofer_distance_m": self.fraunhofer_distance_m,
            "bs_ris_valid": self.bs_ris_valid,
            "ris_ue_valid": self.ris_ue_valid,
            "valid": self.valid,
        }


@dataclass(frozen=True)
class PanelCouplingAudit:
    incidence_cosine: float
    departure_cosine: float
    field_amplitude_factor: float
    front_side_valid: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "incidence_cosine": self.incidence_cosine,
            "departure_cosine": self.departure_cosine,
            "field_amplitude_factor": self.field_amplitude_factor,
            "front_side_valid": self.front_side_valid,
        }


def audit_far_field(
    bs_xyz: np.ndarray,
    ris_xyz: np.ndarray,
    ue_xyz: np.ndarray,
    fraunhofer_distance_m: float,
) -> FarFieldAudit:
    bs_ris = float(np.linalg.norm(np.asarray(bs_xyz, float) - np.asarray(ris_xyz, float)))
    ris_ue = float(np.linalg.norm(np.asarray(ris_xyz, float) - np.asarray(ue_xyz, float)))
    limit = float(fraunhofer_distance_m)
    return FarFieldAudit(bs_ris, ris_ue, limit, bs_ris >= limit, ris_ue >= limit)


def require_far_field(audit: FarFieldAudit) -> None:
    if not audit.valid:
        raise ValueError(
            "RIS v1 far-field model is invalid for this frame: "
            f"BS-RIS={audit.bs_ris_distance_m:.3f} m, "
            f"RIS-UE={audit.ris_ue_distance_m:.3f} m, "
            f"required >= {audit.fraunhofer_distance_m:.3f} m. "
            "Use a smaller panel/lower frequency, move the nodes, or implement "
            "a near-field spherical-wave model; the v1 code will not silently continue."
        )


def panel_coupling_audit(
    bs_xyz: np.ndarray,
    ris_xyz: np.ndarray,
    ue_xyz: np.ndarray,
    panel_normal: np.ndarray,
) -> PanelCouplingAudit:
    """Return passive front-side projected-aperture field coupling.

    The two Sionna component links already include unit-gain element Friis
    effective aperture and both propagation losses.  This factor only applies
    the missing projected-area loss of the planar passive element.  Its square
    is the product of incidence and departure power cosines, bounded in [0,1].
    A node on the back side of the configured panel makes the
    explicit RIS contribution zero.
    """
    center = np.asarray(ris_xyz, dtype=np.float64).reshape(3)
    normal = np.asarray(panel_normal, dtype=np.float64).reshape(3)
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= 0.0:
        raise ValueError("RIS panel normal must be non-zero")
    normal /= normal_norm

    def cosine_toward(point):
        direction = np.asarray(point, dtype=np.float64).reshape(3) - center
        norm = float(np.linalg.norm(direction))
        if norm <= 0.0:
            return 0.0
        return float(np.dot(normal, direction / norm))

    incidence = cosine_toward(bs_xyz)
    departure = cosine_toward(ue_xyz)
    valid = incidence > 0.0 and departure > 0.0
    factor = float(np.sqrt(incidence * departure)) if valid else 0.0
    return PanelCouplingAudit(incidence, departure, factor, valid)


def validate_component_shapes(
    h_br: np.ndarray, h_ru: np.ndarray, h_direct: np.ndarray | None = None
) -> tuple[int, int, int, int]:
    """Validate [K,Nris,Nbs], [K,Nue,Nris], [K,Nue,Nbs]."""
    h_br = np.asarray(h_br)
    h_ru = np.asarray(h_ru)
    if h_br.ndim != 3 or h_ru.ndim != 3:
        raise ValueError(f"H_BR and H_RU must be rank-3; got {h_br.shape}, {h_ru.shape}")
    k, n_ris, n_bs = h_br.shape
    k2, n_ue, n_ris2 = h_ru.shape
    if k != k2 or n_ris != n_ris2:
        raise ValueError(
            f"RIS component mismatch: H_BR={h_br.shape}, H_RU={h_ru.shape}"
        )
    if h_direct is not None and tuple(np.asarray(h_direct).shape) != (k, n_ue, n_bs):
        raise ValueError(
            f"H_BU must have shape {(k, n_ue, n_bs)}; got {np.asarray(h_direct).shape}"
        )
    return k, n_ris, n_bs, n_ue


def phase_vector(
    h_br: np.ndarray,
    h_ru: np.ndarray,
    *,
    reflection_amplitude: float,
    phase_mode: str,
) -> np.ndarray:
    """Return diagonal entries of Phi with shape [K,N_RIS].

    ``focus`` aligns the reference BS port 0 and UE port 0 independently per
    subcarrier. Stored metadata identifies this reference-port objective.
    """
    k, n_ris, _n_bs, _n_ue = validate_component_shapes(h_br, h_ru)
    rho = float(reflection_amplitude)
    if not (0.0 <= rho <= 1.0):
        raise ValueError("reflection_amplitude must be in [0,1]")
    mode = str(phase_mode).strip().lower()
    if mode == "zero":
        return np.full((k, n_ris), np.complex64(rho + 0j), dtype=np.complex64)
    if mode == "focus":
        product = np.asarray(h_ru)[:, 0, :] * np.asarray(h_br)[:, :, 0]
        phase = np.where(np.abs(product) > 0.0, -np.angle(product), 0.0)
        return (rho * np.exp(1j * phase)).astype(np.complex64)
    raise ValueError(f"unsupported RIS phase_mode={phase_mode!r}")


def cascade_channel(
    h_br: np.ndarray,
    h_ru: np.ndarray,
    phi: np.ndarray,
) -> np.ndarray:
    """Compute H_RU Phi H_BR for every subcarrier."""
    k, n_ris, _n_bs, _n_ue = validate_component_shapes(h_br, h_ru)
    phi = np.asarray(phi)
    if phi.shape == (n_ris,):
        phi = np.broadcast_to(phi[None, :], (k, n_ris))
    if phi.shape != (k, n_ris):
        raise ValueError(f"phi must have shape {(k, n_ris)} or {(n_ris,)}, got {phi.shape}")
    return np.einsum(
        "kun,kn,knb->kub",
        np.asarray(h_ru, dtype=np.complex128),
        np.asarray(phi, dtype=np.complex128),
        np.asarray(h_br, dtype=np.complex128),
        optimize=True,
    ).astype(np.complex64)


def effective_channel(
    h_direct: np.ndarray,
    h_br: np.ndarray,
    h_ru: np.ndarray,
    phi: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    validate_component_shapes(h_br, h_ru, h_direct)
    cascaded = cascade_channel(h_br, h_ru, phi)
    return (np.asarray(h_direct, dtype=np.complex64) + cascaded).astype(np.complex64), cascaded


def synthesize_ris(
    h_direct: np.ndarray,
    h_br: np.ndarray,
    h_ru: np.ndarray,
    channel_cfg: Mapping[str, Any],
) -> dict[str, np.ndarray]:
    phi = phase_vector(
        h_br,
        h_ru,
        reflection_amplitude=float(channel_cfg.get("reflection_amplitude", 1.0)),
        phase_mode=str(channel_cfg.get("phase_mode", "zero")),
    )
    h_eff, h_cascade = effective_channel(h_direct, h_br, h_ru, phi)
    return {"phi": phi, "h_cascade": h_cascade, "h_effective": h_eff}
