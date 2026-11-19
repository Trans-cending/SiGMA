"""Stage III RIS component channels; nearfield uses physical array tracing.

Run: python -m sigma stage3 -c configs/town03.yaml.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from mosaic_core.coordinates import carla_to_sionna_xyz
from mosaic_core.ris import (
    estimate_component_storage_bytes,
    normalize_ris_config,
    panel_geometry,
    ris_config_hash,
)

from .channel_processor import synthesize_ofdm_channel
from .sionna_config import CHANNEL_RECONSTRUCTION_MODE
from .explicit_array import trace_explicit_channel
from .path_extractor import extract_paths_data
from .pose_loader import carla_rot_to_sionna_forward
from .radio_writer import paths_to_npz_arrays, stage3_root
from .raytracing import compute_paths, create_scene_for_ray_tracing
from .ris_channel import (
    audit_far_field, panel_coupling_audit, require_far_field, synthesize_ris,
)
from .ris_writer import (
    load_ris_paths,
    ris_component_path,
    ris_file_complete,
    ris_path_complete,
    ris_path_path,
    save_ris_channels,
    save_ris_paths,
)


def all_ris_outputs_complete(
    traj_dir: Path, frame_idx: int, links: dict, array_configs: list[dict], ris_cfg: dict
) -> bool:
    if not ris_cfg["enabled"] or ris_cfg["channel"]["mode"] == "off":
        return True
    config_hash = ris_config_hash(ris_cfg)
    mode = ris_cfg["channel"]["mode"]
    for link in links.get("rc", []):
        for cfg in array_configs:
            if "rc" not in cfg.get("enabled_link_types", []):
                continue
            path = ris_component_path(stage3_root(traj_dir), cfg["id"], link["link_id"], frame_idx)
            if not ris_file_complete(
                path, config_hash=config_hash, mode=mode,
                save_full_component_channels=bool(
                    ris_cfg["channel"]["save_full_component_channels"]),
            ):
                return False
    return True


def load_ris_stage3_config(traj_dir: Path, *, stage3_frequency_hz: float) -> tuple[dict, dict | None]:
    stage1_root = Path(traj_dir).parent
    sensor_config = stage1_root / "sensor_config.yaml"
    raw: dict[str, Any] = {}
    if sensor_config.is_file():
        raw = yaml.safe_load(sensor_config.read_text(encoding="utf-8")) or {}
    cfg = normalize_ris_config(raw.get("ris"))
    if not cfg["enabled"] or cfg["channel"]["mode"] == "off":
        return cfg, None
    if not np.isclose(float(cfg["frequency_hz"]), float(stage3_frequency_hz), rtol=0.0, atol=1.0):
        raise ValueError(
            "RIS frequency must match the Stage3 communication carrier for a shared "
            f"path/phase convention: ris={cfg['frequency_hz']}, stage3={stage3_frequency_hz}"
        )
    metadata_path = stage1_root / "ris_metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(
            f"RIS is enabled but Stage1 metadata is missing: {metadata_path}"
        )
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    return cfg, metadata


def _ris_node(metadata: dict, node_id: str) -> dict:
    nodes = dict(metadata.get("nodes") or {})
    if node_id not in nodes:
        raise KeyError(f"RIS metadata has no node {node_id!r}; available={sorted(nodes)}")
    return dict(nodes[node_id])


def _pose(tx_pos, tx_forward, rx_pos, rx_forward, tx_alias: str, rx_alias: str) -> dict:
    tx_pos = np.asarray(tx_pos, dtype=np.float32).reshape(3)
    rx_pos = np.asarray(rx_pos, dtype=np.float32).reshape(3)
    tx_forward = np.asarray(tx_forward, dtype=np.float32).reshape(3)
    rx_forward = np.asarray(rx_forward, dtype=np.float32).reshape(3)
    return {
        "tx_alias": tx_alias, "rx_alias": rx_alias,
        "tx_pos": tx_pos, "rx_pos": rx_pos,
        "tx_forward": tx_forward, "rx_forward": rx_forward,
        "tx_pos_all": tx_pos.reshape(1, 3),
        "tx_forward_all": tx_forward.reshape(1, 3),
        "rx_pos_all": rx_pos.reshape(1, 3),
        "rx_forward_all": rx_forward.reshape(1, 3),
        "num_tx": 1, "num_users": 1,
    }


def endpoint_receiver_doppler_hz(
    glob_theta_r: np.ndarray,
    glob_phi_r: np.ndarray,
    receiver_velocity_m_s: np.ndarray,
    frequency_hz: float,
) -> np.ndarray:
    """Return one-way endpoint Doppler using Sionna global AoA angles.

    Sionna RT 2.0.1 supplies global arrival angles for endpoint Doppler. The
    direction points from the receiver toward the preceding interaction, so
    motion toward that direction is positive Doppler.
    """
    theta = np.asarray(glob_theta_r, dtype=np.float64).reshape(-1)
    phi = np.asarray(glob_phi_r, dtype=np.float64).reshape(-1)
    if theta.shape != phi.shape:
        raise ValueError(f"arrival angle shape mismatch: {theta.shape} != {phi.shape}")
    if not theta.size:
        return np.zeros(0, dtype=np.float32)
    velocity = np.asarray(receiver_velocity_m_s, dtype=np.float64).reshape(3)
    arrival = np.column_stack((
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta),
    ))
    wavelength_m = 299_792_458.0 / float(frequency_hz)
    return (arrival @ velocity / wavelength_m).astype(np.float32)


def _trace_or_load_leg(
    *, safe_xml: Path, output_path: Path, pose: dict, config_hash: str,
    skip_existing: bool, save_path_parameters: bool, frequency_hz: float,
    receiver_velocity_m_s: np.ndarray,
) -> dict:
    if (save_path_parameters and skip_existing
            and ris_path_complete(output_path, config_hash=config_hash)):
        return load_ris_paths(output_path)
    scene, _camera, path_solver = create_scene_for_ray_tracing(safe_xml, pose)
    paths = compute_paths(scene, path_solver)
    data = extract_paths_data(paths, pose)
    rx = data["rx_users"][0]
    rx["doppler_hz"] = endpoint_receiver_doppler_hz(
        rx.get("glob_theta_r", []), rx.get("glob_phi_r", []),
        receiver_velocity_m_s, frequency_hz,
    )
    if save_path_parameters:
        save_ris_paths(output_path, paths_to_npz_arrays(data), config_hash=config_hash)
    return data


def _component_cfg(base_cfg: dict, *, rx_rows: int, rx_cols: int,
                   tx_rows: int, tx_cols: int, frequency_hz: float) -> dict:
    cfg = dict(base_cfg)
    cfg.update({
        "rx_rows": int(rx_rows), "rx_cols": int(rx_cols),
        "tx_rows": int(tx_rows), "tx_cols": int(tx_cols),
        "center_frequency_hz": float(frequency_hz),
    })
    return cfg


def process_ris_for_direct_link(
    *,
    traj_dir: Path,
    frame_idx: int,
    safe_xml: Path,
    link: dict,
    direct_pose: dict,
    direct_channel: np.ndarray,
    array_cfg: dict,
    ris_cfg: dict,
    ris_metadata: dict,
    skip_existing: bool,
) -> Path | None:
    """Generate one RSU->RIS->CAV product for an existing direct RC link."""
    if not ris_cfg["enabled"] or ris_cfg["channel"]["mode"] == "off":
        return None
    if not (str(link.get("tx", "")).startswith("r") and str(link.get("rx", "")).startswith("c")):
        return None
    node_id = str(ris_cfg["nodes"][0]["id"])
    node = _ris_node(ris_metadata, node_id)
    config_hash = ris_config_hash(ris_cfg)
    out = ris_component_path(stage3_root(traj_dir), array_cfg["id"], link["link_id"], frame_idx)
    if skip_existing and ris_file_complete(
        out, config_hash=config_hash, mode=ris_cfg["channel"]["mode"],
        save_full_component_channels=bool(
            ris_cfg["channel"]["save_full_component_channels"]),
    ):
        return out

    ris_carla = np.asarray(node["location_carla_m"], dtype=np.float64)
    ris_pos = carla_to_sionna_xyz(ris_carla, dtype=np.float32)
    ris_forward = carla_rot_to_sionna_forward(node["rotation_carla_deg"])
    geom = panel_geometry(ris_cfg)
    audit = audit_far_field(
        direct_pose["tx_pos"], ris_pos, direct_pose["rx_pos"], geom.fraunhofer_distance_m)
    if CHANNEL_RECONSTRUCTION_MODE == "farfield" and ris_cfg["channel"].get("require_far_field", True):
        require_far_field(audit)
    coupling = panel_coupling_audit(
        direct_pose["tx_pos"], ris_pos, direct_pose["rx_pos"], ris_forward)

    bytes_est = estimate_component_storage_bytes(
        num_subcarriers=int(array_cfg["num_subcarriers"]),
        num_ris_elements=geom.num_elements,
        num_bs=int(array_cfg["tx_rows"] * array_cfg["tx_cols"]),
        num_ue=int(array_cfg["rx_rows"] * array_cfg["rx_cols"]),
    )
    limit = float(ris_cfg["channel"]["max_storage_gib"]) * (1024**3)
    # H_BR and H_RU are required in memory, including configurations that skip
    # persistence. Apply the budget to both computation and storage, including
    # path-only output settings.
    if bytes_est > limit:
        raise MemoryError(
            f"RIS component channel memory estimate {bytes_est / 1024**3:.3f} GiB exceeds "
            f"ris.channel.max_storage_gib={limit / 1024**3:.3f} for one frame/link"
        )

    br_pose = _pose(
        direct_pose["tx_pos"], direct_pose["tx_forward"],
        ris_pos, ris_forward, str(link["tx"]), node_id)
    ru_pose = _pose(
        ris_pos, ris_forward, direct_pose["rx_pos"], direct_pose["rx_forward"],
        node_id, str(link["rx"]))
    br_path = ris_path_path(stage3_root(traj_dir), "rsu_to_ris", link["link_id"], frame_idx)
    ru_path = ris_path_path(stage3_root(traj_dir), "ris_to_cav", link["link_id"], frame_idx)
    br_data = _trace_or_load_leg(
        safe_xml=safe_xml, output_path=br_path, pose=br_pose,
        config_hash=config_hash, skip_existing=skip_existing,
        save_path_parameters=bool(ris_cfg["channel"]["save_path_parameters"]),
        frequency_hz=float(ris_cfg["frequency_hz"]),
        receiver_velocity_m_s=np.zeros(3, dtype=np.float32))
    ru_data = _trace_or_load_leg(
        safe_xml=safe_xml, output_path=ru_path, pose=ru_pose,
        config_hash=config_hash, skip_existing=skip_existing,
        save_path_parameters=bool(ris_cfg["channel"]["save_path_parameters"]),
        frequency_hz=float(ris_cfg["frequency_hz"]),
        receiver_velocity_m_s=np.asarray(
            direct_pose.get("rx_velocity", np.zeros(3)), dtype=np.float32))

    br_cfg = _component_cfg(
        array_cfg, rx_rows=geom.num_rows, rx_cols=geom.num_cols,
        tx_rows=array_cfg["tx_rows"], tx_cols=array_cfg["tx_cols"],
        frequency_hz=ris_cfg["frequency_hz"])
    ru_cfg = _component_cfg(
        array_cfg, rx_rows=array_cfg["rx_rows"], rx_cols=array_cfg["rx_cols"],
        tx_rows=geom.num_rows, tx_cols=geom.num_cols,
        frequency_hz=ris_cfg["frequency_hz"])
    if CHANNEL_RECONSTRUCTION_MODE == "nearfield":
        h_br = trace_explicit_channel(safe_xml, br_pose, br_cfg,
            br_path.parent / array_cfg["id"] / br_path.name)[0].transpose(2, 0, 1)
        h_ru = trace_explicit_channel(safe_xml, ru_pose, ru_cfg,
            ru_path.parent / array_cfg["id"] / ru_path.name)[0].transpose(2, 0, 1)
    else:
        h_br = synthesize_ofdm_channel(br_data, br_cfg, br_pose)[0].transpose(2, 0, 1)
        h_ru = synthesize_ofdm_channel(ru_data, ru_cfg, ru_pose)[0].transpose(2, 0, 1)
    h_direct = np.asarray(direct_channel[0].transpose(2, 0, 1), dtype=np.complex64)

    rx_br = br_data["rx_users"][0]
    rx_ru = ru_data["rx_users"][0]
    br_valid = bool(np.any(np.asarray(rx_br.get("dedup_keep", []), bool)))
    ru_valid = bool(np.any(np.asarray(rx_ru.get("dedup_keep", []), bool)))
    products = None
    if br_valid and ru_valid and coupling.front_side_valid:
        physical_channel_cfg = dict(ris_cfg["channel"])
        physical_channel_cfg["reflection_amplitude"] = (
            float(physical_channel_cfg["reflection_amplitude"])
            * coupling.field_amplitude_factor
        )
        products = synthesize_ris(h_direct, h_br, h_ru, physical_channel_cfg)
    else:
        # A missing component leg sets the RIS field to zero. Effective mode
        # then equals the direct channel.
        products = {
            "phi": np.zeros((h_direct.shape[0], geom.num_elements), np.complex64),
            "h_cascade": np.zeros_like(h_direct),
            "h_effective": h_direct.copy(),
        }

    mode = ris_cfg["channel"]["mode"]
    save_ris_channels(
        out,
        config_hash=config_hash,
        h_direct=h_direct,
        h_br=h_br,
        h_ru=h_ru,
        phi=products["phi"] if mode == "effective" else None,
        h_cascade=products["h_cascade"] if mode == "effective" else None,
        h_effective=products["h_effective"] if mode == "effective" else None,
        metadata={
            "node_id": node_id,
            "array_mode": CHANNEL_RECONSTRUCTION_MODE,
            "mode": mode,
            "frequency_hz": float(ris_cfg["frequency_hz"]),
            "subcarriers": int(array_cfg["num_subcarriers"]),
            "axis_order": {
                "h_br": ["subcarrier", "ris_element", "bs_antenna"],
                "h_ru": ["subcarrier", "ue_antenna", "ris_element"],
                "h_effective": ["subcarrier", "ue_antenna", "bs_antenna"],
                "phi": ["subcarrier", "ris_element"],
            },
            "phase_mode": ris_cfg["channel"]["phase_mode"],
            "reflection_amplitude": float(ris_cfg["channel"]["reflection_amplitude"]),
            "panel_projected_aperture_coupling": coupling.as_dict(),
            "unit_gain_element_effective_aperture_m2": float(
                geom.wavelength_m**2 / (4.0 * np.pi)
            ),
            "aperture_model": (
                "Sionna component-link Friis effective aperture per unit-gain element; "
                "projected-area field factor sqrt(cos_incidence*cos_departure)"
            ),
            "far_field": audit.as_dict(),
            "br_has_valid_path": br_valid,
            "ru_has_valid_path": ru_valid,
            "ordinary_rt_geometry": False,
            "path_parameters_saved": bool(
                ris_cfg["channel"]["save_path_parameters"]),
            "doppler_model": (
                "endpoint receiver velocity projected onto the Sionna global "
                "arrival direction; RSU and RIS are static"
            ),
            "component_storage_estimate_bytes": bytes_est,
        },
        save_full_component_channels=bool(
            ris_cfg["channel"]["save_full_component_channels"]),
    )
    return out
