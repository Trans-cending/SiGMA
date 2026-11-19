"""Save map-scoped CSI and paths with shared sensor-frame identity.

Run: python -m sigma stage3 -c configs/town03.yaml.
"""
# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from datetime import datetime
from .sionna_config import *

from .link_manager import parse_trajectory_name, configured_node_aliases, frame_idx_from_name

from mosaic_core.rf_materials import material_compatible_hashes, material_config_hash

RF_MATERIAL_CONFIG_HASH = material_config_hash()
RF_MATERIAL_COMPATIBLE_HASHES = material_compatible_hashes()

from .path_extractor import _empty_rx_path_data
from .link_manager import build_radio_links

def frame_identity(traj_dir, frame_idx):
    source = Path(traj_dir) / "world_state" / f"world_state_{int(frame_idx):06d}.yaml"
    state = yaml.safe_load(source.read_text())
    return {"site_id": np.asarray(str(state.get("site_id", DATASET_ROOT.name))),
            "trajectory_id": np.asarray(state["trajectory_id"]),
            "frame_idx": np.asarray(frame_idx),
            "world_frame": np.asarray(state["world_frame"]),
            "time_s": np.asarray(state["time_s"], dtype=np.float64)}


def stage3_dataset_root() -> Path:
    return DATASET_ROOT / STAGE3_SUBDIR

def radio_index_path() -> Path:
    return stage3_dataset_root() / "radio_index.yaml"

def stage3_root(traj_dir: Path) -> Path:
    return stage3_dataset_root() / Path(traj_dir).name


def frame_error_path(traj_dir: Path, frame_idx: int) -> Path:
    return stage3_root(traj_dir) / "_errors" / f"{int(frame_idx):06d}.json"


def record_frame_error(traj_dir: Path, frame_name: str, exc: BaseException) -> Path:
    """Persist one unresolved Stage3 frame error for CLI status reporting."""
    frame_idx = frame_idx_from_name(frame_name)
    out = frame_error_path(traj_dir, frame_idx)
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "stage": "channel",
        "trajectory": Path(traj_dir).name,
        "frame": int(frame_idx),
        "frame_name": str(frame_name),
        "error_type": type(exc).__name__,
        "message": str(exc),
        "timestamp": datetime.now().astimezone().isoformat(),
    }
    tmp = out.with_suffix(out.suffix + f".tmp.{os.getpid()}")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, out)
    return out


def clear_frame_error(traj_dir: Path, frame_name: str) -> None:
    """Clear a prior unresolved error once the frame succeeds on retry."""
    try:
        frame_error_path(traj_dir, frame_idx_from_name(frame_name)).unlink(missing_ok=True)
    except OSError:
        pass

def path_npz_path(traj_dir: Path, link_type: str, link_id: str, frame_idx: int) -> Path:
    return stage3_root(traj_dir) / "paths" / link_type / link_id / f"p_{int(frame_idx):06d}.npz"

def channel_npz_path(traj_dir: Path, config_id: str, link_type: str, link_id: str, frame_idx: int) -> Path:
    return stage3_root(traj_dir) / "channels" / config_id / link_type / link_id / f"h_{int(frame_idx):06d}.npz"

def paths_to_npz_arrays(paths_data: dict) -> dict:
    rx = paths_data.get("rx_users", [_empty_rx_path_data()])[0]
    theta_t = np.asarray(rx.get("theta_t", np.zeros((0,), dtype=np.float32)), dtype=np.float32)
    phi_t = np.asarray(rx.get("phi_t", np.zeros((0,), dtype=np.float32)), dtype=np.float32)
    theta_r = np.asarray(rx.get("theta_r", np.zeros((0,), dtype=np.float32)), dtype=np.float32)
    phi_r = np.asarray(rx.get("phi_r", np.zeros((0,), dtype=np.float32)), dtype=np.float32)
    n = int(np.asarray(rx.get("a", [])).shape[0])
    path_type = np.asarray(rx.get("path_type", np.zeros((n,), dtype=np.uint8)), dtype=np.uint8)
    los = np.asarray(rx.get("los", np.zeros((n,), dtype=bool)), dtype=bool)
    if path_type.shape[0] != n:
        path_type = np.zeros((n,), dtype=np.uint8)
    if los.shape[0] != n:
        los = np.zeros((n,), dtype=bool)
    aod = np.stack(
        [phi_t, (np.pi / 2.0 - theta_t).astype(np.float32)], axis=-1
    ).astype(np.float32).reshape(n, 2)
    aoa = np.stack(
        [phi_r, (np.pi / 2.0 - theta_r).astype(np.float32)], axis=-1
    ).astype(np.float32).reshape(n, 2)
    # Preserve every valid propagation path. These booleans only expose the
    # same Array-LCS +Z front/back classification used later by channel IF
    # filtering while preserving every stored ray and its weight.
    tx_front = (np.cos(theta_t.astype(np.float64)) >= -1e-7).astype(bool)
    rx_front = (np.cos(theta_r.astype(np.float64)) >= -1e-7).astype(bool)
    interaction_types = np.asarray(rx.get("interaction_types", np.zeros((n, 0), dtype=np.uint8)), dtype=np.uint8)
    object_ids = np.asarray(rx.get("object_ids", np.zeros((n, 0), dtype=np.uint32)), dtype=np.uint32)
    primitive_ids = np.asarray(rx.get("primitive_ids", np.zeros((n, 0), dtype=np.uint32)), dtype=np.uint32)
    interaction_vertices = np.asarray(rx.get("interaction_vertices_m", np.zeros((n, 0, 3), dtype=np.float32)), dtype=np.float32)
    if interaction_types.ndim != 2 or interaction_types.shape[0] != n:
        interaction_types = np.zeros((n, 0), dtype=np.uint8)
    depth = int(interaction_types.shape[1])
    if object_ids.shape != (n, depth):
        object_ids = np.full((n, depth), np.uint32(0xFFFFFFFF), dtype=np.uint32)
    if primitive_ids.shape != (n, depth):
        primitive_ids = np.full((n, depth), np.uint32(0xFFFFFFFF), dtype=np.uint32)
    if interaction_vertices.shape != (n, depth, 3):
        interaction_vertices = np.zeros((n, depth, 3), dtype=np.float32)
    dedup_keep = np.asarray(rx.get("dedup_keep", np.ones((n,), dtype=bool)), dtype=bool).reshape(n)
    duplicate_of = np.asarray(rx.get("duplicate_of", np.full((n,), -1, dtype=np.int32)), dtype=np.int32).reshape(n)
    duplicate_group_size = np.asarray(rx.get("duplicate_group_size", np.ones((n,), dtype=np.uint16)), dtype=np.uint16).reshape(n)
    dedup_reason = np.asarray(rx.get("dedup_reason", np.zeros((n,), dtype=np.uint8)), dtype=np.uint8).reshape(n)
    dedup_medoid_score = np.asarray(rx.get("dedup_medoid_score", np.zeros((n,), dtype=np.float32)), dtype=np.float32).reshape(n)
    return {
        "coefficient": np.asarray(rx.get("a", np.zeros((0,), dtype=np.complex64)), dtype=np.complex64),
        "delay_s": np.asarray(rx.get("tau", np.zeros((0,), dtype=np.float32)), dtype=np.float32),
        "doppler_hz": np.asarray(
            rx.get("doppler_hz", np.zeros((n,), dtype=np.float32)),
            dtype=np.float32,
        ).reshape(n),
        "aod_rad": aod,
        "aoa_rad": aoa,
        "path_type": path_type,
        "los": los,
        "tx_front": tx_front.reshape(n),
        "rx_front": rx_front.reshape(n),
        # Retain raw Sionna interaction metadata and every ray for diagnostics;
        # dedup_keep marks the representatives used for H.
        "interaction_types": interaction_types,
        "object_ids": object_ids,
        "primitive_ids": primitive_ids,
        "interaction_vertices_m": interaction_vertices,
        "dedup_keep": dedup_keep,
        "dedup_reason": dedup_reason,
        "dedup_medoid_score": dedup_medoid_score,
        "duplicate_of": duplicate_of,
        "duplicate_group_size": duplicate_group_size,
        "path_dedup_schema_version": np.asarray(PATH_DEDUP_SCHEMA_VERSION, dtype=np.int16),
        "material_config_hash": np.asarray(RF_MATERIAL_CONFIG_HASH),
    }

def _validate_stage1_radar_material_hash(traj_dir: Path, frame_idx: int) -> None:
    radar_root = Path(traj_dir) / "nodes"
    if not radar_root.is_dir():
        return
    suffix = f"_{int(frame_idx):06d}.npz"
    for radar_path in radar_root.glob("*/radar/*.npz"):
        if not radar_path.name.endswith(suffix):
            continue
        with np.load(radar_path) as radar_data:
            if "material_config_hash" not in radar_data:
                continue
            radar_hash = str(np.asarray(radar_data["material_config_hash"]).reshape(()))
        if radar_hash not in RF_MATERIAL_COMPATIBLE_HASHES:
            raise RuntimeError(
                "Radar/Sionna RF material registry mismatch: "
                f"trajectory={Path(traj_dir).name}, frame={frame_idx}, "
                f"radar={radar_hash}, sionna={RF_MATERIAL_CONFIG_HASH}, "
                f"compatible={sorted(RF_MATERIAL_COMPATIBLE_HASHES)}, "
                f"file={radar_path}"
            )

def save_path_frame(traj_dir: Path, frame_idx: int, link_type: str, link: dict, paths_data: dict):
    _validate_stage1_radar_material_hash(traj_dir, frame_idx)
    out = path_npz_path(traj_dir, link_type, link["link_id"], frame_idx)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **paths_to_npz_arrays(paths_data))

def load_path_frame(traj_dir: Path, frame_idx: int, link_type: str, link: dict) -> tuple[dict, dict]:
    p = path_npz_path(traj_dir, link_type, link["link_id"], frame_idx)
    with np.load(p) as data:
        coeff = data["coefficient"].astype(np.complex64)
        tau = data["delay_s"].astype(np.float32)
        aod = data["aod_rad"].astype(np.float32)
        aoa = data["aoa_rad"].astype(np.float32)
        n = int(coeff.shape[0])
        path_type = data["path_type"].astype(np.uint8) if "path_type" in data else np.zeros((n,), dtype=np.uint8)
        los = data["los"].astype(bool) if "los" in data else (path_type == 0)
        dedup_keep = data["dedup_keep"].astype(bool) if "dedup_keep" in data else np.ones((n,), dtype=bool)
        duplicate_of = data["duplicate_of"].astype(np.int32) if "duplicate_of" in data else np.full((n,), -1, dtype=np.int32)
        duplicate_group_size = data["duplicate_group_size"].astype(np.uint16) if "duplicate_group_size" in data else np.ones((n,), dtype=np.uint16)
        dedup_reason = data["dedup_reason"].astype(np.uint8) if "dedup_reason" in data else np.zeros((n,), dtype=np.uint8)
        dedup_medoid_score = data["dedup_medoid_score"].astype(np.float32) if "dedup_medoid_score" in data else np.zeros((n,), dtype=np.float32)
        interaction_types = data["interaction_types"].astype(np.uint8) if "interaction_types" in data else np.zeros((n, 0), dtype=np.uint8)
        object_ids = data["object_ids"].astype(np.uint32) if "object_ids" in data else np.zeros((n, 0), dtype=np.uint32)
        primitive_ids = data["primitive_ids"].astype(np.uint32) if "primitive_ids" in data else np.zeros((n, 0), dtype=np.uint32)
        interaction_vertices = data["interaction_vertices_m"].astype(np.float32) if "interaction_vertices_m" in data else np.zeros((n, 0, 3), dtype=np.float32)
    rx = {
        "a": coeff,
        "tau": tau,
        "theta_t": (np.pi / 2.0 - aod[:, 1]).astype(np.float32),
        "phi_t": aod[:, 0].astype(np.float32),
        "theta_r": (np.pi / 2.0 - aoa[:, 1]).astype(np.float32),
        "phi_r": aoa[:, 0].astype(np.float32),
        "glob_theta_t": (np.pi / 2.0 - aod[:, 1]).astype(np.float32),
        "glob_phi_t": aod[:, 0].astype(np.float32),
        "glob_theta_r": (np.pi / 2.0 - aoa[:, 1]).astype(np.float32),
        "glob_phi_r": aoa[:, 0].astype(np.float32),
        "first_interaction_pos": np.full((coeff.shape[0], 3), np.nan, dtype=np.float32),
        "last_interaction_pos": np.full((coeff.shape[0], 3), np.nan, dtype=np.float32),
        "interaction_types": interaction_types,
        "object_ids": object_ids,
        "primitive_ids": primitive_ids,
        "dedup_reason": dedup_reason,
        "dedup_medoid_score": dedup_medoid_score,
        "interaction_vertices_m": interaction_vertices,
        "dedup_keep": dedup_keep,
        "duplicate_of": duplicate_of,
        "duplicate_group_size": duplicate_group_size,
        "path_type": path_type,
        "los": los,
    }
    paths_data = {"theta_t": rx["theta_t"], "phi_t": rx["phi_t"], "glob_theta_t": rx["glob_theta_t"], "glob_phi_t": rx["glob_phi_t"], "rx_users": [rx], "num_users": 1}
    return paths_data, {}

def path_frame_complete(traj_dir: Path, frame_idx: int, link_type: str, link: dict) -> bool:
    p = path_npz_path(traj_dir, link_type, link["link_id"], frame_idx)
    if not p.is_file() or p.stat().st_size <= 0:
        return False
    try:
        with np.load(p) as data:
            required = [
                "coefficient", "delay_s", "doppler_hz", "aod_rad", "aoa_rad",
                "path_type", "los", "material_config_hash",
            ]
            if PATH_GEOMETRY_DEDUP_ENABLED:
                required += [
                    "interaction_types", "object_ids", "primitive_ids",
                    "interaction_vertices_m", "dedup_keep", "duplicate_of",
                    "duplicate_group_size", "dedup_reason", "dedup_medoid_score",
                    "path_dedup_schema_version",
                ]
            if not all(k in data for k in required):
                return False
            if PATH_GEOMETRY_DEDUP_ENABLED:
                version = int(np.asarray(data["path_dedup_schema_version"]).reshape(()))
                if version != int(PATH_DEDUP_SCHEMA_VERSION):
                    return False
            stored_hash = str(np.asarray(data["material_config_hash"]).reshape(()))
            return stored_hash == RF_MATERIAL_CONFIG_HASH
    except Exception:
        return False

def channel_frame_complete(traj_dir: Path, frame_idx: int, cfg: dict, link_type: str, link: dict) -> bool:
    p = channel_npz_path(traj_dir, cfg["id"], link_type, link["link_id"], frame_idx)
    if not p.is_file() or p.stat().st_size <= 0:
        return False
    try:
        with np.load(p) as data:
            if "h" not in data or "material_config_hash" not in data:
                return False
            h = data["h"]
            expected = (int(cfg["num_subcarriers"]), int(cfg["rx_rows"] * cfg["rx_cols"]), int(cfg["tx_rows"] * cfg["tx_cols"]))
            if tuple(h.shape) != expected:
                return False
            if PATH_GEOMETRY_DEDUP_ENABLED:
                if "path_dedup_schema_version" not in data:
                    return False
                version = int(np.asarray(data["path_dedup_schema_version"]).reshape(()))
                if version != int(PATH_DEDUP_SCHEMA_VERSION):
                    return False
            stored_hash = str(np.asarray(data["material_config_hash"]).reshape(()))
            return stored_hash == RF_MATERIAL_CONFIG_HASH
    except Exception:
        return False

def all_outputs_complete(traj_dir: Path, frame_idx: int, links: dict) -> bool:
    for link_type, link_list in links.items():
        for link in link_list:
            if not path_frame_complete(traj_dir, frame_idx, link_type, link):
                return False
            for cfg in ARRAY_CONFIG_LIST:
                if link_type in cfg.get("enabled_link_types", []):
                    if not channel_frame_complete(traj_dir, frame_idx, cfg, link_type, link):
                        return False
    return True

def save_channel_frame(traj_dir: Path, frame_idx: int, cfg: dict, link_type: str, link: dict, h_freq: np.ndarray):
    out = channel_npz_path(traj_dir, cfg["id"], link_type, link["link_id"], frame_idx)
    out.parent.mkdir(parents=True, exist_ok=True)
    # Stored axis order: [subcarrier, rx_antenna, tx_antenna]
    h = h_freq[0].transpose(2, 0, 1).astype(np.complex64)
    np.savez_compressed(
        out,
        h=h,
        reconstruction=np.asarray(CHANNEL_RECONSTRUCTION_MODE),
        **frame_identity(traj_dir, frame_idx),
        path_dedup_schema_version=np.asarray(PATH_DEDUP_SCHEMA_VERSION, dtype=np.int16),
        material_config_hash=np.asarray(RF_MATERIAL_CONFIG_HASH),
    )


def load_channel_frame(traj_dir: Path, frame_idx: int, cfg: dict, link_type: str, link: dict) -> np.ndarray:
    """Load stored [K,R,T] data into the in-memory [1,R,T,K] convention."""
    path = channel_npz_path(traj_dir, cfg["id"], link_type, link["link_id"], frame_idx)
    with np.load(path, allow_pickle=False) as data:
        h = np.asarray(data["h"], dtype=np.complex64)
    return h.transpose(1, 2, 0)[None, ...]

def _array_index_cfg(cfg: dict, side: str) -> dict:
    return {
        "rows": int(cfg[f"{side}_rows"]),
        "columns": int(cfg[f"{side}_cols"]),
        "polarization": str(cfg.get("polarization", POLARIZATION)),
        "element_spacing_lambda": list(cfg.get("element_spacing_lambda", [ARRAY_SPACING_WL, ARRAY_SPACING_WL])),
    }

def build_radio_index_payload(node_aliases: dict | None = None, links: dict | None = None) -> dict:
    node_aliases = dict(node_aliases or configured_node_aliases())
    links = links or build_radio_links(node_aliases)
    channel_configs = {}
    for cfg in ARRAY_CONFIG_LIST:
        cid = cfg["id"]
        channel_configs[cid] = {
            "enabled_link_types": list(cfg.get("enabled_link_types", [])),
            "path_config": str(cfg.get("path_config", "p001")),
            "center_frequency_hz": int(float(cfg.get("center_frequency_hz", FREQUENCY_HZ))),
            "bandwidth_hz": int(float(cfg.get("bandwidth_hz", BANDWIDTH_HZ))),
            "num_subcarriers": int(cfg.get("num_subcarriers", N_SUBCARRIERS)),
            "tx_array": _array_index_cfg(cfg, "tx"),
            "rx_array": _array_index_cfg(cfg, "rx"),
        }
    return {
        "schema_version": "1.2",
        "material_config_hash": RF_MATERIAL_CONFIG_HASH,
        "dataset_layout": {
            "stage1_root": "stage1_multimodal",
            "stage2_root": "stage2_xml",
            "stage3_root": STAGE3_SUBDIR,
            "trajectory_dir_template": "{trajectory_id:06d}",
        },
        "reciprocity": {
            "enabled": True,
            "stored_link_types": [k for k in ["rc", "ru", "cu", "rs"] if k in links],
            "convention": "first_tx_second_rx",
        },
        "node_aliases": dict(sorted(node_aliases.items())),
        "links": links,
        "path_storage": {
            "format": "npz",
            "compression": True,
            "file_template": "{trajectory}/paths/{link_type}/{link_id}/p_{frame_idx:06d}.npz",
            "arrays": {
                "coefficient": {"dtype": "complex64", "shape": ["num_paths"]},
                "delay_s": {"dtype": "float32", "shape": ["num_paths"]},
                "doppler_hz": {"dtype": "float32", "shape": ["num_paths"]},
                "aod_rad": {"dtype": "float32", "shape": ["num_paths", 2], "order": ["azimuth", "elevation"]},
                "aoa_rad": {"dtype": "float32", "shape": ["num_paths", 2], "order": ["azimuth", "elevation"]},
                "path_type": {"dtype": "uint8", "shape": ["num_paths"]},
                "los": {"dtype": "bool", "shape": ["num_paths"]},
                "tx_front": {"dtype": "bool", "shape": ["num_paths"], "meaning": "Array-LCS +Z front hemisphere; raw path retained regardless"},
                "rx_front": {"dtype": "bool", "shape": ["num_paths"], "meaning": "Array-LCS +Z front hemisphere; raw path retained regardless"},
                "interaction_types": {"dtype": "uint8", "shape": ["num_paths", "max_depth"]},
                "object_ids": {"dtype": "uint32", "shape": ["num_paths", "max_depth"]},
                "primitive_ids": {"dtype": "uint32", "shape": ["num_paths", "max_depth"]},
                "interaction_vertices_m": {"dtype": "float32", "shape": ["num_paths", "max_depth", 3]},
                "dedup_keep": {"dtype": "bool", "shape": ["num_paths"], "meaning": "Representative mask used only for channel reconstruction; raw paths are retained"},
                "duplicate_of": {"dtype": "int32", "shape": ["num_paths"], "meaning": "Representative raw-path index for duplicates; -1 for retained or ineligible paths"},
                "duplicate_group_size": {"dtype": "uint16", "shape": ["num_paths"]},
                "dedup_reason": {"dtype": "uint8", "shape": ["num_paths"], "values": {"0": "retained_or_ineligible", "1": "duplicate_exact_specular_chain"}},
                "dedup_medoid_score": {"dtype": "float32", "shape": ["num_paths"], "meaning": "Dimensionless mean distance to the other candidates in the same exact-chain group; lower is more central"},
                "path_dedup_schema_version": {"dtype": "int16", "shape": [], "value": int(PATH_DEDUP_SCHEMA_VERSION)},
            },
        },
        "path_config": {
            "id": "p001",
            "solver": "sionna_rt",
            "max_depth": int(RT_MAX_DEPTH),
            "mechanisms": {
                "los": bool(RT_LOS),
                "specular_reflection": bool(RT_SPECULAR_REFLECTION),
                "diffuse_reflection": bool(RT_DIFFUSE_REFLECTION),
                "refraction": bool(RT_REFRACTION),
                "diffraction": bool(RT_DIFFRACTION),
                "edge_diffraction": bool(RT_EDGE_DIFFRACTION),
                "global_scattering_coefficient": float(GLOBAL_SCATTERING_COEFFICIENT),
                "effective_scattering_coefficient": float(EFFECTIVE_SCATTERING_COEFFICIENT),
            },
            "synthetic_array": bool(RT_SYNTHETIC_ARRAY),
            "array_if_filter": bool(ARRAY_IF_FILTER),
            "save_path_vertices": True,
            "geometry_dedup": {
                "enabled": bool(PATH_GEOMETRY_DEDUP_ENABLED),
                "schema_version": int(PATH_DEDUP_SCHEMA_VERSION),
                "raw_paths_retained": True,
                "eligible_mechanisms": ["specular_reflection"],
                "candidate_key": "ordered_interaction_type_object_id_primitive_id_chain",
                "identity_rule": "one_physical_solution_per_exact_specular_chain",
                "representative": "deterministic_numeric_medoid",
                "coefficient_domain": "passband",
                "vertex_distance_used": True,
                "representative_scales": {
                    "delay_s": float(PATH_DEDUP_DELAY_TOL_S),
                    "angle_rad": float(PATH_DEDUP_ANGLE_TOL_RAD),
                    "coefficient_magnitude_relative": float(PATH_DEDUP_COEFF_MAG_REL_TOL),
                    "coefficient_phase_rad": float(PATH_DEDUP_COEFF_PHASE_TOL_RAD),
                    "interaction_vertex_m": float(PATH_DEDUP_VERTEX_SCALE_M),
                },
            },
            "coordinate_system": "sionna_right_handed",
        },
        "channel_configs": channel_configs,
        "channel_storage": {
            "format": "npz",
            "path_dedup_schema_version": int(PATH_DEDUP_SCHEMA_VERSION),
            "compression": True,
            "file_template": "{trajectory}/channels/{config_id}/{link_type}/{link_id}/h_{frame_idx:06d}.npz",
            "array_key": "h",
            "dtype": "complex64",
            "axis_order": ["subcarrier", "rx_antenna", "tx_antenna"],
            "normalization": "none",
            "material_config_hash": RF_MATERIAL_CONFIG_HASH,
        },
    }

def write_radio_index(node_aliases: dict | None = None, links: dict | None = None):
    path = radio_index_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = build_radio_index_payload(node_aliases=node_aliases, links=links)
    if path.is_file():
        try:
            with open(path, "r", encoding="utf-8") as f:
                if yaml.safe_load(f) == payload:
                    return path
        except Exception:
            pass
    tmp = path.with_suffix(path.suffix + f".tmp_{os.getpid()}")
    with open(tmp, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False, allow_unicode=True)
    os.replace(tmp, path)
    return path



__all__=[k for k in globals() if not k.startswith("__")]
