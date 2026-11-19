# -*- coding: utf-8 -*-
from __future__ import annotations
import os
from .sionna_config import *

from mosaic_core.dataset import frame_idx_from_name, stage1_world_state_dir

from .pose_loader import read_yaml, normalize_rotation, carla_to_sionna_xyz, carla_rot_to_sionna_forward
from .path_extractor import _empty_rx_path_data

def parse_trajectory_name(traj_dir: Path) -> tuple[str, int]:
    path = Path(traj_dir)
    name = path.name
    if re.fullmatch(r"\d{6}", name) is None:
        raise ValueError(
            f"Trajectory directory must use canonical six-digit name NNNNNN: {traj_dir}"
        )
    map_name = os.environ.get("DATASET_MAP_NAME", "").strip()
    if not map_name and path.parent.name == "stage1_multimodal":
        map_name = path.parent.parent.name
    if not map_name:
        raise ValueError(f"Cannot infer map for canonical trajectory directory: {traj_dir}")
    return map_name, int(name)


def stage1_world_state_path(traj_dir: Path, frame_name: str) -> Path:
    parse_trajectory_name(traj_dir)
    frame_idx = frame_idx_from_name(frame_name)
    candidate = stage1_world_state_dir(traj_dir) / f"world_state_{frame_idx:06d}.yaml"
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(f"Missing canonical Stage1 world_state YAML: {candidate}")

def _stage1_sorted_nodes(value) -> list[dict]:
    if isinstance(value, dict):
        items = list(value.values())
    elif isinstance(value, list):
        items = list(value)
    else:
        items = []
    items = [x for x in items if isinstance(x, dict)]
    return sorted(items, key=lambda x: (int(x.get("index", 999999)), str(x.get("node_id") or x.get("label") or "")))

def _stage1_node_alias(kind: str, prefix: str, item: dict) -> tuple[str, int]:
    node_id = str(item.get("node_id") or item.get("label") or "")
    m = re.match(rf"^{re.escape(kind)}_(\d+)$", node_id)
    if m:
        n = int(m.group(1))
    else:
        n = int(item.get("index", 0)) + 1
    return f"{prefix}{n:03d}", n

def _select_stage1_nodes(world_state: dict) -> tuple[dict, dict]:
    nodes = world_state.get("nodes") if isinstance(world_state.get("nodes"), dict) else {}
    selected = {}
    aliases = {}
    specs = [
        ("rsu", "r", RADIO_RSU_IDS),
        ("static_rx", "s", RADIO_STATIC_RX_IDS),
        ("cav", "c", RADIO_CAV_IDS),
        ("uav", "u", RADIO_UAV_IDS),
    ]
    for kind, prefix, selector in specs:
        for item in _stage1_sorted_nodes(nodes.get(kind)):
            alias, ordinal = _stage1_node_alias(kind, prefix, item)
            if ordinal < 1:
                continue
            if selector is not None and ordinal not in selector:
                continue
            # Preserve Stage1 numbering. If one node is unavailable in a frame,
            # retain the original aliases of later nodes.
            if item.get("available", True) is False:
                continue
            full = str(item.get("node_id") or item.get("label") or f"{kind}_{ordinal:03d}")
            selected[alias] = dict(item)
            aliases[alias] = full
    return selected, aliases

def _alias_for_node_token(token: str, node_aliases: dict) -> str:
    value = str(token or "").strip()
    if re.fullmatch(r"[rcus]\d{3}", value):
        if value not in node_aliases:
            raise KeyError(f"CHANNEL_LINKS references unavailable node alias {value!r}")
        return value
    aliases_by_full = {str(full): alias for alias, full in node_aliases.items()}
    if value in aliases_by_full:
        return aliases_by_full[value]
    m = re.fullmatch(r"(rsu|cav|uav|static_rx)_(\d{3})", value)
    if m:
        prefix = {"rsu": "r", "cav": "c", "uav": "u", "static_rx": "s"}[m.group(1)]
        alias = f"{prefix}{int(m.group(2)):03d}"
        if alias in node_aliases:
            return alias
    raise KeyError(f"CHANNEL_LINKS references unavailable node {value!r}; available={sorted(node_aliases.values())}")


def _link_type_for_aliases(tx: str, rx: str) -> str:
    pair = (str(tx)[0], str(rx)[0])
    mapping = {
        ("r", "c"): "rc",
        ("r", "u"): "ru",
        ("c", "u"): "cu",
        ("r", "s"): "rs",
    }
    if pair not in mapping:
        raise ValueError(f"Unsupported explicit radio link {tx}->{rx}")
    return mapping[pair]


def build_radio_links(node_aliases: dict) -> dict[str, list[dict]]:
    if CHANNEL_LINKS:
        explicit: dict[str, list[dict]] = {}
        for tx_token, rx_token in CHANNEL_LINKS:
            tx = _alias_for_node_token(tx_token, node_aliases)
            rx = _alias_for_node_token(rx_token, node_aliases)
            link_type = _link_type_for_aliases(tx, rx)
            explicit.setdefault(link_type, []).append({"link_id": f"{tx}_{rx}", "tx": tx, "rx": rx})
        return explicit

    rsus = sorted(a for a in node_aliases if a.startswith("r"))
    cavs = sorted(a for a in node_aliases if a.startswith("c"))
    uavs = sorted(a for a in node_aliases if a.startswith("u"))
    links = {"rc": [], "ru": [], "cu": []}
    for r in rsus:
        for c in cavs:
            links["rc"].append({"link_id": f"{r}_{c}", "tx": r, "rx": c})
        for u in uavs:
            links["ru"].append({"link_id": f"{r}_{u}", "tx": r, "rx": u})
    for c in cavs:
        for u in uavs:
            links["cu"].append({"link_id": f"{c}_{u}", "tx": c, "rx": u})
    return {k: v for k, v in links.items() if v}

def _bbox_top_z_for_node(node: dict, world_state: dict | None) -> float:
    """Bounding-box top Z relative to the actor origin.

    CAV world_state stores ``center`` and ``extent`` in CARLA actor-local
    coordinates, so top = center.z + extent.z. This is the canonical RF
    phase-center reference for positive/negative antenna offsets.
    """
    try:
        if world_state and str(node.get("node_type", "")).lower() == "cav":
            vehicles = ((world_state.get("scene") or {}).get("vehicles") or {})
            actor_id = node.get("actor_id")
            vinfo = vehicles.get(actor_id) or vehicles.get(str(actor_id))
            if isinstance(vinfo, dict):
                center = vinfo.get("center")
                extent = vinfo.get("extent")
                if (isinstance(center, (list, tuple)) and len(center) >= 3
                        and isinstance(extent, (list, tuple)) and len(extent) >= 3):
                    return float(center[2]) + float(extent[2])
    except Exception:
        pass
    return 0.0


def _uav_bbox_top_z_for_node(node: dict) -> float:
    """UAV bounding-box top Z relative to the actor/entity origin."""
    try:
        ent = node.get("carla_entity") or {}
        bc = ent.get("bbox_center")
        size = ent.get("bbox_size_m") or ent.get("bbox_size")
        center_z = float(bc[2]) if isinstance(bc, (list, tuple)) and len(bc) >= 3 else 0.0
        if isinstance(size, (list, tuple)) and len(size) >= 3:
            return center_z + 0.5 * float(size[2])
        extent = ent.get("bbox_extent")
        if isinstance(extent, (list, tuple)) and len(extent) >= 3:
            return center_z + float(extent[2])
        return center_z
    except Exception:
        return 0.0


def _node_pose(alias: str, node: dict, world_state: dict | None = None) -> tuple[np.ndarray, np.ndarray, dict]:
    raw_loc = node.get("location") or node.get("position")
    loc = node.get("antenna_location") or raw_loc
    if loc is None:
        raise KeyError(f"node {alias} has no location/antenna_location")
    rot = node.get("rotation") or node.get("rotation_carla") or {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
    rot_norm = normalize_rotation(rot)
    node_type = str(node.get("node_type", "")).lower()
    if node_type == "rsu":
        # RF antenna facing = the sign face direction. New Stage1 data writes
        # an explicit face_yaw. Keep the historical RSU fallback for old Stage1
        # files that stored only the marker actor rotation.
        rot_norm = dict(rot_norm)
        face_yaw = node.get("face_yaw")
        if face_yaw is not None:
            rot_norm["yaw"] = float(face_yaw)
        else:
            rot_norm["yaw"] = float(rot_norm["yaw"]) + 90.0
    elif node_type == "static_rx":
        # New static-RX Stage1 records mirror RSU/static-TX semantics and carry
        # face_yaw explicitly. Pre-patch static-RX records missing face_yaw
        # retain their recorded rotation.
        face_yaw = node.get("face_yaw")
        if face_yaw is not None:
            rot_norm = dict(rot_norm)
            rot_norm["yaw"] = float(face_yaw)

    pos = np.asarray(carla_to_sionna_xyz(loc), dtype=np.float32)
    node_type = str(node.get("node_type", "")).lower()
    # Stage-3 antenna placement overrides (env-driven; stage1 data untouched).
    if node_type == "cav" and RX_ANTENNA_HEIGHT_M is not None and raw_loc is not None:
        raw = np.asarray(carla_to_sionna_xyz(raw_loc), dtype=np.float32)
        bbox_z = _bbox_top_z_for_node(node, world_state)
        raw[2] = float(raw[2]) + float(bbox_z) + float(RX_ANTENNA_HEIGHT_M)
        pos = raw
    elif node_type == "uav" and UAV_ANTENNA_HEIGHT_M is not None and raw_loc is not None:
        raw = np.asarray(carla_to_sionna_xyz(raw_loc), dtype=np.float32)
        bbox_z = _uav_bbox_top_z_for_node(node)
        raw[2] = float(raw[2]) + float(bbox_z) + float(UAV_ANTENNA_HEIGHT_M)
        pos = raw

    forward = carla_rot_to_sionna_forward(rot_norm).astype(np.float32)
    return pos, forward, rot_norm


def _node_velocity(node: dict) -> np.ndarray:
    """Return the node velocity in the same Sionna world axes as positions."""
    value = node.get("velocity")
    if isinstance(value, (list, tuple, np.ndarray)) and len(value) >= 3:
        return np.asarray(carla_to_sionna_xyz(value[:3]), dtype=np.float32)
    return np.zeros(3, dtype=np.float32)



def _read_world_state_for_pose(traj_dir: Path, frame_name: str) -> dict | None:
    try:
        return read_yaml(stage1_world_state_path(traj_dir, frame_name))
    except Exception:
        return None


def load_link_pose_from_stage1(traj_dir: Path, frame_name: str, link: dict, selected_nodes: dict | None = None) -> dict:
    ws = _read_world_state_for_pose(traj_dir, frame_name)
    if selected_nodes is None:
        if ws is None:
            raise FileNotFoundError(f"Missing stage1 world_state for {traj_dir}/{frame_name}")
        selected_nodes, _aliases = _select_stage1_nodes(ws)
    tx_alias = str(link["tx"])
    rx_alias = str(link["rx"])
    tx_node = selected_nodes[tx_alias]
    rx_node = selected_nodes[rx_alias]
    tx_pos, tx_forward, tx_rot = _node_pose(tx_alias, tx_node, world_state=ws)
    rx_pos, rx_forward, rx_rot = _node_pose(rx_alias, rx_node, world_state=ws)
    return {
        "tx_alias": tx_alias,
        "rx_alias": rx_alias,
        "tx_pos": tx_pos,
        "rx_pos": rx_pos,
        "tx_forward": tx_forward,
        "rx_forward": rx_forward,
        "tx_velocity": _node_velocity(tx_node),
        "rx_velocity": _node_velocity(rx_node),
        "rx_pos_all": rx_pos.reshape(1, 3).astype(np.float32),
        "rx_forward_all": rx_forward.reshape(1, 3).astype(np.float32),
        "tx_rot": tx_rot,
        "rx_rot": [rx_rot],
        "tx_source": f"stage1.nodes.{tx_alias}",
        "rx_source": f"stage1.nodes.{rx_alias}",
        "num_users": 1,
    }

def load_link_group_pose_from_stage1(
    traj_dir: Path,
    frame_name: str,
    link_list: list[dict],
    selected_nodes: dict | None = None,
) -> dict:
    """Build M Tx + N Rx pose_info for a batch of requested links."""
    if not link_list:
        raise ValueError("link_list is empty")
    ws = _read_world_state_for_pose(traj_dir, frame_name)
    if selected_nodes is None:
        if ws is None:
            raise FileNotFoundError(f"Missing stage1 world_state for {traj_dir}/{frame_name}")
        selected_nodes, _aliases = _select_stage1_nodes(ws)

    tx_aliases = []
    rx_aliases = []
    for link in link_list:
        tx = str(link["tx"])
        rx = str(link["rx"])
        if tx not in tx_aliases:
            tx_aliases.append(tx)
        if rx not in rx_aliases:
            rx_aliases.append(rx)

    tx_poses = [_node_pose(tx_alias, selected_nodes[tx_alias], world_state=ws) for tx_alias in tx_aliases]
    rx_poses = [_node_pose(rx_alias, selected_nodes[rx_alias], world_state=ws) for rx_alias in rx_aliases]
    tx_pos_all = np.stack([p[0] for p in tx_poses], axis=0).astype(np.float32)
    tx_forward_all = np.stack([p[1] for p in tx_poses], axis=0).astype(np.float32)
    rx_pos_all = np.stack([p[0] for p in rx_poses], axis=0).astype(np.float32)
    rx_forward_all = np.stack([p[1] for p in rx_poses], axis=0).astype(np.float32)
    tx_rot = [p[2] for p in tx_poses]
    rx_rot = [p[2] for p in rx_poses]
    tx_velocity_all = np.stack(
        [_node_velocity(selected_nodes[alias]) for alias in tx_aliases], axis=0
    ).astype(np.float32)
    rx_velocity_all = np.stack(
        [_node_velocity(selected_nodes[alias]) for alias in rx_aliases], axis=0
    ).astype(np.float32)

    link_specs = []
    for i, link in enumerate(link_list):
        tx = str(link["tx"])
        rx = str(link["rx"])
        link_specs.append({
            "link_index": i,
            "link_id": str(link["link_id"]),
            "tx": tx,
            "rx": rx,
            "tx_idx": tx_aliases.index(tx),
            "rx_idx": rx_aliases.index(rx),
        })

    return {
        "tx_alias": tx_aliases[0],
        "tx_aliases": tx_aliases,
        "rx_alias": rx_aliases[0],
        "rx_aliases": rx_aliases,
        "tx_pos": tx_pos_all[0],
        "tx_pos_all": tx_pos_all,
        "rx_pos": rx_pos_all[0],
        "tx_forward": tx_forward_all[0],
        "tx_forward_all": tx_forward_all,
        "rx_forward": rx_forward_all[0],
        "rx_pos_all": rx_pos_all,
        "rx_forward_all": rx_forward_all,
        "tx_velocity": tx_velocity_all[0],
        "tx_velocity_all": tx_velocity_all,
        "rx_velocity": rx_velocity_all[0],
        "rx_velocity_all": rx_velocity_all,
        "tx_rot": tx_rot,
        "rx_rot": rx_rot,
        "tx_source": [f"stage1.nodes.{tx}" for tx in tx_aliases],
        "rx_source": [f"stage1.nodes.{rx}" for rx in rx_aliases],
        "num_tx": len(tx_aliases),
        "num_users": len(rx_aliases),
        "link_specs": link_specs,
    }

def single_link_pose_from_group(group_pose: dict, rx_idx: int, link: dict) -> dict:
    tx_pos_all = np.asarray(group_pose.get("tx_pos_all", group_pose["tx_pos"]), dtype=np.float32).reshape(-1, 3)
    tx_forward_all = np.asarray(group_pose.get("tx_forward_all", group_pose["tx_forward"]), dtype=np.float32).reshape(-1, 3)
    rx_pos_all = np.asarray(group_pose["rx_pos_all"], dtype=np.float32).reshape(-1, 3)
    rx_forward_all = np.asarray(group_pose["rx_forward_all"], dtype=np.float32).reshape(-1, 3)
    tx_velocity_all = np.asarray(
        group_pose.get("tx_velocity_all", np.zeros_like(tx_pos_all)), dtype=np.float32
    ).reshape(-1, 3)
    rx_velocity_all = np.asarray(
        group_pose.get("rx_velocity_all", np.zeros_like(rx_pos_all)), dtype=np.float32
    ).reshape(-1, 3)
    tx_aliases = list(group_pose.get("tx_aliases") or [group_pose.get("tx_alias")])
    try:
        tx_idx = tx_aliases.index(str(link["tx"]))
    except ValueError:
        tx_idx = 0
    rx_rot_all = list(group_pose.get("rx_rot", []))
    rx_rot = rx_rot_all[rx_idx] if rx_idx < len(rx_rot_all) else {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
    tx_rot_all = group_pose.get("tx_rot", {"pitch": 0.0, "yaw": 0.0, "roll": 0.0})
    tx_rot = tx_rot_all[tx_idx] if isinstance(tx_rot_all, list) and tx_idx < len(tx_rot_all) else tx_rot_all
    return {
        "tx_alias": str(link["tx"]),
        "rx_alias": str(link["rx"]),
        "tx_pos": tx_pos_all[tx_idx],
        "rx_pos": rx_pos_all[rx_idx],
        "tx_forward": tx_forward_all[tx_idx],
        "rx_forward": rx_forward_all[rx_idx],
        "tx_velocity": tx_velocity_all[tx_idx],
        "rx_velocity": rx_velocity_all[rx_idx],
        "rx_pos_all": rx_pos_all[rx_idx].reshape(1, 3).astype(np.float32),
        "rx_forward_all": rx_forward_all[rx_idx].reshape(1, 3).astype(np.float32),
        "tx_rot": tx_rot,
        "rx_rot": [rx_rot],
        "tx_source": f"stage1.nodes.{link['tx']}",
        "rx_source": f"stage1.nodes.{link['rx']}",
        "num_users": 1,
    }

def _single_paths_payload(rx: dict) -> dict:
    return {
        "theta_t": rx["theta_t"].astype(np.float32),
        "phi_t": rx["phi_t"].astype(np.float32),
        "glob_theta_t": rx["glob_theta_t"].astype(np.float32),
        "glob_phi_t": rx["glob_phi_t"].astype(np.float32),
        "rx_users": [rx],
        "num_users": 1,
    }

def paths_data_for_rx(paths_data: dict, rx_idx: int) -> dict:
    rx_users = list(paths_data.get("rx_users", []))
    rx = rx_users[rx_idx] if 0 <= rx_idx < len(rx_users) else _empty_rx_path_data()
    return _single_paths_payload(rx)

def paths_data_for_link(paths_data: dict, link_idx: int) -> dict:
    link_users = list(paths_data.get("link_users") or paths_data.get("rx_users", []))
    rx = link_users[link_idx] if 0 <= link_idx < len(link_users) else _empty_rx_path_data()
    return _single_paths_payload(rx)

def group_links_by_trace(links: dict) -> list[tuple[str, str, list[dict]]]:
    groups = []
    for link_type in sorted(links):
        link_list = list(links.get(link_type) or [])
        if link_list:
            groups.append((link_type, "all", link_list))
    return groups

def group_links_by_tx(links: dict) -> list[tuple[str, str, list[dict]]]:
    # Backward-compatible name; new implementation batches all links of one
    # link_type so CU links with multiple CAV transmitters trace once.
    return group_links_by_trace(links)

def configured_node_aliases() -> dict:
    aliases = {}
    # RADIO_*_IDS = all/0 means auto/all Stage1 nodes. A frame-specific
    # world_state supplies the concrete alias list; callers should
    # pass the actual node_aliases from _select_stage1_nodes().
    for i in RADIO_RSU_IDS or tuple():
        aliases[f"r{i:03d}"] = f"rsu_{i:03d}"
    for i in RADIO_CAV_IDS or tuple():
        aliases[f"c{i:03d}"] = f"cav_{i:03d}"
    for i in RADIO_UAV_IDS or tuple():
        aliases[f"u{i:03d}"] = f"uav_{i:03d}"
    for i in RADIO_STATIC_RX_IDS or tuple():
        aliases[f"s{i:03d}"] = f"static_rx_{i:03d}"
    return aliases


__all__=[k for k in globals() if not k.startswith("__")]
