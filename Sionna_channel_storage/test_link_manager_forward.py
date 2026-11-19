# -*- coding: utf-8 -*-
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]

from Sionna_channel_storage.link_manager import (
    load_link_group_pose_from_stage1,
    load_link_pose_from_stage1,
)
from Sionna_channel_storage.pose_loader import carla_rot_to_sionna_forward


def _write_world_state(traj_dir: Path) -> None:
    ws_dir = traj_dir / "world_state"
    ws_dir.mkdir(parents=True)
    payload = {
        "nodes": {
            "rsu": [
                {
                    "node_id": "rsu_001",
                    "node_type": "rsu",
                    "index": 0,
                    "location": [0.0, 0.0, 1.5],
                    "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
                    "face_yaw": 0.0,
                }
            ],
            "cav": [
                {
                    "node_id": "cav_001",
                    "node_type": "cav",
                    "index": 0,
                    "actor_id": "101",
                    "location": [20.0, 0.0, 0.0],
                    "rotation": {"pitch": 0.0, "yaw": 90.0, "roll": 0.0},
                }
            ],
            "uav": [
                {
                    "node_id": "uav_001",
                    "node_type": "uav",
                    "index": 0,
                    "location": [20.0, 20.0, 10.0],
                    "rotation": {"pitch": 0.0, "yaw": 180.0, "roll": 0.0},
                    "carla_entity": {"bbox_center": [0.0, 0.0, 0.0]},
                }
            ],
        },
        "scene": {
            "vehicles": {
                "101": {
                    "center": [0.0, 0.0, 0.5],
                    "extent": [2.0, 1.0, 0.5],
                }
            }
        },
    }
    with open(ws_dir / "world_state_000000.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f)


def test_mobile_rx_forward_uses_stage1_heading_not_peer_direction(tmp_path: Path) -> None:
    traj_dir = tmp_path / "Magic_Test" / "stage1_multimodal" / "000000"
    _write_world_state(traj_dir)

    pose = load_link_pose_from_stage1(
        traj_dir,
        "dynamic_000000",
        {"link_id": "r001_c001", "tx": "r001", "rx": "c001"},
    )

    expected_cav_forward = carla_rot_to_sionna_forward({"pitch": 0.0, "yaw": 90.0, "roll": 0.0})
    peer_direction = pose["tx_pos"] - pose["rx_pos"]
    peer_direction = peer_direction / np.linalg.norm(peer_direction)

    assert np.allclose(pose["rx_forward"], expected_cav_forward.astype(np.float32), atol=1e-6)
    assert not np.allclose(pose["rx_forward"], peer_direction.astype(np.float32), atol=1e-6)


def test_group_pose_keeps_mobile_tx_and_rx_stage1_headings(tmp_path: Path) -> None:
    traj_dir = tmp_path / "Magic_Test" / "stage1_multimodal" / "000000"
    _write_world_state(traj_dir)

    pose = load_link_group_pose_from_stage1(
        traj_dir,
        "dynamic_000000",
        [
            {"link_id": "r001_c001", "tx": "r001", "rx": "c001"},
            {"link_id": "c001_u001", "tx": "c001", "rx": "u001"},
        ],
    )

    tx_aliases = list(pose["tx_aliases"])
    rx_aliases = list(pose["rx_aliases"])
    cav_tx_idx = tx_aliases.index("c001")
    cav_rx_idx = rx_aliases.index("c001")
    uav_rx_idx = rx_aliases.index("u001")

    expected_cav_forward = carla_rot_to_sionna_forward({"pitch": 0.0, "yaw": 90.0, "roll": 0.0})
    expected_uav_forward = carla_rot_to_sionna_forward({"pitch": 0.0, "yaw": 180.0, "roll": 0.0})

    assert np.allclose(pose["tx_forward_all"][cav_tx_idx], expected_cav_forward.astype(np.float32), atol=1e-6)
    assert np.allclose(pose["rx_forward_all"][cav_rx_idx], expected_cav_forward.astype(np.float32), atol=1e-6)
    assert np.allclose(pose["rx_forward_all"][uav_rx_idx], expected_uav_forward.astype(np.float32), atol=1e-6)
