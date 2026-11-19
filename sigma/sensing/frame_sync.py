"""Strict frame-alignment helpers for Stage II CARLA datasets.

The CARLA-independent helpers support local tests of cadence and metadata
contracts.
"""
from __future__ import annotations

from typing import Any, Dict, Iterable, Mapping, Optional, Tuple


class FrameSyncError(RuntimeError):
    """Raised when sensor data and its CARLA world frame fail alignment checks."""


def strict_frame_sync_enabled(config: Optional[Mapping[str, Any]]) -> bool:
    return bool((config or {}).get("strict_frame_sync", True))


def sidecar_capture_stride(
    sample_every_n_frames: int,
    sidecar_sensor_sample_every_n: int,
) -> int:
    return max(1, int(sample_every_n_frames)) * max(
        1, int(sidecar_sensor_sample_every_n)
    )


def validate_world_state_cadence(
    world_state_every_n_frames: int,
    sample_every_n_frames: int,
    sidecar_sensor_sample_every_n: int,
) -> int:
    """Require every emitted world state to coincide with a sensor capture."""
    world_stride = max(1, int(world_state_every_n_frames))
    capture_stride = sidecar_capture_stride(
        sample_every_n_frames,
        sidecar_sensor_sample_every_n,
    )
    if world_stride % capture_stride:
        raise ValueError(
            "strict_frame_sync requires world_state_every_n_frames "
            f"({world_stride}) to be a multiple of the CARLA sidecar capture "
            f"stride ({capture_stride})"
        )
    return capture_stride


def capture_due(frame_idx: int, origin_frame_idx: Optional[int], stride: int) -> bool:
    """Use one post-gate origin for sensor files AND world_state output.

    File suffixes retain the trajectory frame_idx. Simulation ticks, sensor
    callbacks and FoV entry/termination retain their original cadence.
    """
    if origin_frame_idx is None:
        return False
    offset = int(frame_idx) - int(origin_frame_idx)
    return offset >= 0 and offset % max(1, int(stride)) == 0


def capture_matches_frame(
    capture: Optional[Mapping[str, Any]],
    frame_idx: int,
    world_frame: int,
) -> bool:
    if not isinstance(capture, Mapping):
        return False
    return (
        int(capture.get("frame_idx", -1)) == int(frame_idx)
        and int(capture.get("world_frame", -1)) == int(world_frame)
    )


def validate_sensor_payloads(
    groups: Iterable[Tuple[str, Mapping[str, Mapping[str, Any]]]],
    expected_world_frame: int,
) -> None:
    """Reject callback/save failures even when their frame numbers match.

    Known file-producing sensors must report a nonempty relative file path.
    New managers also supply file_written/file_size_bytes after a disk check.
    """
    expected = int(expected_world_frame)
    errors = []
    for group_name, sensors in groups:
        for sensor_name, payload in (sensors or {}).items():
            if not isinstance(payload, Mapping):
                errors.append(f"{group_name}/{sensor_name}: invalid payload")
                continue
            try:
                actual_world = int(payload.get("world_frame", -1))
                sensor_frame = int(payload.get("sensor_frame", -1))
            except (ValueError, TypeError, OverflowError):
                errors.append(f"{group_name}/{sensor_name}: invalid frame identifiers")
                continue
            if payload.get("error") or payload.get("available") is False:
                errors.append(
                    f"{group_name}/{sensor_name}: unavailable: "
                    f"{payload.get('error') or payload.get('reason') or 'unknown'}"
                )
            elif actual_world != expected or sensor_frame != expected:
                errors.append(
                    f"{group_name}/{sensor_name}: expected={expected}, "
                    f"world={actual_world}, sensor={sensor_frame}"
                )
            elif payload.get("frame_sync_ok") is not True:
                errors.append(f"{group_name}/{sensor_name}: frame_sync_ok is not true")
            elif payload.get("file_written") is False:
                errors.append(f"{group_name}/{sensor_name}: output file was not written")
            else:
                kind = str(payload.get("type", "")).lower()
                produces_file = kind in {
                    "rgb", "camera", "camera.rgb", "depth", "camera.depth",
                    "semantic", "semantic_segmentation", "camera.semantic_segmentation",
                    "gps", "gnss", "imu", "radar", "lidar", "lidar_2d", "lidar_3d",
                    "ray_cast", "lidar.ray_cast", "sensor.lidar.ray_cast",
                }
                if produces_file and not payload.get("file"):
                    errors.append(f"{group_name}/{sensor_name}: missing sensor file")
                if "file_size_bytes" in payload:
                    try:
                        if int(payload["file_size_bytes"]) <= 0:
                            errors.append(f"{group_name}/{sensor_name}: empty sensor file")
                    except (TypeError, ValueError, OverflowError):
                        errors.append(f"{group_name}/{sensor_name}: invalid file size")
    if errors:
        raise FrameSyncError("; ".join(errors))


def require_capture_for_world_state(
    capture: Optional[Mapping[str, Any]],
    frame_idx: int,
    world_frame: int,
) -> Dict[str, Any]:
    if not capture_matches_frame(capture, frame_idx, world_frame):
        raise FrameSyncError(
            "world_state has no exact-frame sensor capture: "
            f"frame_idx={int(frame_idx)}, world_frame={int(world_frame)}"
        )
    return dict(capture or {})
