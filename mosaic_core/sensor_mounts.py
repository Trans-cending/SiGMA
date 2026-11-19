"""CARLA-free fixed-node mount composition (X forward, Y right, Z up)."""
from copy import deepcopy
import math
import numpy as np

from .ris import carla_rotation_matrix


def fixed_sensor_world_config(item, center, rotation):
    """Resolve a panel-local pose directly to a world pose."""
    result = deepcopy(item)
    mount = result.get('transform') or {}
    loc = np.asarray(mount.get('location', [0, 0, 0]), dtype=float)
    angles = np.asarray(mount.get('rotation', [0, 0, 0]), dtype=float)
    origin = np.asarray(center, dtype=float)
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in (loc, angles, origin)):
        raise ValueError('fixed sensor mount requires finite 3-vector location/rotation/center')
    parent = carla_rotation_matrix(rotation)
    matrix = parent @ carla_rotation_matrix(angles)
    pitch = math.asin(float(np.clip(matrix[2, 0], -1, 1)))
    if abs(math.cos(pitch)) > 1e-8:
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        roll = math.atan2(-matrix[2, 1], matrix[2, 2])
    else:
        yaw = math.atan2(-matrix[0, 1], matrix[1, 1])
        roll = 0.0
    result['_mount_reference'] = 'ris_panel_center_local'
    result['_mount_transform_local'] = deepcopy(mount)
    result['transform'] = {'location': (origin + parent @ loc).tolist(),
                           'rotation': list(map(math.degrees, (pitch, yaw, roll)))}
    return result
