# -*- coding: utf-8 -*-
"""Stage1 world_state helpers for Blender dynamic XML export.

Stage1 ``world_state`` YAML is the canonical source for CAV/UAV dynamic
actor geometry. World-state YAML is the canonical pose source.
"""

try:
    from .blender_common import *
except ImportError:
    from Blender_xml_storage.blender_common import *


def list3_from_location_dict_or_list(loc):
    if loc is None:
        return None
    if isinstance(loc, dict):
        return [
            float(loc.get("x", 0.0)),
            float(loc.get("y", 0.0)),
            float(loc.get("z", 0.0)),
        ]
    if isinstance(loc, (list, tuple)) and len(loc) >= 3:
        return [float(loc[0]), float(loc[1]), float(loc[2])]
    return None



def _rotation_list_pyr(value):
    """Return [pitch, yaw, roll] from common list/dict representations."""
    if isinstance(value, dict):
        return [
            float(value.get("pitch", 0.0)),
            float(value.get("yaw", 0.0)),
            float(value.get("roll", 0.0)),
        ]
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return [float(value[0]), float(value[1]), float(value[2])]
    return [0.0, 0.0, 0.0]



__all__ = [k for k in globals() if not k.startswith("__")]
