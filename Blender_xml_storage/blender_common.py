"""Blender settings supplied explicitly by sigma.stage3.

Run: python -m sigma stage3 -c configs/town03.yaml.
"""

import bpy
import os
import sys
import re
import json
import bisect
import yaml
import math
import hashlib
import contextlib
from pathlib import Path
from mosaic_core.coordinates import carla_to_blender_location
from mosaic_core.naming import strip_blender_suffix

try:
    from .static_streetlight_filter import normalize_dtt_streetlight_mode
except ImportError:
    from Blender_xml_storage.static_streetlight_filter import normalize_dtt_streetlight_mode

# ============================================================
# 1. Global config
# ============================================================

FBX_ROOT = Path(os.environ.get(
    "SIONNA_FBX_ROOT",
    "assets/actors",
)).resolve()

XML_INPUT_VIEW = os.environ.get("XML_INPUT_VIEW", "auto")
OUTPUT_SUBDIR = os.environ.get("MITSUBA_OUTPUT_DIR", "mitsuba_out")

# Export mode:
#   full    : original behavior, export static Town + dynamic actors every frame
#   static  : export only the opened Town blend once to DATASET_ROOT/stage2_xml/static/TownXX.xml
#   dynamic : remove static Town objects, export only CAV/vehicles/trucks/pedestrians/RSU per frame
XML_EXPORT_MODE = os.environ.get("XML_EXPORT_MODE", "full").strip().lower()
if XML_EXPORT_MODE not in {"full", "static", "dynamic"}:
    print(f"WARNING: unknown XML_EXPORT_MODE={XML_EXPORT_MODE!r}; fallback to full", flush=True)
    XML_EXPORT_MODE = "full"

# For split export, static scenes are placed under DATASET_ROOT/stage2_xml/static/TownXX.xml.
MITSUBA_STATIC_DIR = os.environ.get("MITSUBA_STATIC_DIR", os.path.join("stage2_xml", "static"))

# Optional static-map FBX import. When set, static mode can start from an empty
# Blender session, import a UE-exported FBX map, canonicalize RF materials, save
# a reusable .blend, and then export the static Mitsuba XML.
XML_STATIC_FBX_PATH = os.environ.get("XML_STATIC_FBX_PATH", "").strip()
XML_STATIC_BLEND_OUTPUT = os.environ.get("XML_STATIC_BLEND_OUTPUT", "").strip()
XML_MAX_SKIP_BAD_MESHES = max(1, int(os.environ.get("XML_MAX_SKIP_BAD_MESHES", "20")))
# Static-scene cropping at export time.  When enabled (default), the exported
# static XML keeps only meshes whose world bbox intersects the TX/RX bounding
# XML_STATIC_BBOX_EXTEND meters on the four horizontal sides.
XML_STATIC_IF_CROP = os.environ.get("XML_STATIC_IF_CROP", "1") == "1"
# When enabled, a missing Stage1 TX/RX crop specification is a hard failure.
# The CLI enables this for channel-generation workflows to bound the static
# geometry sent to Sionna and its GPU memory footprint.
XML_STATIC_REQUIRE_CROP = os.environ.get("XML_STATIC_REQUIRE_CROP", "0") == "1"
XML_STATIC_BBOX_EXTEND = float(os.environ.get("XML_STATIC_BBOX_EXTEND", "100"))
# Include the RSU/static-RX placeholder props (sign box + pole) in the DYNAMIC XML.
# Default 0: they are visualization-only props that sit exactly on the RF
# antenna and block the LOS in ray tracing, so they are omitted.  Set to 1
# only for pure-visualization exports (stage3 sanitization strips them again
# before ray tracing either way).
XML_EXPORT_RSU_PROPS = os.environ.get("XML_EXPORT_RSU_PROPS", "0") == "1"
XML_STATIC_SAVE_BLEND = os.environ.get("XML_STATIC_SAVE_BLEND", "1") == "1"
XML_STATIC_DROP_SKY = os.environ.get("XML_STATIC_DROP_SKY", "1") == "1"
XML_STATIC_DTT_STREETLIGHT_MODE = normalize_dtt_streetlight_mode(
    os.environ.get("XML_STATIC_DTT_STREETLIGHT_MODE", "light_only")
)
XML_STATIC_OSM_STREETLIGHT_MATCH_TOLERANCE_M = float(
    os.environ.get("XML_STATIC_OSM_STREETLIGHT_MATCH_TOLERANCE_M", "0.05")
)
# UE/DTT exports may contain invisible vegetation impostor sheets or stale
# small wood blockers. These artifacts can create false LOS/NLOS blockage
# in Sionna. Keep this as a static-XML export cleanup so
# retained objects preserve their original transforms unchanged.
XML_STATIC_DROP_GHOST_BLOCKERS = os.environ.get("XML_STATIC_DROP_GHOST_BLOCKERS", "1") == "1"
XML_STATIC_GHOST_MAX_HORIZONTAL_M = float(os.environ.get("XML_STATIC_GHOST_MAX_HORIZONTAL_M", "4.0"))
XML_STATIC_GHOST_MIN_HEIGHT_M = float(os.environ.get("XML_STATIC_GHOST_MIN_HEIGHT_M", "2.0"))
XML_STATIC_GHOST_MAX_HEIGHT_M = float(os.environ.get("XML_STATIC_GHOST_MAX_HEIGHT_M", "12.0"))
XML_TOWN_KEY = os.environ.get("XML_TOWN_KEY", "").strip()

# In dynamic mode, default frame output directory is stage2_xml/TownXX_xxxxxx/dynamic unless user explicitly sets it.
if XML_EXPORT_MODE == "dynamic" and "MITSUBA_OUTPUT_DIR" not in os.environ:
    OUTPUT_SUBDIR = "dynamic"

XML_FRAME_SUBDIR = os.environ.get("XML_FRAME_SUBDIR", "0") == "1"

# 关闭每帧全场景材质扫描可明显加速。需要排查时设为 1。
XML_WARN_MATERIALS = os.environ.get("XML_WARN_MATERIALS", "0") == "1"

# Quiet external-call mode.
# Default:
#   XML_VERBOSE=0        only print important progress
#   XML_QUIET_FBX=1     suppress Blender FBX importer logs
#   XML_QUIET_MITSUBA=1 suppress Mitsuba exporter warnings
XML_VERBOSE = os.environ.get("XML_VERBOSE", "0") == "1"
XML_QUIET_FBX = os.environ.get("XML_QUIET_FBX", "1") == "1"
XML_QUIET_MITSUBA = os.environ.get("XML_QUIET_MITSUBA", "1") == "1"
XML_PROGRESS_BAR_WIDTH = int(os.environ.get("XML_PROGRESS_BAR_WIDTH", "28"))

# Resume support: skip frames whose XML already exists and is non-empty.
XML_SKIP_EXISTING = os.environ.get("XML_SKIP_EXISTING", "1") == "1"

# Dynamic actor export policy.
# Stage1 world_state is authoritative: export every CAV and every UAV node
# present in the frame.


# CAV placement policy:
#   1: Prefer the selected actor entry inside vehicles[actor_id] for CAV mesh pose/model.
#      This makes the CAV mesh use the same location/angle/bp_id convention as
#      background vehicles, while still exporting it as actor_name="CAV".
#   0: Old behavior, use sensors.vehicle_pose directly.
XML_CAV_FROM_SELECTED_VEHICLE = os.environ.get("XML_CAV_FROM_SELECTED_VEHICLE", "1") == "1"

# Require successful CAV mesh placement before exporting each frame.
XML_FAIL_IF_CAV_MISSING = os.environ.get("XML_FAIL_IF_CAV_MISSING", "1") == "1"

# Nearest-vehicle fallback threshold for records with an implicit selected vehicle ID.
XML_CAV_NEAREST_EPS_M = float(os.environ.get("XML_CAV_NEAREST_EPS_M", "2.0"))

# 模型本地姿态额外修正，默认旋转修正量为零。
# 例如模板 FBX 在 Blender 里躺着，可试：
#   export MODEL_EXTRA_ROT_DEG="90,0,0"
MODEL_EXTRA_ROT_DEG = os.environ.get("MODEL_EXTRA_ROT_DEG", "0,0,0")

# Fix FBX/PLY local-origin offset.
# Some UE/CARLA FBX exports keep vertices far away from the model-local origin.
# Then actor pose is correct, but the visible mesh appears offset from Rx/CAV.
#
# For vehicle/truck models, re-center local geometry:
#   X/Y -> bbox center becomes 0
#   Z   -> bbox bottom becomes 0 if XML_RECENTER_Z_TO_BOTTOM=1
XML_RECENTER_MODEL_LOCAL_ORIGIN = os.environ.get("XML_RECENTER_MODEL_LOCAL_ORIGIN", "1") == "1"
XML_RECENTER_Z_TO_BOTTOM = os.environ.get("XML_RECENTER_Z_TO_BOTTOM", "1") == "1"
XML_RECENTER_ACTOR_CLASSES = set(
    x.strip().lower()
    for x in os.environ.get("XML_RECENTER_ACTOR_CLASSES", "vehicle,truck").split(",")
    if x.strip()
)

# Long-term dynamic export mode:
#   shared canonical local meshes + per-frame XML transforms.
#
# This avoids the Mitsuba-Blender exporter problem where per-frame actor
# transforms are baked into PLY files and shared names such as CAV__mesh_00.ply
# are overwritten by later frames.
#
# In dynamic mode with XML_DYNAMIC_SHARED_MESH=1:
#   - each FBX blueprint is imported/cleaned once per Blender process;
#   - local canonical PLY meshes are written once under DATASET_ROOT/_dynamic_shared_meshes;
#   - each frame XML references those shared meshes and writes only to_world transforms.
XML_DYNAMIC_SHARED_MESH = os.environ.get("XML_DYNAMIC_SHARED_MESH", "1") == "1"
XML_SHARED_MESH_ROOT = os.environ.get("XML_SHARED_MESH_ROOT", "").strip()
XML_SHARED_MESH_ABS_PATHS = os.environ.get("XML_SHARED_MESH_ABS_PATHS", "1") == "1"
XML_SHARED_MESH_SUBDIR = os.environ.get("XML_SHARED_MESH_SUBDIR", "_dynamic_shared_meshes")

# Force rewrite shared PLY files. Keep this on after fixing a writer bug or
# changing recenter/material split logic to regenerate matching PLY files.
XML_FORCE_REWRITE_SHARED_PLY = os.environ.get("XML_FORCE_REWRITE_SHARED_PLY", "1") == "1"

# Shared mesh directory naming.
# Default nohash:
#   _dynamic_shared_meshes/vehicle/mercedes/mesh_00_itu_metal.ply
# Set XML_SHARED_MESH_USE_HASH=1 to restore the older mercedes_<hash> style.
XML_SHARED_MESH_USE_HASH = os.environ.get("XML_SHARED_MESH_USE_HASH", "0") == "1"

# Cross-process shared-mesh writer lock. This prevents SLURM array jobs from
# simultaneously replacing the same shared PLY file on network filesystems.
XML_SHARED_MESH_LOCK_TIMEOUT_SEC = float(os.environ.get("XML_SHARED_MESH_LOCK_TIMEOUT_SEC", "600"))
XML_SHARED_MESH_LOCK_STALE_SEC = float(os.environ.get("XML_SHARED_MESH_LOCK_STALE_SEC", "3600"))

# Log/progress style:
#   compact: one carriage-return progress line; warnings/errors occupy their own lines.
#   verbose: regular detailed prints.
XML_PROGRESS_MODE = os.environ.get("XML_PROGRESS_MODE", "compact").strip().lower()
XML_PROGRESS_COMPACT = XML_PROGRESS_MODE in {"compact", "1", "true", "yes"}
_PROGRESS_LINE_ACTIVE = False

DYNAMIC_TAG = "_xml_export_dynamic_actor"
DYNAMIC_PREFIXES = ("Vehicle_", "Pedestrian_", "RSU_", "CAV_")




def vprint(*args, **kwargs):
    if XML_VERBOSE:
        print(*args, **kwargs)

@contextlib.contextmanager
def quiet_blender_output(enabled=True):
    """
    Suppress noisy stdout/stderr produced by Blender import/export operators.
    This hides FBX version lines and Mitsuba UV warnings in normal runs.
    Set XML_VERBOSE=1 or XML_QUIET_FBX=0 / XML_QUIET_MITSUBA=0 for debugging.
    """
    if not enabled:
        yield
        return

    sys.stdout.flush()
    sys.stderr.flush()

    old_stdout_fd = os.dup(1)
    old_stderr_fd = os.dup(2)

    try:
        with open(os.devnull, "w") as devnull:
            os.dup2(devnull.fileno(), 1)
            os.dup2(devnull.fileno(), 2)
            try:
                yield
            finally:
                sys.stdout.flush()
                sys.stderr.flush()
                os.dup2(old_stdout_fd, 1)
                os.dup2(old_stderr_fd, 2)
    finally:
        os.close(old_stdout_fd)
        os.close(old_stderr_fd)

def make_progress_bar(done, total, width=None):
    if width is None:
        width = XML_PROGRESS_BAR_WIDTH
    total = max(int(total), 1)
    done = min(max(int(done), 0), total)
    frac = done / total
    filled = int(width * frac)
    return "[" + "#" * filled + "-" * (width - filled) + "]", frac

def normalize_text(s):
    s = str(s).lower()
    s = re.sub(r"[^a-z0-9]+", "_", s)
    s = re.sub(r"_+", "_", s)
    return s.strip("_")

def compact_text(s):
    return normalize_text(s).replace("_", "")

def stable_int(text):
    h = hashlib.md5(str(text).encode("utf-8")).hexdigest()
    return int(h[:8], 16)

def to_list3(v):
    if isinstance(v, (list, tuple)):
        return float(v[0]), float(v[1]), float(v[2])
    if isinstance(v, dict):
        return float(v.get("x", 0.0)), float(v.get("y", 0.0)), float(v.get("z", 0.0))
    raise TypeError(f"Cannot interpret as 3D vector: {v!r}")

def carla_to_blender_rotation(rot, order="ryp"):
    """
    order='ryp':
        vehicle angle list: [roll, yaw, pitch]
    order='pyr':
        sensor dict/list: pitch, yaw, roll

    Preserve previous convention:
        Blender euler = (-roll, -pitch, -yaw)
    """
    if isinstance(rot, (list, tuple)):
        if order == "ryp":
            roll, yaw, pitch = float(rot[0]), float(rot[1]), float(rot[2])
        else:
            pitch, yaw, roll = float(rot[0]), float(rot[1]), float(rot[2])
    else:
        pitch = float(rot.get("pitch", 0.0))
        yaw = float(rot.get("yaw", 0.0))
        roll = float(rot.get("roll", 0.0))

    return -math.radians(roll), -math.radians(pitch), -math.radians(yaw)

def parse_extra_rot():
    try:
        vals = [float(x.strip()) for x in MODEL_EXTRA_ROT_DEG.split(",")]
        if len(vals) != 3:
            return 0.0, 0.0, 0.0
        return tuple(math.radians(v) for v in vals)
    except Exception:
        return 0.0, 0.0, 0.0

def glass_like_name(name):
    n = str(name).lower()
    keys = [
        "glass",
        "glasses",
        "window",
        "windshield",
        "windscreen",
        "light",
        "lights",
        "lamp",
        "headlight",
        "taillight",
        "mirror",
    ]
    return any(k in n for k in keys)

def _progress_finish_line():
    """
    If a carriage-return progress line is active, terminate it before writing a
    warning/error/status line. This keeps tail -f readable.
    """
    global _PROGRESS_LINE_ACTIVE, _PROGRESS_LAST_LEN
    if XML_PROGRESS_COMPACT and _PROGRESS_LINE_ACTIVE:
        print("", flush=True)
        _PROGRESS_LINE_ACTIVE = False
        _PROGRESS_LAST_LEN = 0

def progress_line(msg):
    """
    Overwrite a single progress line when viewed with tail -f.

    Pads only enough spaces to erase the *previous* progress message, so the
    line width follows the previous message and fits narrow terminals.
    """
    global _PROGRESS_LINE_ACTIVE, _PROGRESS_LAST_LEN
    if XML_PROGRESS_COMPACT:
        s = str(msg)
        # Ensure we overwrite the previous (possibly longer) line completely.
        if len(s) < _PROGRESS_LAST_LEN:
            s = s.ljust(_PROGRESS_LAST_LEN)
        _PROGRESS_LAST_LEN = len(s)
        print("\r" + s, end="", flush=True)
        _PROGRESS_LINE_ACTIVE = True
    else:
        print(str(msg), flush=True)

def progress_status(msg):
    """
    Non-warning status. Only shown when XML_VERBOSE=1.
    """
    if XML_VERBOSE:
        _progress_finish_line()
        print(str(msg), flush=True)

def warning_line(msg):
    _progress_finish_line()
    s = str(msg)
    if not s.startswith("WARNING"):
        s = "WARNING: " + s
    print(s, flush=True)

def error_line(msg):
    _progress_finish_line()
    s = str(msg)
    if not s.startswith("ERROR"):
        s = "ERROR: " + s
    print(s, flush=True)

def safe_int(x, default=None):
    try:
        return int(x)
    except Exception:
        return default

def actor_sort_key(item):
    actor_id, _vinfo = item
    aid = safe_int(actor_id, 999999999)
    return (0 if aid == 0 else 1, aid, str(actor_id))

def squared_distance(a, b):
    ax, ay, az = to_list3(a)
    bx, by, bz = to_list3(b)
    return (ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2

def infer_cav_bp(data):
    if data.get("selected_vehicle_bp"):
        return data["selected_vehicle_bp"]

    selected_vehicle = data.get("selected_vehicle", {}) or {}
    if isinstance(selected_vehicle, dict):
        for key in ["bp_id", "type", "blueprint", "vehicle_type"]:
            if selected_vehicle.get(key):
                return selected_vehicle[key]

    sensors = data.get("sensors", {}) or {}
    vp = sensors.get("vehicle_pose", {}) or {}

    if vp.get("bp_id"):
        return vp["bp_id"]

    vehicles = data.get("vehicles", {}) or {}
    selected_ids = collect_selected_vehicle_ids(data)

    for sid in selected_ids:
        if sid in vehicles and vehicles[sid].get("bp_id"):
            return vehicles[sid]["bp_id"]
        if safe_int(sid, None) in vehicles and vehicles[safe_int(sid)].get("bp_id"):
            return vehicles[safe_int(sid)]["bp_id"]

    # Last fallback: match by nearest location to vehicle_pose.
    if vp.get("location") is not None:
        best = None
        best_d2 = None

        for _actor_id, vinfo in vehicles.items():
            if not vinfo.get("bp_id") or vinfo.get("location") is None:
                continue
            bp = str(vinfo.get("bp_id", "")).lower()
            if "walker" in bp or "pedestrian" in bp:
                continue
            d2 = squared_distance(vp["location"], vinfo["location"])
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best = vinfo.get("bp_id")

        if best is not None and best_d2 is not None and best_d2 < 4.0:
            return best

    return "vehicle.car"

def get_vehicle_by_id(vehicles_dict, actor_id):
    """
    Robustly fetch one entry from YAML vehicles using str/int actor id.
    """
    if actor_id is None or not isinstance(vehicles_dict, dict):
        return None

    keys = [actor_id, str(actor_id)]
    iid = safe_int(actor_id, None)
    if iid is not None:
        keys.append(iid)

    for key in keys:
        if key in vehicles_dict and isinstance(vehicles_dict[key], dict):
            return vehicles_dict[key]

    return None

def is_same_location(loc_a, loc_b, eps=0.25):
    try:
        return squared_distance(loc_a, loc_b) <= eps * eps
    except Exception:
        return False

def clean_dynamic_actors():
    to_remove = []

    for obj in bpy.data.objects:
        if obj.get(DYNAMIC_TAG):
            to_remove.append(obj)
        elif any(obj.name.startswith(p) for p in DYNAMIC_PREFIXES):
            to_remove.append(obj)

    to_remove = sorted(set(to_remove), key=lambda o: len(o.children_recursive), reverse=True)

    for obj in to_remove:
        try:
            bpy.data.objects.remove(obj, do_unlink=True)
        except Exception:
            pass

    remove_unused_materials()


def collect_selected_vehicle_ids(data):
    ids = set()

    def add_id(value):
        if value is None:
            return
        try:
            if str(value).lower() in {"none", "null", ""}:
                return
        except Exception:
            pass
        ids.add(str(value))

    for key in ["selected_vehicle_id", "cav_id", "ego_id"]:
        if key in data:
            add_id(data[key])

    # Some YAML/meta files store the selected CAV in a nested dict.
    selected_vehicle = data.get("selected_vehicle", {}) or {}
    if isinstance(selected_vehicle, dict):
        for key in ["id", "actor_id", "vehicle_id", "selected_vehicle_id", "cav_id", "ego_id"]:
            if key in selected_vehicle:
                add_id(selected_vehicle[key])

    sensors = data.get("sensors", {}) or {}

    for key in ["selected_vehicle_id", "cav_id", "ego_id"]:
        if key in sensors:
            add_id(sensors[key])

    vp = sensors.get("vehicle_pose", {}) or {}

    for key in ["id", "actor_id", "vehicle_id"]:
        if isinstance(vp, dict) and key in vp:
            add_id(vp[key])

    # Multi-CAV Stage1 world_state stores all selected CAVs in
    # data["selected_cavs"]. Include every CAV id here so the background loop
    # exports each physical vehicle once.
    selected_cavs = data.get("selected_cavs", []) if isinstance(data, dict) else []
    if isinstance(selected_cavs, (list, tuple)):
        for cav in selected_cavs:
            if isinstance(cav, dict):
                add_id(cav.get("id", cav.get("vehicle_id", cav.get("actor_id"))))

    return ids


def remove_unused_materials():
    try:
        from .material_manager import remove_unused_materials as _impl
    except ImportError:
        from Blender_xml_storage.material_manager import remove_unused_materials as _impl
    return _impl()

MODEL_EXTRA_ROT = parse_extra_rot()
_PROGRESS_LAST_LEN = 0
__all__ = [k for k in globals() if not k.startswith("__")]
