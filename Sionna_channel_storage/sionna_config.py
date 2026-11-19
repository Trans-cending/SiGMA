"""YAML-derived Stage III numerical settings, bound in an isolated process.

Run: python -m sigma stage3 -c configs/town03.yaml.
"""

from __future__ import annotations

import csv
import os
import re
import sys
import copy
import time
import math
import shutil
import traceback
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

# Project utility imports keep TensorFlow loading deferred until runtime.
from .utils.data_utils import generate_gt_bartlett_codebook, generate_gt_bartlett_front_mask, reconstruct_farfield_H_upa_squeeze
# Optional near-field reconstruction helper. Missing optional functionality is
# handled explicitly; import-path compatibility fallbacks are intentionally gone.
try:
    from .utils.data_utils import reconstruct_nearfield_H_upa_squeeze
except ImportError:
    reconstruct_nearfield_H_upa_squeeze = None


# ============================================================
# User configuration via environment variables
# ============================================================

def env_flag(name: str, default: str = "0") -> bool:
    return str(os.environ.get(name, default)).strip().lower() in {"1", "true", "yes", "y", "on"}


DATASET_ROOT_ENV_EXPLICIT = "DATASET_ROOT" in os.environ
DATASET_ROOT = Path(os.environ.get("DATASET_ROOT", "outputs/site")).resolve()
STAGE3_SUBDIR = os.environ.get("STAGE3_SUBDIR", "stage3_radio")
# Stage3 writes only under DATASET_ROOT/STAGE3_SUBDIR. DATASET_ROOT must be the
# map-scoped dataset root, e.g. /.../traj_dataset/Magic_D2D_Scenario3. The
# root is validated by the formal generator before any Stage3 output mkdir.
PATH_DATASET_ROOT = DATASET_ROOT
CHANNEL_DATASET_ROOT = DATASET_ROOT


def validate_stage3_dataset_root() -> Path:
    """Fail fast if Stage3 was launched with a non map-scoped dataset root."""
    if not DATASET_ROOT_ENV_EXPLICIT:
        raise RuntimeError(
            "Stage3 requires explicit DATASET_ROOT pointing to "
            "a map-scoped dataset root, e.g. /.../traj_dataset/Magic_D2D_Scenario3"
        )
    missing = [
        str(DATASET_ROOT / name)
        for name in ("stage1_multimodal", "stage2_xml")
        if not (DATASET_ROOT / name).is_dir()
    ]
    if missing:
        raise RuntimeError(
            "Invalid Stage3 DATASET_ROOT; expected a map-scoped root containing "
            "stage1_multimodal/ and stage2_xml/. Missing: " + ", ".join(missing)
        )
    if DATASET_ROOT.name == "traj_dataset":
        raise RuntimeError(
            "Invalid Stage3 DATASET_ROOT appears to be the dataset base root, not a "
            "scenario/map root. Use /.../traj_dataset/<MAP_NAME>."
        )
    return DATASET_ROOT

DYNAMIC_SUBDIR = os.environ.get("DYNAMIC_SUBDIR", os.path.join("stage2_xml", "{trajectory}", "dynamic"))
NUM_CAVS = int(os.environ.get("NUM_CAVS", "1"))


def parse_radio_node_selector(primary_name: str, default: str = "0") -> tuple[int, ...] | None:
    """Parse selected Stage1 node IDs.

    Returns None for all nodes. Otherwise returns sorted unique one-based IDs.
    Accepts values such as "1,2,3", "2,1", "cav_002,cav_001", or "all"/"0".
    """
    raw = os.environ.get(primary_name, default)
    text = str(raw).strip()
    if text == "" or text.lower() in {"0", "all", "auto", "*"}:
        return None
    if text.lower() in {"none", "null", "no", "-"}:
        return tuple()
    ids = []
    for token in re.split(r"[,;:\s]+", text):
        token = token.strip()
        if not token:
            continue
        m = re.search(r"(\d+)$", token)
        if not m:
            raise ValueError(f"{primary_name}: cannot parse node id token {token!r} from {text!r}")
        value = int(m.group(1))
        if value < 1:
            raise ValueError(f"{primary_name}: node ids must be >= 1, got {value}")
        ids.append(value)
    return tuple(sorted(set(ids)))


def radio_selector_label(selector: tuple[int, ...] | None) -> str:
    if selector is None:
        return "all"
    if not selector:
        return "none"
    return ",".join(str(x) for x in selector)


RADIO_RSU_IDS = parse_radio_node_selector("RADIO_RSU_IDS", "0")
RADIO_CAV_IDS = parse_radio_node_selector("RADIO_CAV_IDS", "0")
RADIO_UAV_IDS = parse_radio_node_selector("RADIO_UAV_IDS", "0")
RADIO_STATIC_RX_IDS = parse_radio_node_selector("RADIO_STATIC_RX_IDS", "0")

def parse_channel_links(raw: str) -> tuple[tuple[str, str], ...]:
    text = str(raw or "").strip()
    if not text:
        return tuple()
    out = []
    for item in re.split(r"[;\n]+", text):
        item = item.strip()
        if not item:
            continue
        parts = [x.strip() for x in re.split(r"(?:->|:|,)", item) if x.strip()]
        if len(parts) != 2:
            raise ValueError(f"CHANNEL_LINKS entry must be tx:rx or tx->rx, got {item!r}")
        out.append((parts[0], parts[1]))
    return tuple(out)

CHANNEL_LINKS = parse_channel_links(os.environ.get("CHANNEL_LINKS", ""))
# Pose source for Tx/Rx devices: Stage1 world_state is the single canonical
# pose source for Sionna. Legacy pose-file fallback paths were removed;
# Mobile RF phase centers follow unified bbox-top semantics: CAV uses
# "bbox top + RX_ANTENNA_HEIGHT_M" and UAV uses
# "bbox top + UAV_ANTENNA_HEIGHT_M". RSU/static poses come from their
# fixed nodes in the world_state YAML.
FREQUENCY_HZ = float(os.environ.get("FREQUENCY_HZ", "28e9"))
BANDWIDTH_HZ = float(os.environ.get("BANDWIDTH_HZ", "1e9"))
N_SUBCARRIERS = int(os.environ.get("N_SUBCARRIERS", "256"))

ANTENNA_PATTERN = os.environ.get("ANTENNA_PATTERN", "iso")
POLARIZATION = str(os.environ.get("POLARIZATION", "V")).strip().upper()
# The current SiGMA channel reconstruction stores one complex coefficient per
# spatial antenna element in H[rx_spatial, tx_spatial]. Ray-tracing polarization
# is scalar (V or H); dual-polarized patterns such as VH/cross require a
# port-aware channel tensor.
if POLARIZATION not in {"V", "H"}:
    raise ValueError(
        "SiGMA Stage3 scalar channel reconstruction supports POLARIZATION=V or H only; "
        f"got {POLARIZATION!r}. Dual-polarization (e.g. VH/cross) requires a "
        "port-aware channel reconstruction/output schema."
    )
ARRAY_SPACING_WL = float(os.environ.get("ARRAY_SPACING_WL", "0.5"))

# Multiple antenna configurations in h,v,h,v order:
#   rx_h,rx_v,tx_h,tx_v;rx_h,rx_v,tx_h,tx_v;...
# Requested default:
#   RX 1x1, TX 4x4 / 4x8 / 8x8
ARRAY_CONFIGS = os.environ.get("ARRAY_CONFIGS", "")

# Channel reconstruction mode:
#   farfield  : use reconstruct_farfield_H_upa_squeeze() from utils.data_utils
#   nearfield : configure physical PlanarArrays and trace each antenna pair
#               using explicit_array.py (synthetic_array=False).
CHANNEL_RECONSTRUCTION_MODE = os.environ.get("CHANNEL_RECONSTRUCTION_MODE", "farfield").strip().lower()
if CHANNEL_RECONSTRUCTION_MODE not in {"farfield", "nearfield"}:
    raise ValueError(f"Unknown CHANNEL_RECONSTRUCTION_MODE={CHANNEL_RECONSTRUCTION_MODE!r}")

# Back-lobe / backward-ray filtering passed to the UPA reconstruction helper.
# Default 1: front-facing UPA panels reject paths from the back hemisphere unless
# the caller explicitly requests ARRAY_IF_FILTER=0 for legacy/full-isotropic replay.
ARRAY_IF_FILTER = env_flag("ARRAY_IF_FILTER", os.environ.get("IF_FILTER", "1"))

# Rx position policy (stage1_world_state mode):
#   number    -> RX z = vehicle location z + bbox-top z (world_state
#                center.z + extent.z) + this value.
#                Default 1.5 m: antenna 1.5 m above the vehicle bbox top.
#   none      -> use the raw Stage1 node location. Stage1 records node poses;
#                Stage3 resolves antenna phase-center positions.
#
# Configuration note: channel.cav_antenna_height_m is exported only to Stage3 as
# RX_ANTENNA_HEIGHT_M. Stage1 vehicle selection, FoV, LOS and sensor geometry
# retain their configured values.
RX_ANTENNA_HEIGHT_M_STR = os.environ.get("RX_ANTENNA_HEIGHT_M", "1.5")
RX_ANTENNA_HEIGHT_M = None if RX_ANTENNA_HEIGHT_M_STR.strip().lower() in {"none", "null", ""} else float(RX_ANTENNA_HEIGHT_M_STR)

# UAV antenna phase center (stage1_world_state mode):
#   number    -> UAV z = uav location z + bbox-top z + this value.
#   none      -> use the uav location as the phase center.
UAV_ANTENNA_HEIGHT_M_STR = os.environ.get("UAV_ANTENNA_HEIGHT_M", "-0.5")
UAV_ANTENNA_HEIGHT_M = None if UAV_ANTENNA_HEIGHT_M_STR.strip().lower() in {"none", "null", ""} else float(UAV_ANTENNA_HEIGHT_M_STR)

TX_ANTENNA_OFFSET_CARLA_M = np.array(
    [float(x) for x in os.environ.get("TX_ANTENNA_OFFSET_CARLA_M", "0,0,0").split(",")],
    dtype=float,
)

# Sionna RT PathSolver settings
RT_MAX_DEPTH = int(os.environ.get("RT_MAX_DEPTH", "4"))
RT_LOS = os.environ.get("RT_LOS", "1") == "1"
RT_SPECULAR_REFLECTION = os.environ.get("RT_SPECULAR_REFLECTION", "1") == "1"
RT_DIFFUSE_REFLECTION = os.environ.get("RT_DIFFUSE_REFLECTION", "1") == "1"
RT_REFRACTION = os.environ.get("RT_REFRACTION", "0") == "1"
GLOBAL_SCATTERING_COEFFICIENT = float(os.environ.get("GLOBAL_SCATTERING_COEFFICIENT", "0.1"))
if not (0.0 <= GLOBAL_SCATTERING_COEFFICIENT <= 1.0):
    raise ValueError(
        "GLOBAL_SCATTERING_COEFFICIENT must be within [0,1], got "
        f"{GLOBAL_SCATTERING_COEFFICIENT}"
    )
# Sionna allocates a fraction S^2 of reflected power to diffuse scattering.
# When diffuse reflection is disabled, use an effective coefficient of zero so
# the specular component retains its full reflected power.
EFFECTIVE_SCATTERING_COEFFICIENT = (
    GLOBAL_SCATTERING_COEFFICIENT if RT_DIFFUSE_REFLECTION else 0.0
)
RT_DIFFRACTION = os.environ.get("RT_DIFFRACTION", "0") == "1"
RT_EDGE_DIFFRACTION = os.environ.get("RT_EDGE_DIFFRACTION", "0") == "1"
RT_SAMPLES_PER_SRC = int(os.environ.get("RT_SAMPLES_PER_SRC", "100000"))
RT_MAX_NUM_PATHS_PER_SRC = int(os.environ.get("RT_MAX_NUM_PATHS_PER_SRC", "100000"))
RT_SEED = int(os.environ.get("RT_SEED", "42"))
RT_SYNTHETIC_ARRAY = os.environ.get("RT_SYNTHETIC_ARRAY", "1") == "1"

MERGE_SHAPES = os.environ.get("MERGE_SHAPES", "1") == "1"
REMOVE_DUPLICATE_VERTICES = os.environ.get("REMOVE_DUPLICATE_VERTICES", "1") == "1"

# Stage3 duplicate-path protection. Raw Sionna paths are kept in the dataset;
# dedup_keep only controls which representative paths enter H. Only repeated
# pure-specular paths with the same ordered interaction/object/primitive chain
# are grouped. Numeric scales below select the group's deterministic medoid;
# distinct chains and LOS/diffuse/mixed paths retain their identities.
PATH_GEOMETRY_DEDUP_ENABLED = env_flag("PATH_GEOMETRY_DEDUP_ENABLED", "1")
# Bump when dedup semantics change to regenerate matching path/channel artifacts.
PATH_DEDUP_SCHEMA_VERSION = 4
PATH_DEDUP_DELAY_TOL_S = float(os.environ.get("PATH_DEDUP_DELAY_TOL_S", "2e-12"))
PATH_DEDUP_ANGLE_TOL_RAD = float(os.environ.get("PATH_DEDUP_ANGLE_TOL_RAD", "2e-4"))
PATH_DEDUP_COEFF_MAG_REL_TOL = float(os.environ.get("PATH_DEDUP_COEFF_MAG_REL_TOL", "1e-3"))
PATH_DEDUP_COEFF_PHASE_TOL_RAD = float(os.environ.get("PATH_DEDUP_COEFF_PHASE_TOL_RAD", "1e-3"))
PATH_DEDUP_VERTEX_SCALE_M = float(os.environ.get("PATH_DEDUP_VERTEX_SCALE_M", "0.01"))
SIONNA_XML_EXACT_SHAPE_DEDUP = env_flag("SIONNA_XML_EXACT_SHAPE_DEDUP", "1")

# Visualization
VIS_ENABLE = os.environ.get("VIS_ENABLE", "0") == "1"
VIS_EVERY_N = int(os.environ.get("VIS_EVERY_N", "30"))
VIS_RENDER_SCENE = os.environ.get("VIS_RENDER_SCENE", "1") == "1"
VIS_RENDER_SCENE_EVERY_N = int(os.environ.get("VIS_RENDER_SCENE_EVERY_N", "90"))
VIS_RENDER_RAYS = os.environ.get("VIS_RENDER_RAYS", "1") == "1"
VIS_RENDER_MAX_PATHS = int(os.environ.get("VIS_RENDER_MAX_PATHS", "50"))
if VIS_RENDER_MAX_PATHS < 0:
    raise ValueError("VIS_RENDER_MAX_PATHS must be >= 0")
VIS_TOPDOWN_HEIGHT_M = float(os.environ.get("VIS_TOPDOWN_HEIGHT_M", "150.0"))
VIS_TX_TO_RX_HEIGHT_M = float(os.environ.get("VIS_TX_TO_RX_HEIGHT_M", "12.0"))
VIS_TX_TO_RX_BACKOFF_M = float(os.environ.get("VIS_TX_TO_RX_BACKOFF_M", "10.0"))
VIS_RESOLUTION = tuple(int(x) for x in os.environ.get("VIS_RESOLUTION", "1280,720").split(","))
VIS_NUM_SAMPLES = int(os.environ.get("VIS_NUM_SAMPLES", "64"))
VIS_LIGHTING_SCALE = float(os.environ.get("VIS_LIGHTING_SCALE", "2.0"))
DEVICE_DISPLAY_RADIUS_M = float(os.environ.get("DEVICE_DISPLAY_RADIUS_M", "1.25"))

BARTLETT_IMG_H = int(os.environ.get("BARTLETT_IMG_H", "90"))
BARTLETT_IMG_W = int(os.environ.get("BARTLETT_IMG_W", "360"))
BARTLETT_DB_FLOOR = float(os.environ.get("BARTLETT_DB_FLOOR", "-40"))

SKIP_EXISTING_FRAMES = os.environ.get("SKIP_EXISTING_FRAMES", "1") == "1"
KEEP_WORK_XML = os.environ.get("KEEP_WORK_XML", "1") == "1"
WORK_SUBDIR = os.environ.get("WORK_SUBDIR", "sionna_dataset_work")

PROGRESS_MODE = os.environ.get("PROGRESS_MODE", "compact").strip().lower()
PROGRESS_COMPACT = PROGRESS_MODE in {"compact", "1", "true", "yes"}
_PROGRESS_ACTIVE = False


# ============================================================
# Logging
# ============================================================

def _finish_progress_line():
    global _PROGRESS_ACTIVE
    if PROGRESS_COMPACT and _PROGRESS_ACTIVE:
        print("", flush=True)
        _PROGRESS_ACTIVE = False


_PROGRESS_LAST_LEN = 0

def progress_line(msg: str):
    global _PROGRESS_ACTIVE, _PROGRESS_LAST_LEN
    if PROGRESS_COMPACT:
        s = str(msg)
        if len(s) < _PROGRESS_LAST_LEN:
            s = s.ljust(_PROGRESS_LAST_LEN)
        _PROGRESS_LAST_LEN = len(s)
        print("\r" + s, end="", flush=True)
        _PROGRESS_ACTIVE = True
    else:
        print(str(msg), flush=True)


def status_line(msg: str):
    _finish_progress_line()
    print(str(msg), flush=True)


def warning_line(msg: str):
    _finish_progress_line()
    s = str(msg)
    if not s.startswith("WARNING"):
        s = "WARNING: " + s
    print(s, flush=True)


def error_line(msg: str):
    _finish_progress_line()
    s = str(msg)
    if not s.startswith("ERROR"):
        s = "ERROR: " + s
    print(s, flush=True)


# ============================================================
# Array configuration helpers
# ============================================================

def parse_array_configs() -> list[dict]:
    configs = []
    seen = set()
    for item in ARRAY_CONFIGS.split(";"):
        item = item.strip()
        if not item:
            continue
        vals = [int(x.strip()) for x in item.split(",")]
        if len(vals) != 4:
            raise ValueError(f"Invalid ARRAY_CONFIGS item {item!r}; expected rx_h,rx_v,tx_h,tx_v")
        rx_h, rx_v, tx_h, tx_v = vals
        if min(vals) <= 0:
            raise ValueError(f"Invalid non-positive array dimensions in {item!r}")
        key = (rx_h, rx_v, tx_h, tx_v)
        if key in seen:
            continue
        seen.add(key)
        configs.append({
            "rx_h": rx_h, "rx_v": rx_v, "rx_cols": rx_h, "rx_rows": rx_v,
            "tx_h": tx_h, "tx_v": tx_v, "tx_cols": tx_h, "tx_rows": tx_v,
            "name": f"{freq_label()}_{rx_h}x{rx_v}_{tx_h}x{tx_v}",
        })
    if not configs:
        raise ValueError("ARRAY_CONFIGS produced no valid configs")
    return configs


def freq_label() -> str:
    ghz = FREQUENCY_HZ / 1e9
    if abs(ghz - round(ghz)) < 0.01:
        return f"{round(ghz)}GHz"
    return f"{ghz:.1f}GHz"


def default_channel_configs() -> list[dict]:
    """Stage3 radio channel matrix configurations.

    h001 is intended for RSU->CAV links. h002 is intended for RSU->UAV
    and CAV->UAV links. h003 is intended for RSU->static-RX links and keeps
    the static RX array symmetric with the default static TX/RSU 8x8 array.
    The legacy ARRAY_CONFIGS env var is still accepted as an escape hatch, but
    it enables all link types for every generated hXXX using the user-specified
    Tx/Rx dimensions.
    """
    if ARRAY_CONFIGS.strip():
        out = []
        for i, cfg in enumerate(parse_array_configs(), start=1):
            rec = dict(cfg)
            rec["id"] = f"h{i:03d}"
            rec["name"] = rec["id"]
            rec["enabled_link_types"] = ["rc", "ru", "cu", "rs"]
            rec["path_config"] = "p001"
            rec["center_frequency_hz"] = float(FREQUENCY_HZ)
            rec["bandwidth_hz"] = float(BANDWIDTH_HZ)
            rec["num_subcarriers"] = int(N_SUBCARRIERS)
            out.append(rec)
        return out

    base = {
        "path_config": "p001",
        "center_frequency_hz": float(FREQUENCY_HZ),
        "bandwidth_hz": float(BANDWIDTH_HZ),
        "num_subcarriers": int(N_SUBCARRIERS),
        "polarization": POLARIZATION,
        "element_spacing_lambda": [float(ARRAY_SPACING_WL), float(ARRAY_SPACING_WL)],
    }
    return [
        {
            **base,
            "id": "h001",
            "name": "h001",
            "enabled_link_types": ["rc"],
            "tx_rows": 8, "tx_cols": 8,
            "rx_rows": 4, "rx_cols": 4,
        },
        {
            **base,
            "id": "h002",
            "name": "h002",
            "enabled_link_types": ["ru", "cu"],
            "tx_rows": 4, "tx_cols": 4,
            "rx_rows": 2, "rx_cols": 2,
        },
        {
            **base,
            "id": "h003",
            "name": "h003",
            "enabled_link_types": ["rs"],
            "tx_rows": 8, "tx_cols": 8,
            "rx_rows": 8, "rx_cols": 8,
        },
    ]


ARRAY_CONFIG_LIST = default_channel_configs()

# Output schema tag. Bump this when path/channel semantics change to
# regenerate cache files using the current output contract.
OUTPUT_SCHEMA_VERSION = "stage3_radio_npz_v1"

__all__=[k for k in globals() if not k.startswith('__')]
