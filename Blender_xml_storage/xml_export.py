# -*- coding: utf-8 -*-
"""SiGMA Stage 2 Blender scene reconstruction/XML export entry point."""
import os as _bootstrap_os, sys as _bootstrap_sys
import re
import time
_PROJECT_ROOT = _bootstrap_os.path.dirname(_bootstrap_os.path.dirname(_bootstrap_os.path.abspath(__file__)))
if _PROJECT_ROOT not in _bootstrap_sys.path:
    _bootstrap_sys.path.insert(0, _PROJECT_ROOT)

from mosaic_core.dataset import stage1_world_state_dir as _stage1_world_state_dir

try:
    from .blender_common import *
    from .asset_manager import *
    from .material_manager import *
    from .stage1_loader import *
    from .mesh_writer import *
    from .xml_writer import *
    from .scene_builder import *
except ImportError:
    from Blender_xml_storage.blender_common import *
    from Blender_xml_storage.asset_manager import *
    from Blender_xml_storage.material_manager import *
    from Blender_xml_storage.stage1_loader import *
    from Blender_xml_storage.mesh_writer import *
    from Blender_xml_storage.xml_writer import *
    from Blender_xml_storage.scene_builder import *

def _is_stage1_world_state(data):
    if not isinstance(data, dict):
        return False
    nodes = data.get("nodes")
    scene = data.get("scene")
    return isinstance(nodes, dict) and isinstance(scene, dict) and (
        "world_frame" in data or "frame_idx" in data or "trajectory_id" in data
    )

def _rotation_dict_to_actor_angle(rotation):
    if isinstance(rotation, dict):
        return [
            float(rotation.get("roll", 0.0)),
            float(rotation.get("yaw", 0.0)),
            float(rotation.get("pitch", 0.0)),
        ]
    vals = list3_from_location_dict_or_list(rotation)
    if vals is None:
        return [0.0, 0.0, 0.0]
    # Stage1 list rotations are normally [pitch, yaw, roll]. XML actor angle is [roll, yaw, pitch].
    return [float(vals[2]), float(vals[1]), float(vals[0])]

def _stage1_sorted_node_items(items):
    if isinstance(items, dict):
        values = list(items.values())
    elif isinstance(items, list):
        values = list(items)
    else:
        values = []
    values = [v for v in values if isinstance(v, dict)]
    return sorted(values, key=lambda x: (safe_int(x.get("index"), 999999), str(x.get("node_id") or x.get("label") or "")))

def normalize_stage1_world_state(data):
    """Convert stage1_multimodal/world_state YAML into xml_export's legacy frame shape."""
    if not _is_stage1_world_state(data):
        return data

    out = dict(data)
    out["_stage1_world_state"] = True
    if out.get("frame") is None and out.get("world_frame") is not None:
        out["frame"] = out.get("world_frame")

    scene = out.get("scene") if isinstance(out.get("scene"), dict) else {}
    vehicles = (
        scene.get("vehicles")
        or scene.get("vehicles_within_primary_rsu_range")
        or out.get("vehicles")
        or {}
    )
    if isinstance(vehicles, dict):
        out["vehicles"] = vehicles

    if "rf_scene" not in out and isinstance(scene.get("rf_scene"), dict):
        out["rf_scene"] = scene.get("rf_scene")

    nodes = out.get("nodes") if isinstance(out.get("nodes"), dict) else {}
    sensors = dict(out.get("sensors") or {})

    rsu_nodes = _stage1_sorted_node_items(nodes.get("rsu"))
    if rsu_nodes:
        rsu0 = rsu_nodes[0]
        if rsu0.get("location") is not None:
            sensors["rsu_pose"] = {
                "location": rsu0.get("location"),
                "rotation": rsu0.get("rotation", {}),
                "actor_id": rsu0.get("actor_id"),
                "node_id": rsu0.get("node_id"),
            }

    selected_cavs = []
    for i, cav in enumerate(_stage1_sorted_node_items(nodes.get("cav")), start=1):
        if cav.get("available") is False:
            continue
        loc = cav.get("location")
        if loc is None:
            continue
        actor_id = cav.get("actor_id")
        bp_id = cav.get("bp_id") or cav.get("type") or cav.get("blueprint") or "vehicle.car"
        rec = {
            "id": str(actor_id) if actor_id is not None else f"CAV_{i}",
            "actor_id": actor_id,
            "selected_label": cav.get("node_id") or f"cav_{i:03d}",
            "bp_id": str(bp_id),
            "type": str(bp_id),
            "location": loc,
            "angle": _rotation_dict_to_actor_angle(cav.get("rotation")),
            "rotation_order": "ryp",
            "_source": "stage1.nodes.cav",
            "_yaml_world_frame": safe_int(out.get("world_frame"), None),
        }
        selected_cavs.append(rec)

    if selected_cavs:
        out["selected_cavs"] = selected_cavs
        out["selected_vehicle"] = selected_cavs[0]
        out["selected_vehicle_id"] = selected_cavs[0].get("id")
        sensors["vehicle_pose"] = {
            "id": selected_cavs[0].get("id"),
            "actor_id": selected_cavs[0].get("actor_id"),
            "bp_id": selected_cavs[0].get("bp_id"),
            "location": selected_cavs[0].get("location"),
            "rotation": {
                "roll": selected_cavs[0]["angle"][0],
                "yaw": selected_cavs[0]["angle"][1],
                "pitch": selected_cavs[0]["angle"][2],
            },
        }

    if sensors:
        out["sensors"] = sensors
    return out


def _dataset_root_for_stage1_traj(traj_path):
    p = Path(traj_path).resolve()
    if p.parent.name == "stage1_multimodal":
        return str(p.parent.parent)
    return str(p.parent)

def _traj_id_from_name(name):
    text = str(name)
    m = re.search(r"(?:^|_)(\d{6})$", text)
    return int(m.group(1)) if m else 0

def _frame_idx_from_frame_name(frame_name):
    m = re.findall(r"(\d{6})(?!.*\d)", str(frame_name))
    if m:
        return int(m[-1])
    return safe_int(frame_name, 0)

def _dynamic_xml_name(traj_path=None, frame_name=None, frame_idx=None):
    if frame_idx is None:
        frame_idx = _frame_idx_from_frame_name(frame_name)
    return f"dynamic_{int(frame_idx):06d}.xml"


def _stage1_frame_idx(data):
    """Absolute CARLA frame_idx from a stage-1 world_state YAML.

    The stage-1 sensor images are named with this exact suffix
    (``rgb_front_%06d.png``), so using it for the stage-2 XML keeps channel
    / scene outputs aligned with RGB/depth/semantic frames. The separate
    world_state output counter starts at 0 after the pre-entry waiting frames.
    Returns None when every candidate field is unusable.
    """
    raw = (data or {}).get("frame_idx")
    if raw is None:
        return None
    try:
        return int(float(raw))
    except Exception:
        return None


def find_frame_dir(traj_path):
    stage1_dir = _stage1_world_state_dir(traj_path)
    if XML_INPUT_VIEW == "auto":
        if os.path.isdir(stage1_dir):
            return stage1_dir
        for name in ["rsu_1", "cav_1"]:
            p = os.path.join(traj_path, name)
            if os.path.isdir(p):
                return p
        return None

    if XML_INPUT_VIEW in {"stage1", "stage1_world_state", "world_state", "stage1_multimodal/world_state"}:
        if os.path.isdir(stage1_dir):
            return stage1_dir
        return None

    p = os.path.join(traj_path, XML_INPUT_VIEW)
    if os.path.isdir(p):
        return p

    return None

def process_trajectory(traj_path, model_library, traj_idx=None, traj_total=None, town_key=None):
    data_dir = find_frame_dir(traj_path)

    traj_name = os.path.basename(traj_path)
    town_label = town_key or "Town"
    traj_label = f"{traj_idx}/{traj_total}" if traj_idx is not None and traj_total is not None else "?"

    if data_dir is None:
        warning_line(f"SKIP {town_label} traj {traj_label} {traj_name}: no input view {XML_INPUT_VIEW}")
        return

    dataset_root = _dataset_root_for_stage1_traj(traj_path)
    output_root = os.path.join(dataset_root, "stage2_xml", os.path.basename(traj_path), "dynamic")
    os.makedirs(output_root, exist_ok=True)

    yaml_files = sorted(f for f in os.listdir(data_dir) if f.endswith(".yaml"))

    if not yaml_files:
        warning_line(f"SKIP {town_label} traj {traj_label} {traj_name}: no YAML frames")
        return

    total = len(yaml_files)

    # Stable actor -> model mapping for this trajectory.
    actor_model_cache = {}

    progress_status(f"  START {town_label} traj {traj_label} {traj_name}: {total} YAML frames")

    t_start = time.perf_counter()

    def show_progress(done, current_yaml, status=""):
        # Compact mode updates one line every frame. Non-compact mode throttles.
        if not XML_PROGRESS_COMPACT:
            if done > 0 and done % _PROGRESS_EVERY != 0 and done != total:
                return

        bar, frac = make_progress_bar(done, total)
        pct = frac * 100.0
        elapsed = time.perf_counter() - t_start
        if done > 0:
            eta_sec = elapsed / done * (total - done)
            if eta_sec < 60:
                eta_str = f"{eta_sec:.0f}s"
            elif eta_sec < 3600:
                eta_str = f"{eta_sec/60:.0f}m{eta_sec%60:.0f}s"
            else:
                eta_str = f"{eta_sec/3600:.1f}h"
        else:
            eta_str = "..."

        line = (
            f"  {town_label} traj {traj_label} {traj_name} | "
            f"yaml {current_yaml} | {bar} {done}/{total} frames {pct:5.1f}%"
            f" | ETA {eta_str}"
        )
        if status:
            line += f" | {status}"

        progress_line(line)

    _PROGRESS_EVERY = 25

    for idx, fname in enumerate(yaml_files):
        frame_name = os.path.splitext(fname)[0]
        yaml_path = os.path.join(data_dir, fname)

        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            error_line(f"reading YAML {fname}: {e}")
            continue

        # Stage-2/3 frame number follows the absolute CARLA frame_idx recorded
        # in the world_state YAML so it matches the stage-1 image suffixes
        # (rgb_front_000016.png etc.). Legacy inputs missing a
        # frame_idx field fall back to the YAML filename counter.
        frame_idx_abs = _stage1_frame_idx(data)
        out_xml_name = _dynamic_xml_name(
            frame_idx=frame_idx_abs if frame_idx_abs is not None else frame_name)
        if XML_FRAME_SUBDIR:
            expected_xml = os.path.join(output_root, os.path.splitext(out_xml_name)[0], "scene.xml")
        else:
            expected_xml = os.path.join(output_root, out_xml_name)

        if XML_SKIP_EXISTING and os.path.isfile(expected_xml) and os.path.getsize(expected_xml) > 0:
            show_progress(idx + 1, fname, "skip existing")
            continue

        # pose source for dynamic XML.
        data["_yaml_path"] = yaml_path
        data["_traj_path"] = traj_path
        data["_frame_name"] = frame_name
        data = normalize_stage1_world_state(data)

        try:
            process_frame(
                data=data,
                output_root=output_root,
                frame_name=os.path.splitext(out_xml_name)[0],
                actor_model_cache=actor_model_cache,
                model_library=model_library,
            )
            show_progress(idx + 1, fname, "done")
        except Exception as e:
            error_line(f"processing frame {fname}: {e}")
            # Record per-frame failures and continue processing the trajectory.
            continue

    _progress_finish_line()
    print(f"  DONE  {town_label} traj {traj_label} {traj_name}: {total} frames", flush=True)

def main():
    ensure_core_itu_materials()
    ensure_mitsuba_exporter()

    try:
        args = sys.argv[sys.argv.index("--") + 1:]
        dataset_root = args[0]
    except (ValueError, IndexError):
        print("Usage: blender -b TownXXFixed.blend -P xml_export.py -- /path/to/traj_dataset")
        sys.exit(1)

    dataset_root = os.path.abspath(dataset_root)

    if not os.path.isdir(dataset_root):
        print(f"ERROR: dataset root does not exist: {dataset_root}")
        sys.exit(1)

    if not FBX_ROOT.is_dir():
        print(f"ERROR: FBX_ROOT does not exist: {FBX_ROOT}")
        sys.exit(1)

    if XML_STATIC_FBX_PATH:
        import_static_fbx_scene(XML_STATIC_FBX_PATH, XML_STATIC_BLEND_OUTPUT)

    blend_name = os.path.basename(bpy.data.filepath)
    town_key = detect_static_town_key(blend_name)

    print("", flush=True)
    print("===== XML EXPORT START =====", flush=True)
    print(f"Blend file        = {blend_name}", flush=True)
    print(f"Town key          = {town_key}", flush=True)
    print(f"Dataset root      = {dataset_root}", flush=True)
    print(f"FBX_ROOT          = {FBX_ROOT}", flush=True)
    print(f"XML_INPUT_VIEW    = {XML_INPUT_VIEW}", flush=True)
    print("CAV export policy = all Stage1 CAV nodes", flush=True)
    print(f"OUTPUT_SUBDIR     = {OUTPUT_SUBDIR}", flush=True)
    print(f"XML_EXPORT_MODE   = {XML_EXPORT_MODE}", flush=True)
    print(f"MITSUBA_STATIC_DIR= {MITSUBA_STATIC_DIR}", flush=True)
    print(f"XML_STATIC_FBX_PATH    = {XML_STATIC_FBX_PATH}", flush=True)
    print(f"XML_STATIC_BLEND_OUTPUT= {XML_STATIC_BLEND_OUTPUT}", flush=True)
    print(f"XML_STATIC_DROP_SKY    = {XML_STATIC_DROP_SKY}", flush=True)
    print(
        f"XML_STATIC_DTT_STREETLIGHT_MODE = {XML_STATIC_DTT_STREETLIGHT_MODE}",
        flush=True,
    )
    print(f"XML_STATIC_DROP_GHOST_BLOCKERS = {XML_STATIC_DROP_GHOST_BLOCKERS}", flush=True)
    print(f"XML_FRAME_SUBDIR  = {XML_FRAME_SUBDIR}", flush=True)
    print(f"XML_WARN_MATERIALS= {XML_WARN_MATERIALS}", flush=True)
    print(f"MODEL_EXTRA_ROT   = {MODEL_EXTRA_ROT}", flush=True)
    print(f"XML_RECENTER_MODEL_LOCAL_ORIGIN = {XML_RECENTER_MODEL_LOCAL_ORIGIN}", flush=True)
    print(f"XML_RECENTER_Z_TO_BOTTOM        = {XML_RECENTER_Z_TO_BOTTOM}", flush=True)
    print(f"XML_RECENTER_ACTOR_CLASSES      = {sorted(XML_RECENTER_ACTOR_CLASSES)}", flush=True)
    print(f"XML_DYNAMIC_SHARED_MESH         = {XML_DYNAMIC_SHARED_MESH}", flush=True)
    print(f"XML_SHARED_MESH_ROOT            = {XML_SHARED_MESH_ROOT}", flush=True)
    print(f"XML_SHARED_MESH_SUBDIR          = {XML_SHARED_MESH_SUBDIR}", flush=True)
    print(f"XML_SHARED_MESH_ABS_PATHS       = {XML_SHARED_MESH_ABS_PATHS}", flush=True)
    print(f"XML_FORCE_REWRITE_SHARED_PLY    = {XML_FORCE_REWRITE_SHARED_PLY}", flush=True)
    print(f"XML_SHARED_MESH_USE_HASH        = {XML_SHARED_MESH_USE_HASH}", flush=True)
    print(f"XML_SHARED_MESH_LOCK_TIMEOUT    = {XML_SHARED_MESH_LOCK_TIMEOUT_SEC}", flush=True)
    print(f"XML_PROGRESS_MODE               = {XML_PROGRESS_MODE}", flush=True)
    print(f"XML_CAV_FROM_SELECTED_VEHICLE = {XML_CAV_FROM_SELECTED_VEHICLE}", flush=True)
    print(f"XML_FAIL_IF_CAV_MISSING       = {XML_FAIL_IF_CAV_MISSING}", flush=True)
    print(f"XML_CAV_NEAREST_EPS_M         = {XML_CAV_NEAREST_EPS_M}", flush=True)
    print("", flush=True)

    if XML_EXPORT_MODE == "static":
        export_static_town_once(dataset_root, town_key)
        print("", flush=True)
        print("===== XML STATIC EXPORT DONE =====", flush=True)
        return

    if XML_EXPORT_MODE == "dynamic":
        remove_static_scene_objects_for_dynamic_export()

    model_library = scan_fbx_library()

    input_root = os.path.join(dataset_root, "stage1_multimodal")
    if not os.path.isdir(input_root):
        raise FileNotFoundError(
            f"Canonical Stage1 root is missing: {input_root}; expected <map>/stage1_multimodal/NNNNNN"
        )
    all_dirs = sorted(
        d for d in os.listdir(input_root)
        if os.path.isdir(os.path.join(input_root, d))
    )

    target = [
        os.path.join(input_root, d)
        for d in all_dirs
        if _is_trajectory_dir_name(d)
    ]

    # Optional exact trajectory scope supplied by CLI configuration. This is
    # deliberately applied before XML_TRAJ_SUBSET sharding so array workers only
    # see trajectories created by the current run.
    traj_names_env = os.environ.get("XML_TRAJ_NAMES", "").strip()
    if traj_names_env:
        requested = {
            token.strip()
            for token in re.split(r"[,;:\s]+", traj_names_env)
            if token.strip()
        }
        invalid = sorted(name for name in requested if re.fullmatch(r"\d{6}", name) is None)
        if invalid:
            raise ValueError(
                f"XML_TRAJ_NAMES accepts canonical six-digit trajectory names only: {invalid[:6]}"
            )
        target = [p for p in target if os.path.basename(p) in requested]
        found = {os.path.basename(p) for p in target}
        missing = sorted(requested - found)
        print(
            f"Exact trajectory scope: requested={len(requested)} found={len(found)} "
            f"missing={missing[:6]}",
            flush=True,
        )

    if not target:
        print(
            f"No canonical six-digit trajectory folders for {town_key} in {input_root}",
            flush=True,
        )
        return

    # Subset support for parallel execution: XML_TRAJ_SUBSET=i/N
    subset_env = os.environ.get("XML_TRAJ_SUBSET", "")
    if subset_env:
        try:
            part_str, total_str = subset_env.split("/")
            part = int(part_str) - 1  # 0-based
            total_parts = int(total_str)
            if part < 0 or part >= total_parts:
                raise ValueError(f"subset part {part+1} out of range [1, {total_parts}]")
            original_count = len(target)
            selected_indices = list(range(part, original_count, total_parts))
            target = target[part::total_parts]
            preview = selected_indices[:6]
            suffix = "..." if len(selected_indices) > 6 else ""
            print(
                f"  Subset {part+1}/{total_parts}: strided trajectory indices "
                f"{preview}{suffix} ({len(target)}/{original_count})",
                flush=True,
            )
        except Exception as e:
            print(f"  WARNING: bad XML_TRAJ_SUBSET='{subset_env}' ({e}); processing all",
                  flush=True)

    if not target:
        print(
            f"[SKIP] Subset {subset_env or '<none>'} assigned no trajectories "
            f"for {town_key}; input_root={input_root}",
            flush=True,
        )
        print("===== XML EXPORT DONE =====", flush=True)
        return

    print(f"Found {len(target)} trajectory folders for {town_key}", flush=True)

    for i, traj in enumerate(target):
        print("", flush=True)
        print(f"[{i + 1}/{len(target)}] {os.path.basename(traj)}", flush=True)
        process_trajectory(traj, model_library, traj_idx=i + 1, traj_total=len(target), town_key=town_key)

    print("", flush=True)
    print("===== XML EXPORT DONE =====", flush=True)


if __name__ == '__main__':
    main()
