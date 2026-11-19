# -*- coding: utf-8 -*-
"""Official CARLA DTT Generate and Save Map automation.

This script intentionally uses the already validated DigitalTwinHeadlessBridge
and the official visible button labels:
- Generate
- Save Map

The object paths are recorded, but the workflow resolves the buttons by
visible text because runtime Widget paths can be transient.
"""

from __future__ import print_function

import errno
import json
import os
import re
import time
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

import unreal


MAP_NAME = os.environ["MOSAIC_MAP_NAME"]
OSM_URL = os.environ["MOSAIC_OSM_URL"]
DTT_OSM_URL = os.environ.get("MOSAIC_DTT_OSM_URL", OSM_URL).strip()
ORIGIN_LAT = float(
    os.environ.get("MOSAIC_XODR_ORIGIN_LAT", os.environ["MOSAIC_CENTER_LAT"])
)
ORIGIN_LON = float(
    os.environ.get("MOSAIC_XODR_ORIGIN_LON", os.environ["MOSAIC_CENTER_LON"])
)
END_LAT = float(
    os.environ.get("MOSAIC_XODR_END_LAT", os.environ["MOSAIC_MAX_LAT"])
)
END_LON = float(
    os.environ.get("MOSAIC_XODR_END_LON", os.environ["MOSAIC_MAX_LON"])
)
REPORT = Path(os.environ["MOSAIC_DTT_REPORT_JSON"])
UE_LOG_PATH = Path(os.environ.get("MOSAIC_DTT_UE_LOG", ""))
LOCAL_OSM_PATH = os.environ.get("MOSAIC_DTT_LOCAL_OSM_PATH", "").strip()
FLAT_TERRAIN = os.environ.get("MOSAIC_DTT_FLAT_TERRAIN", "1").strip().lower() in (
    "1", "true", "yes", "on"
)

WORKFLOW_TIMEOUT = int(
    os.environ.get("MOSAIC_DTT_TIMEOUT_SECONDS", "10800")
)
CONSTRUCT_WAIT = float(
    os.environ.get("MOSAIC_DTT_CONSTRUCT_WAIT_SECONDS", "12")
)
GENERATE_STABLE_SECONDS = int(
    os.environ.get("MOSAIC_DTT_GENERATE_STABLE_SECONDS", "180")
)
GENERATE_NO_OUTPUT_TIMEOUT = int(
    os.environ.get(
        "MOSAIC_DTT_GENERATE_NO_OUTPUT_TIMEOUT_SECONDS",
        "1200",
    )
)
GENERATE_NO_GEOMETRY_TIMEOUT = int(
    os.environ.get(
        "MOSAIC_DTT_GENERATE_NO_GEOMETRY_TIMEOUT_SECONDS",
        "7200",
    )
)
MIN_TILE_LEVEL_BYTES = int(
    os.environ.get(
        "MOSAIC_DTT_MIN_TILE_LEVEL_BYTES",
        "65536",
    )
)
SAVE_STABLE_SECONDS = int(
    os.environ.get("MOSAIC_DTT_SAVE_STABLE_SECONDS", "90")
)
SAVE_OBSERVE_SECONDS = int(
    os.environ.get("MOSAIC_DTT_SAVE_OBSERVE_SECONDS", "300")
)
SCAN_INTERVAL_SECONDS = float(
    os.environ.get("MOSAIC_DTT_SCAN_INTERVAL_SECONDS", "5")
)

WIDGET_ASSET = (
    "/CarlaTools/OnroadMapGenerator/UW_OnRoadMainWidget"
)
ODM_CLASS = (
    "/CarlaTools/OnroadMapGenerator/BP_OpenDriveToMap."
    "BP_OpenDriveToMap_C"
)

KNOWN_GENERATE_BUTTON_PATH = (
    "/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt:"
    "UW_OnRoadMainWidget_C_0.WidgetTree.StartButton"
)
KNOWN_SAVE_BUTTON_PATH = (
    "/Engine/Transient.UW_OnRoadMainWidget_C_0:"
    "WidgetTree.SaveButton"
)

state = {
    "release_version": "1.0.6",
    "runner": "official-dtt-generate-save-geometry-validated",
    "map_name": MAP_NAME,
    "osm_url": OSM_URL,
    "dtt_osm_url": DTT_OSM_URL,
    "local_osm_path": LOCAL_OSM_PATH,
    "flat_terrain": FLAT_TERRAIN,
    "origin": {
        "latitude": ORIGIN_LAT,
        "longitude": ORIGIN_LON,
    },
    "end": {
        "latitude": END_LAT,
        "longitude": END_LON,
    },
    "known_button_paths": {
        "generate": KNOWN_GENERATE_BUTTON_PATH,
        "save_map": KNOWN_SAVE_BUTTON_PATH,
    },
    "started_at": time.time(),
    "generate_triggered": False,
    "save_triggered": False,
    "save_file_delta_detected": False,
    "finished": False,
    "scan_interval_seconds": SCAN_INTERVAL_SECONDS,
    "transient_scan_error_count": 0,
}

widget = None
odm = None
tick_handle = None

# 0 wait for Widget construction
# 1 Generate delegate executing
# 2 monitor Generate output
# 3 Save Map delegate executing
# 4 monitor Save output
# 5 finished
stage = 0
stage_started = time.time()

last_signature = None
last_change_time = time.time()
pre_save_snapshot = None
last_scan_attempt_time = 0.0
last_fatal_log_size = 0


class TransientFilesystemError(RuntimeError):
    """A temporary NFS/directory replacement error during DTT output scan."""


TRANSIENT_SCAN_ERRNOS = set(
    value
    for value in (
        getattr(errno, "ESTALE", 116),
        getattr(errno, "ENOENT", 2),
        getattr(errno, "EIO", 5),
    )
    if value is not None
)

FATAL_LOG_PATTERNS = (
    re.compile(r"Assertion failed:", re.IGNORECASE),
    re.compile(r"\bSignal\s+11\b", re.IGNORECASE),
    re.compile(r"CommonUnixCrashHandler", re.IGNORECASE),
    re.compile(r"Engine crash handling finished", re.IGNORECASE),
    re.compile(
        r"MergeStaticMeshActors\..*Failed to convert",
        re.IGNORECASE,
    ),
)


def log(message):
    unreal.log("[MOSAIC_RELEASE_GENERATE] " + str(message))


def write_report():
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    state["updated_at"] = time.time()
    temporary = REPORT.with_suffix(REPORT.suffix + ".tmp")
    temporary.write_text(
        json.dumps(state, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    temporary.replace(REPORT)


def finish(reason, success):
    global stage
    global tick_handle

    if stage == 5:
        return

    stage = 5
    state["finished"] = True
    state["finish_reason"] = reason
    state["success"] = bool(success)
    state["finished_at"] = time.time()
    write_report()

    log("FINISH_REASON=" + reason)
    log("SUCCESS=" + str(int(bool(success))))

    if tick_handle is not None:
        try:
            unreal.unregister_slate_post_tick_callback(
                tick_handle
            )
        except Exception:
            pass
        tick_handle = None

    try:
        unreal.SystemLibrary.quit_editor()
    except Exception:
        pass


def spawn_widget():
    asset = unreal.load_asset(WIDGET_ASSET)
    if asset is None:
        raise RuntimeError(
            "Cannot load official DTT widget: " + WIDGET_ASSET
        )

    subsystem = unreal.get_editor_subsystem(
        unreal.EditorUtilitySubsystem
    )
    result = subsystem.spawn_and_register_tab_and_get_id(asset)

    if isinstance(result, (tuple, list)):
        for item in result:
            if isinstance(item, unreal.Object):
                return item

    if isinstance(result, unreal.Object):
        return result

    raise RuntimeError(
        "spawn_and_register_tab_and_get_id returned no Widget"
    )


def initialize_odm(root_widget):
    obj = root_widget.get_open_drive_to_map()

    if obj is None:
        cls = unreal.load_class(None, ODM_CLASS)
        if cls is None:
            raise RuntimeError("Cannot load " + ODM_CLASS)
        obj = root_widget.initialize_open_drive_to_map(cls)

    if obj is None:
        raise RuntimeError(
            "initialize_open_drive_to_map returned None"
        )

    obj.set_editor_property("map_name", MAP_NAME)
    if LOCAL_OSM_PATH:
        obj.set_editor_property("url", "")
        obj.set_editor_property("local_file_path", LOCAL_OSM_PATH)
    else:
        obj.set_editor_property("url", DTT_OSM_URL)
        obj.set_editor_property("local_file_path", "")
    obj.set_editor_property(
        "origin_geo_coordinates",
        unreal.Vector2D(ORIGIN_LAT, ORIGIN_LON),
    )

    return obj


def require_bridge_api():
    bridge = unreal.DigitalTwinHeadlessBridge
    required = [
        "list_buttons_by_visible_text_recursive",
        "find_button_by_visible_text_recursive",
        "click_button_by_visible_text_recursive",
        "set_widget_value_by_name_recursive",
    ]
    missing = [
        name for name in required
        if not hasattr(bridge, name)
    ]
    if missing:
        raise RuntimeError(
            "DigitalTwinHeadlessBridge missing methods: "
            + ", ".join(missing)
            + ". Build the optional bridge before using release."
        )


def inject_fields():
    bridge = unreal.DigitalTwinHeadlessBridge
    url_value = "" if LOCAL_OSM_PATH else DTT_OSM_URL
    local_path_value = LOCAL_OSM_PATH
    values = [
        ("FileName", MAP_NAME),
        ("MapName", MAP_NAME),
        ("URL", url_value),
        ("Url", url_value),
        ("LocalFilePath", local_path_value),
        ("Local File Path", local_path_value),
        ("FilePath", local_path_value),
        ("GeoCoordOriginX", str(ORIGIN_LAT)),
        ("GeoCoordOriginY", str(ORIGIN_LON)),
        ("GeoCoordEndX", str(END_LAT)),
        ("GeoCoordEndY", str(END_LON)),
    ]

    counts = {}
    for token, value in values:
        counts[token] = (
            bridge.set_widget_value_by_name_recursive(
                widget,
                token,
                value,
            )
        )

    state["field_injection_counts"] = counts
    write_report()

    missing_origin_fields = [
        token
        for token in (
            "GeoCoordOriginX",
            "GeoCoordOriginY",
            "GeoCoordEndX",
            "GeoCoordEndY",
        )
        if counts.get(token, 0) < 1
    ]
    if missing_origin_fields:
        raise RuntimeError(
            "DTT projection-origin widgets were not found: "
            + ", ".join(missing_origin_fields)
        )


def inventory_buttons():
    bridge = unreal.DigitalTwinHeadlessBridge
    inventory = list(
        bridge.list_buttons_by_visible_text_recursive(widget)
    )
    state["button_inventory"] = inventory
    write_report()

    log("BUTTON_WIDGET_COUNT=" + str(len(inventory)))
    for item in inventory:
        log("BUTTON_WIDGET=" + str(item))

    return inventory


def click_visible_button(label, prefix):
    bridge = unreal.DigitalTwinHeadlessBridge
    button_path = (
        bridge.find_button_by_visible_text_recursive(
            widget,
            label,
        )
    )
    state[prefix + "_button_path"] = button_path
    write_report()

    log(
        prefix.upper()
        + "_BUTTON_PATH="
        + str(button_path)
    )

    if not button_path:
        return False

    clicked = (
        bridge.click_button_by_visible_text_recursive(
            widget,
            label,
        )
    )

    state[prefix + "_triggered"] = bool(clicked)
    state[prefix + "_click_returned_at"] = time.time()
    write_report()
    return bool(clicked)


def map_directory():
    return (
        Path(unreal.Paths.project_content_dir())
        / "CustomMaps"
        / MAP_NAME
    )


def _raise_transient_scan_error(operation, path, exception):
    error_number = getattr(exception, "errno", None)

    if error_number in TRANSIENT_SCAN_ERRNOS:
        raise TransientFilesystemError(
            "{} failed for {}: errno={} {!r}".format(
                operation,
                path,
                error_number,
                exception,
            )
        )

    raise exception


def _strict_recursive_files(directory):
    """Return a complete file listing, raising on any subtree scan failure.

    DTT can replace OpenDrive directories while GenerateTileCommandlet writes
    them. Output stability requires a complete scan of the directory tree.
    """
    stack = [directory]
    files = []

    while stack:
        current = stack.pop()

        try:
            entries = list(os.scandir(str(current)))
        except OSError as exception:
            _raise_transient_scan_error(
                "scandir",
                current,
                exception,
            )

        for entry in entries:
            entry_path = Path(entry.path)

            try:
                if entry.is_dir(follow_symlinks=False):
                    stack.append(entry_path)
                elif entry.is_file(follow_symlinks=False):
                    files.append(entry_path)
            except OSError as exception:
                _raise_transient_scan_error(
                    "classify directory entry",
                    entry_path,
                    exception,
                )

    return sorted(files, key=lambda item: str(item))


def _generated_geometry_summary(directory, entries, stat_by_path):
    category_tokens = {
        "Building": (
            "/static/buildings/",
            "/static/roofs/",
            "buildingmesh",
            "roofmesh",
        ),
        "Road": (
            "/static/drivinglane/",
            "/static/driving_lane/",
            "drivinglane",
            "driving_lane",
        ),
        "SideWalk": (
            "/static/sidewalk/",
            "sidewalk",
            "side_walk",
        ),
        "RoadLine": (
            "/static/lanemark/",
            "lanemark",
            "lane_mark",
            "roadline",
        ),
        "Terrain": (
            "/static/terrain/",
            "terrainmesh",
        ),
    }

    category_files = dict(
        (name, [])
        for name in category_tokens
    )

    all_uassets = []
    non_opendrive_uassets = []

    for path in entries:
        if path.suffix.lower() != ".uasset":
            continue

        all_uassets.append(path)
        lower = str(path).replace("\\", "/").lower()

        if "/opendrive/" not in lower:
            non_opendrive_uassets.append(path)

        for category, tokens in category_tokens.items():
            if any(token in lower for token in tokens):
                category_files[category].append(path)

    tile_levels = sorted(
        path
        for path in entries
        if (
            path.parent == directory
            and path.name.startswith(MAP_NAME + "_Tile_")
            and path.suffix.lower() == ".umap"
        )
    )

    tile_level_sizes = dict(
        (
            str(path),
            int(stat_by_path.get(str(path), {}).get("size", 0)),
        )
        for path in tile_levels
    )

    largest_tile_level_bytes = max(
        list(tile_level_sizes.values()) or [0]
    )

    category_counts = dict(
        (
            name,
            len(paths),
        )
        for name, paths in category_files.items()
    )

    # A generated drivable map must contain driving-lane meshes. A tiny
    # Tile .umap around 8 KB is only an empty shell. The successful validated
    # BBox_001 Tile was about 1.18 MB.
    geometry_checks = {
        "generated_non_opendrive_uassets_exist":
            len(non_opendrive_uassets) > 0,
        "generated_road_assets_exist":
            category_counts.get("Road", 0) > 0,
        "tile_level_is_not_empty_shell":
            largest_tile_level_bytes >= MIN_TILE_LEVEL_BYTES,
    }

    return {
        "category_counts": category_counts,
        "category_sample_files": dict(
            (
                name,
                [str(path) for path in paths[:50]],
            )
            for name, paths in category_files.items()
        ),
        "total_uasset_count": len(all_uassets),
        "non_opendrive_uasset_count":
            len(non_opendrive_uassets),
        "non_opendrive_uasset_samples": [
            str(path)
            for path in non_opendrive_uassets[:100]
        ],
        "tile_level_sizes": tile_level_sizes,
        "largest_tile_level_bytes":
            largest_tile_level_bytes,
        "minimum_tile_level_bytes":
            MIN_TILE_LEVEL_BYTES,
        "checks": geometry_checks,
        "geometry_ready": all(geometry_checks.values()),
    }


def _required_assets_from_entries(
    directory,
    entries,
    stat_by_path,
):
    main_level = directory / (MAP_NAME + ".umap")

    tile_levels = sorted(
        path
        for path in entries
        if (
            path.parent == directory
            and path.name.startswith(MAP_NAME + "_Tile_")
            and path.suffix.lower() == ".umap"
        )
    )

    xodr_files = sorted(
        path for path in entries
        if path.suffix.lower() == ".xodr"
    )
    osm_files = sorted(
        path for path in entries
        if path.suffix.lower() == ".osm"
    )
    opendrive_assets = sorted(
        path
        for path in entries
        if (
            path.suffix.lower() == ".uasset"
            and (
                "opendrive" in str(path).lower()
                or path.stem.lower() == MAP_NAME.lower()
            )
        )
    )

    entry_strings = set(str(path) for path in entries)

    core_checks = {
        "main_level_exists": str(main_level) in entry_strings,
        "tile_level_exists": len(tile_levels) > 0,
        "xodr_exists": len(xodr_files) > 0,
        "osm_exists": len(osm_files) > 0,
        "opendrive_asset_exists":
            len(opendrive_assets) > 0,
    }

    geometry = _generated_geometry_summary(
        directory,
        entries,
        stat_by_path,
    )

    checks = dict(core_checks)
    checks.update(
        dict(
            (
                "geometry_" + name,
                value,
            )
            for name, value in geometry["checks"].items()
        )
    )

    return {
        "checks": checks,
        "core_checks": core_checks,
        "geometry": geometry,
        "main_level": str(main_level),
        "tile_levels": [str(path) for path in tile_levels],
        "xodr_files": [str(path) for path in xodr_files],
        "osm_files": [str(path) for path in osm_files],
        "opendrive_assets": [
            str(path) for path in opendrive_assets
        ],
        "core_ready": all(core_checks.values()),
        "geometry_ready": bool(geometry["geometry_ready"]),
        "all_required": (
            all(core_checks.values())
            and bool(geometry["geometry_ready"])
        ),
    }


def scan_outputs():
    """Perform one complete and internally consistent output-tree scan."""
    directory = map_directory()

    try:
        directory_exists = directory.exists()
    except OSError as exception:
        _raise_transient_scan_error(
            "exists",
            directory,
            exception,
        )

    files = (
        _strict_recursive_files(directory)
        if directory_exists
        else []
    )

    entries = []
    snapshot = {}
    suffix_counts = {}

    for path in files:
        try:
            stat = path.stat()
        except OSError as exception:
            _raise_transient_scan_error(
                "stat",
                path,
                exception,
            )

        item = {
            "path": str(path),
            "size": int(stat.st_size),
            "mtime_ns": int(stat.st_mtime_ns),
        }
        entries.append(item)
        snapshot[item["path"]] = {
            "size": item["size"],
            "mtime_ns": item["mtime_ns"],
        }

        suffix = path.suffix.lower() or "<none>"
        suffix_counts[suffix] = (
            suffix_counts.get(suffix, 0) + 1
        )

    output = {
        "directory": str(directory),
        "exists": directory_exists,
        "file_count": len(entries),
        "total_size": sum(
            item["size"] for item in entries
        ),
        "suffix_counts": suffix_counts,
        "sample_files": entries[:100],
    }

    signature = tuple(
        (
            item["path"],
            item["size"],
            item["mtime_ns"],
        )
        for item in entries
    )

    assets = _required_assets_from_entries(
        directory,
        files,
        snapshot,
    )

    return output, signature, snapshot, assets


def inspect_generated_xodr(assets):
    diagnostics = []
    for value in assets.get("xodr_files", []):
        path = Path(value)
        try:
            if not path.is_file() or path.stat().st_size < 100:
                continue
            root = ET.parse(str(path)).getroot()
        except (OSError, ET.ParseError):
            continue

        header = root.find("header")
        if header is None:
            continue
        bounds = {}
        try:
            bounds = dict(
                (name, float(header.get(name)))
                for name in ("west", "south", "east", "north")
            )
        except (TypeError, ValueError):
            pass
        georef_text = header.findtext("geoReference") or ""
        lat_match = re.search(r"(?:^|\s)\+lat_0=([^\s<]+)", georef_text)
        lon_match = re.search(r"(?:^|\s)\+lon_0=([^\s<]+)", georef_text)
        actual_lat = float(lat_match.group(1)) if lat_match else None
        actual_lon = float(lon_match.group(1)) if lon_match else None
        item = {
            "path": str(path),
            "bounds": bounds,
            "latitude": actual_lat,
            "longitude": actual_lon,
            "expected_latitude": ORIGIN_LAT,
            "expected_longitude": ORIGIN_LON,
        }
        diagnostics.append(item)

        origin_mismatch = (
            actual_lat is not None
            and actual_lon is not None
            and (
                abs(actual_lat - ORIGIN_LAT) > 0.0001
                or abs(actual_lon - ORIGIN_LON) > 0.0001
            )
        )
        excessive_coordinates = bool(bounds) and max(
            abs(value) for value in bounds.values()
        ) > 50000.0
        if origin_mismatch or excessive_coordinates:
            item["origin_mismatch"] = origin_mismatch
            item["excessive_coordinates"] = excessive_coordinates
            return diagnostics, item
    return diagnostics, None


def diff_snapshots(before, after):
    before = before or {}
    after = after or {}

    added = sorted(set(after) - set(before))
    removed = sorted(set(before) - set(after))
    modified = sorted(
        path
        for path in set(before) & set(after)
        if before[path] != after[path]
    )

    return {
        "added": added,
        "removed": removed,
        "modified": modified,
        "changed_count":
            len(added) + len(removed) + len(modified),
    }


def detect_fatal_ue_log():
    global last_fatal_log_size

    if not str(UE_LOG_PATH) or not UE_LOG_PATH.is_file():
        return None

    try:
        size = UE_LOG_PATH.stat().st_size
        start = max(0, min(last_fatal_log_size, size) - 4096)
        with UE_LOG_PATH.open("rb") as handle:
            handle.seek(start)
            raw = handle.read()
    except OSError:
        return None

    last_fatal_log_size = size
    text = (
        raw.replace(b"\x00", b"")
        .decode("utf-8", errors="replace")
        .replace("\r", "")
    )
    lines = [line.strip() for line in text.splitlines() if line.strip()]

    fatal_line = None
    for line in lines:
        if any(pattern.search(line) for pattern in FATAL_LOG_PATTERNS):
            fatal_line = line[-1000:]

    if fatal_line is None:
        return None

    last_building = None
    for line in lines:
        if "MOSAIC_BUILDING_DEBUG" in line:
            last_building = line[-1500:]

    return {
        "ue_log": str(UE_LOG_PATH),
        "fatal_line": fatal_line,
        "last_building_debug": last_building,
    }


def tick(_delta_seconds):
    global stage
    global stage_started
    global last_signature
    global last_change_time
    global pre_save_snapshot
    global last_scan_attempt_time

    now = time.time()

    try:
        if stage in (1, 3, 5):
            return

        if stage == 0:
            if now - stage_started < CONSTRUCT_WAIT:
                return

            inject_fields()
            inventory_buttons()

            stage = 1
            state["generate_click_started_at"] = now
            state["generate_click_in_progress"] = True
            write_report()

            clicked = click_visible_button(
                "Generate",
                "generate",
            )

            state["generate_click_in_progress"] = False
            write_report()

            if not clicked:
                finish(
                    "visible_generate_button_click_failed",
                    False,
                )
                return

            log("VISIBLE_GENERATE_CLICK=1")
            stage = 2
            stage_started = time.time()
            last_change_time = time.time()
            last_signature = None
            last_scan_attempt_time = 0.0
            return

        if stage == 2:
            fatal = detect_fatal_ue_log()
            if fatal is not None:
                state["fatal_ue_log_detected"] = fatal
                write_report()
                finish("fatal_ue_log_detected", False)
                return

            if (
                now - last_scan_attempt_time
                < SCAN_INTERVAL_SECONDS
            ):
                return

            last_scan_attempt_time = now
            output, signature, snapshot, assets = (
                scan_outputs()
            )

            state["generate_outputs"] = output
            state["required_assets"] = assets
            xodr_diagnostics, xodr_failure = inspect_generated_xodr(assets)
            state["generated_xodr_projection"] = xodr_diagnostics
            if xodr_failure is not None:
                state["generated_xodr_projection_failure"] = xodr_failure
                write_report()
                finish("generated_xodr_projection_origin_mismatch", False)
                return
            state["elapsed_seconds"] = round(
                now - state["started_at"],
                1,
            )

            if signature != last_signature:
                last_signature = signature
                last_change_time = now
                log(
                    "GENERATE_OUTPUT_CHANGED="
                    + str(output["file_count"])
                    + " files"
                )
                geometry = assets.get("geometry", {})
                log(
                    "GENERATED_GEOMETRY_COUNTS="
                    + str(geometry.get("category_counts", {}))
                )
                log(
                    "LARGEST_TILE_LEVEL_BYTES="
                    + str(
                        geometry.get(
                            "largest_tile_level_bytes",
                            0,
                        )
                    )
                )

            state["generate_stable_seconds"] = round(
                now - last_change_time,
                1,
            )
            write_report()

            if (
                assets["all_required"]
                and now - last_change_time
                >= GENERATE_STABLE_SECONDS
            ):
                state["generate_completed_at"] = now
                state["pre_save_outputs"] = output
                pre_save_snapshot = snapshot
                write_report()

                log("GENERATE_COMPLETED_AND_STABLE=1")
                inventory_buttons()

                stage = 3
                state["save_click_started_at"] = time.time()
                state["save_click_in_progress"] = True
                write_report()

                clicked = click_visible_button(
                    "Save Map",
                    "save",
                )

                state["save_click_in_progress"] = False
                write_report()

                if not clicked:
                    finish(
                        "visible_save_map_button_click_failed",
                        False,
                    )
                    return

                log("VISIBLE_SAVE_MAP_CLICK=1")
                stage = 4
                stage_started = time.time()
                last_signature = None
                last_change_time = time.time()
                last_scan_attempt_time = 0.0
                return

            if (
                output["file_count"] == 0
                and now - stage_started
                >= GENERATE_NO_OUTPUT_TIMEOUT
            ):
                finish(
                    "generate_clicked_but_no_custom_map_output",
                    False,
                )
                return

            if (
                assets.get("core_ready")
                and not assets.get("geometry_ready")
                and now - stage_started
                >= GENERATE_NO_GEOMETRY_TIMEOUT
            ):
                state["geometry_failure_diagnosis"] = (
                    "Core OSM/OpenDRIVE/Level files exist, but "
                    "generated road meshes and a non-empty Tile "
                    "Level were not produced."
                )
                write_report()
                finish(
                    "generate_core_files_exist_but_geometry_missing",
                    False,
                )
                return

        if stage == 4:
            fatal = detect_fatal_ue_log()
            if fatal is not None:
                state["fatal_ue_log_detected"] = fatal
                write_report()
                finish("fatal_ue_log_detected", False)
                return

            if (
                now - last_scan_attempt_time
                < SCAN_INTERVAL_SECONDS
            ):
                return

            last_scan_attempt_time = now
            output, signature, snapshot, assets = (
                scan_outputs()
            )
            delta = diff_snapshots(
                pre_save_snapshot,
                snapshot,
            )

            state["post_save_outputs"] = output
            state["save_file_delta"] = delta
            state["required_assets"] = assets

            if delta["changed_count"] > 0:
                state["save_file_delta_detected"] = True

            if signature != last_signature:
                last_signature = signature
                last_change_time = now

            stable_seconds = now - last_change_time
            state["save_stable_seconds"] = round(
                stable_seconds,
                1,
            )
            state["save_observe_seconds"] = round(
                now - stage_started,
                1,
            )
            write_report()

            if (
                assets["all_required"]
                and state["save_file_delta_detected"]
                and stable_seconds >= SAVE_STABLE_SECONDS
            ):
                finish(
                    "generate_and_save_completed_with_file_delta",
                    True,
                )
                return

            if (
                assets["all_required"]
                and now - stage_started
                >= SAVE_OBSERVE_SECONDS
            ):
                # The validated DTT workflow frequently saves required
                # packages during Generate itself. Save Map can therefore
                # succeed while their modification times remain unchanged.
                finish(
                    "save_clicked_required_assets_exist_no_extra_delta",
                    True,
                )
                return

        if now - state["started_at"] >= WORKFLOW_TIMEOUT:
            finish("workflow_timeout", False)

    except TransientFilesystemError as exception:
        count = int(
            state.get("transient_scan_error_count", 0)
        ) + 1
        state["transient_scan_error_count"] = count
        state["last_transient_scan_error"] = repr(exception)
        state["last_transient_scan_error_at"] = time.time()
        write_report()

        # GenerateTileCommandlet atomically replaces NFS directories.
        # Preserve last_change_time and continue polling after this scan error.
        if count <= 10 or count % 20 == 0:
            log(
                "TRANSIENT_OUTPUT_SCAN_ERROR="
                + str(exception)
                + " count="
                + str(count)
            )
        return

    except Exception:
        state["tick_exception"] = traceback.format_exc()
        write_report()
        finish("tick_exception", False)


def main():
    global widget
    global odm
    global tick_handle
    global stage_started

    try:
        require_bridge_api()
        widget = spawn_widget()
        odm = initialize_odm(widget)

        state["widget_class"] = (
            widget.get_class().get_name()
        )
        state["odm_class"] = odm.get_class().get_name()
        write_report()

        log("WIDGET_CLASS=" + state["widget_class"])
        log("ODM_CLASS=" + state["odm_class"])
        log("OSM_URL=" + OSM_URL)
        log("DTT_OSM_URL=" + DTT_OSM_URL)
        if LOCAL_OSM_PATH:
            log("LOCAL_OSM_PATH=" + LOCAL_OSM_PATH)

        stage_started = time.time()
        tick_handle = (
            unreal.register_slate_post_tick_callback(tick)
        )

    except Exception:
        state["main_exception"] = traceback.format_exc()
        write_report()
        finish("main_exception", False)


main()
