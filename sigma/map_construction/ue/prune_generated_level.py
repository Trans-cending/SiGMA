# -*- coding: utf-8 -*-
"""Clean generated map layers immediately after Digital Twin Tool save.

This runs inside UE4Editor after generation. By default it preserves and tags
DTT streetlights for runtime full/light_only/remove switching, keeps safe traffic
controls and generated vegetation, and removes vegetation/traffic-control objects
that violate the configured spatial safety checks. The
``MOSAIC_KEEP_GENERATED_VEGETATION=1`` override means unconditional vegetation
preservation and is intentionally stronger than the default safety-prune mode.

The pass edits and saves the loaded Level so cleanup and terrain/road layer fixes
persist for later pipeline stages and other projects that load the updated source
map. Road surfaces receive a small positive Z epsilon while terrain stays at its
generated height. This resolves coplanar depth conflicts while preserving
the terrain edge beside the road.
"""

from __future__ import print_function

import json
import math
import os
import re
import sys
import time
import traceback
from collections import Counter
from pathlib import Path

import unreal

SCRIPT_DIR = str(Path(__file__).resolve().parent)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from level_content_classification import (
    building_night_lighting_classification as classify_building_lighting,
    classify_semantic_context,
    is_osm_streetlamp,
    is_procedural_streetlight_context,
    is_traffic_control_context,
    normalized_context,
)


MAP_NAME = os.environ["MOSAIC_MAP_NAME"]
TARGET_LEVEL = os.environ["MOSAIC_PRUNE_TARGET_LEVEL"]
TARGET_LABEL = os.environ.get(
    "MOSAIC_PRUNE_TARGET_LABEL",
    "unknown",
)
REPORT_PATH = Path(os.environ["MOSAIC_PRUNE_REPORT"])
PRUNE_MARGIN_CM = float(
    os.environ.get("MOSAIC_PRUNE_VEGETATION_BUILDING_MARGIN_CM", "50.0")
)
DRY_RUN = os.environ.get(
    "MOSAIC_PRUNE_DRY_RUN",
    "0",
).strip().lower() in {"1", "true", "yes", "on"}
TERRAIN_MATERIAL = os.environ.get(
    "MOSAIC_PRUNE_TERRAIN_MATERIAL",
    os.environ.get("MOSAIC_PRECOOK_TERRAIN_MATERIAL", ""),
).strip()
TERRAIN_ROAD_MARGIN_CM = float(
    os.environ.get("MOSAIC_PRUNE_TERRAIN_ROAD_MARGIN_CM", "10.0")
)
TERRAIN_ROAD_Z_OFFSET_CM = float(
    os.environ.get("MOSAIC_PRUNE_TERRAIN_ROAD_Z_OFFSET_CM", "0.0")
)
ROAD_SURFACE_Z_OFFSET_CM = float(
    os.environ.get("MOSAIC_PRUNE_ROAD_SURFACE_Z_OFFSET_CM", "4.0")
)
TERRAIN_ROAD_MAX_COMPONENT_AREA_RATIO = float(
    os.environ.get("MOSAIC_PRUNE_TERRAIN_ROAD_MAX_COMPONENT_AREA_RATIO", "3.0")
)
KEEP_GENERATED_VEGETATION = os.environ.get(
    "MOSAIC_KEEP_GENERATED_VEGETATION",
    "0",
).strip().lower() in {"1", "true", "yes", "on"}
REMOVE_DTT_GENERATED_VEGETATION = os.environ.get(
    "MOSAIC_PRUNE_REMOVE_DTT_GENERATED_VEGETATION",
    "0",
).strip().lower() in {"1", "true", "yes", "on"}
PRUNE_VEGETATION_IN_DRIVING_LANES = True
DRIVING_LANE_SAMPLES_PATH = os.environ.get(
    "MOSAIC_PRUNE_DRIVING_LANE_SAMPLES",
    "",
).strip()
PRUNE_STREETLIGHTS_IN_DRIVING_LANES = True
DTT_STREETLIGHT_RUNTIME_TAG = "MOSAIC_DTT_PROCEDURAL_STREETLIGHT"
PLACE_OFFICIAL_STREETLIGHTS = os.environ.get(
    "MOSAIC_PLACE_OFFICIAL_STREETLIGHTS",
    "1",
).strip().lower() in {"1", "true", "yes", "on"}
REMOVE_PROCEDURAL_STREETLIGHTS = os.environ.get(
    "MOSAIC_PRUNE_REMOVE_PROCEDURAL_STREETLIGHTS",
    "0" if PLACE_OFFICIAL_STREETLIGHTS else "1",
).strip().lower() in {"1", "true", "yes", "on"}
STREETLIGHT_LANE_CLEARANCE_M = float(
    os.environ.get("MOSAIC_PRUNE_STREETLIGHT_LANE_CLEARANCE_M", "0.20")
)
DTT_TILE_SIZE_M = float(
    os.environ.get("MOSAIC_PRUNE_DTT_TILE_SIZE_M", "500.0")
)
STREETLIGHT_DEEP_INTRUSION_M = float(
    os.environ.get("MOSAIC_PRUNE_STREETLIGHT_DEEP_INTRUSION_M", "0.75")
)
PLACE_OFFICIAL_TRAFFIC_CONTROLS = os.environ.get(
    "MOSAIC_PLACE_OFFICIAL_TRAFFIC_CONTROLS",
    "1",
).strip().lower() in {"1", "true", "yes", "on"}
PRUNE_TRAFFIC_CONTROLS_BY_OSM = os.environ.get(
    "MOSAIC_PRUNE_TRAFFIC_CONTROLS_BY_OSM",
    "0" if PLACE_OFFICIAL_TRAFFIC_CONTROLS else "1",
).strip().lower() in {"1", "true", "yes", "on"}
TRAFFIC_SIGNAL_PLAN_PATH = os.environ.get(
    "MOSAIC_PRUNE_TRAFFIC_SIGNAL_PLAN",
    "",
).strip()
TRAFFIC_CONTROL_OSM_MATCH_RADIUS_M = float(
    os.environ.get("MOSAIC_PRUNE_TRAFFIC_CONTROL_OSM_MATCH_RADIUS_M", "20.0")
)
TILE_MATCH = re.search(r"_Tile_(-?\d+)_(-?\d+)$", TARGET_LABEL)
TILE_OFFSET_X_M = (
    int(TILE_MATCH.group(1)) * DTT_TILE_SIZE_M if TILE_MATCH else 0.0
)
TILE_OFFSET_Y_M = (
    -int(TILE_MATCH.group(2)) * DTT_TILE_SIZE_M if TILE_MATCH else 0.0
)

STATE = {
    "release_version": "1.0.2",
    "runner": "generated-level-map-layer-cleanup",
    "road_terrain_layer_strategy": "road_surface_epsilon_no_terrain_shift",
    "map_name": MAP_NAME,
    "target_level": TARGET_LEVEL,
    "target_label": TARGET_LABEL,
    "margin_cm": PRUNE_MARGIN_CM,
    "terrain_road_margin_cm": TERRAIN_ROAD_MARGIN_CM,
    "terrain_road_z_offset_cm": TERRAIN_ROAD_Z_OFFSET_CM,
    "road_surface_z_offset_cm": ROAD_SURFACE_Z_OFFSET_CM,
    "terrain_road_max_component_area_ratio": (
        TERRAIN_ROAD_MAX_COMPONENT_AREA_RATIO
    ),
    "keep_generated_vegetation": KEEP_GENERATED_VEGETATION,
    "remove_dtt_generated_vegetation": REMOVE_DTT_GENERATED_VEGETATION,
    "prune_vegetation_in_driving_lanes": (
        PRUNE_VEGETATION_IN_DRIVING_LANES
    ),
    "driving_lane_samples_path": DRIVING_LANE_SAMPLES_PATH,
    "prune_streetlights_in_driving_lanes": (
        PRUNE_STREETLIGHTS_IN_DRIVING_LANES
    ),
    "remove_procedural_streetlights": REMOVE_PROCEDURAL_STREETLIGHTS,
    "place_official_streetlights": PLACE_OFFICIAL_STREETLIGHTS,
    "place_official_traffic_controls": PLACE_OFFICIAL_TRAFFIC_CONTROLS,
    "streetlight_lane_clearance_m": STREETLIGHT_LANE_CLEARANCE_M,
    "streetlight_deep_intrusion_m": STREETLIGHT_DEEP_INTRUSION_M,
    "prune_traffic_controls_by_osm": PRUNE_TRAFFIC_CONTROLS_BY_OSM,
    "traffic_signal_plan_path": TRAFFIC_SIGNAL_PLAN_PATH,
    "traffic_control_osm_match_radius_m": (
        TRAFFIC_CONTROL_OSM_MATCH_RADIUS_M
    ),
    "dtt_tile_size_m": DTT_TILE_SIZE_M,
    "tile_offset_m": {
        "x": TILE_OFFSET_X_M,
        "y": TILE_OFFSET_Y_M,
    },
    "dry_run": DRY_RUN,
    "started_at": time.time(),
    "success": False,
}


def log(message):
    unreal.log("[MOSAIC_MAP_LAYER_CLEANUP] " + str(message))


def package_path(value):
    return str(value).split(".", 1)[0]


def safe_name(obj):
    try:
        return str(obj.get_name())
    except Exception:
        return ""


def safe_path(obj):
    try:
        return str(obj.get_path_name())
    except Exception:
        return ""


def safe_actor_label(actor):
    try:
        return str(actor.get_actor_label())
    except Exception:
        return ""


def safe_actor_class_path(actor):
    try:
        return str(actor.get_class().get_path_name())
    except Exception:
        return ""


def ensure_actor_tag(actor, tag):
    try:
        tags = list(actor.get_editor_property("tags") or [])
        if any(str(value) == tag for value in tags):
            return True
        actor.modify()
        tags.append(unreal.Name(tag))
        actor.set_editor_property("tags", tags)
        return any(
            str(value) == tag
            for value in list(actor.get_editor_property("tags") or [])
        )
    except Exception:
        return False


def actor_has_light_component(actor):
    """Return whether an actor owns a Light component.

    LightComponent ownership supplies diagnostic metadata. Streetlight
    classification and deletion require an explicit asset or class match.
    """
    for class_name in (
        "PointLightComponent",
        "SpotLightComponent",
        "RectLightComponent",
        "LightComponent",
    ):
        component_class = getattr(unreal, class_name, None)
        if component_class is None:
            continue
        try:
            if actor.get_components_by_class(component_class):
                return True
        except Exception:
            continue
    return False


def save_report():
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(
        json.dumps(STATE, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def classify_context(mesh_path, actor_path, actor_label, component_name):
    return classify_semantic_context(
        mesh_path,
        actor_path,
        actor_label,
        component_name,
    )


def actor_location_xy_m(actor, apply_tile_offset=True):
    try:
        location = actor.get_actor_location()
        x = float(location.x) / 100.0
        y = float(location.y) / 100.0
    except Exception:
        return None
    if apply_tile_offset:
        x += TILE_OFFSET_X_M
        y += TILE_OFFSET_Y_M
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return x, y


def is_streetlight_actor(actor_info):
    context = " ".join(
        [
            str(actor_info.get("actor_path", "")),
            str(actor_info.get("actor_label", "")),
        ]
        + [str(item) for item in actor_info.get("mesh_samples", [])]
    ).lower()
    return any(
        token in context
        for token in (
            "streetlight", "street_light", "street light",
            "streetlamp", "street_lamp", "street lamp",
            "lamppost", "lamp_post", "lamp post",
            "lightpole", "light_pole", "light pole",
            "/streetlights/", "/streetlight/", "/lamp/",
        )
    )


def is_traffic_control_actor(actor_info):
    context = " ".join(
        [
            str(actor_info.get("actor_path", "")),
            str(actor_info.get("actor_label", "")),
        ]
        + [str(item) for item in actor_info.get("mesh_samples", [])]
    ).lower()
    return is_traffic_control_context(context)


def actor_context_text(actor_info):
    return normalized_context(
        actor_info.get("actor_path", ""),
        actor_info.get("actor_class_path", ""),
        actor_info.get("actor_label", ""),
        *actor_info.get("mesh_samples", []),
    )


def is_osm_streetlamp_actor(actor_info):
    return is_osm_streetlamp(
        actor_info.get("actor_label", ""),
        actor_info.get("actor_path", ""),
    )


def building_night_lighting_classification(actor_info, tags):
    return classify_building_lighting(
        actor_context_text(actor_info),
        has_light_component=bool(actor_info.get("has_light_component")),
        semantic_tags=tags,
    )


def is_building_night_lighting(actor_info, tags):
    return bool(building_night_lighting_classification(actor_info, tags))


def is_dtt_official_streetlight(actor_info, tags):
    if is_osm_streetlamp_actor(actor_info):
        return False
    if is_traffic_control_actor(actor_info):
        return False
    if is_building_night_lighting(actor_info, tags):
        return False
    return is_procedural_streetlight_context(actor_context_text(actor_info))


def lighting_classification(actor_info, tags):
    if is_osm_streetlamp_actor(actor_info):
        return "osm_streetlight"
    building_class = building_night_lighting_classification(actor_info, tags)
    if building_class:
        return building_class
    if is_dtt_official_streetlight(actor_info, tags):
        return "dtt_official_streetlight"
    if bool(actor_info.get("has_light_component")):
        return "unclassified_light_component"
    return ""


def load_driving_lane_samples():
    if (
        not PRUNE_STREETLIGHTS_IN_DRIVING_LANES
        and not PRUNE_VEGETATION_IN_DRIVING_LANES
    ):
        return [], {}, 10.0
    if not DRIVING_LANE_SAMPLES_PATH:
        raise RuntimeError(
            "MOSAIC_PRUNE_DRIVING_LANE_SAMPLES is required when "
            "streetlight lane pruning is enabled"
        )

    path = Path(DRIVING_LANE_SAMPLES_PATH)
    payload = json.loads(path.read_text(encoding="utf-8"))
    samples = payload.get("samples", [])
    if not samples:
        raise RuntimeError(
            "Driving lane sample file contains no samples: {}".format(path)
        )

    cell_size_m = 10.0
    grid = {}
    normalized = []
    for sample in samples:
        try:
            item = {
                "x": float(sample["x"]),
                "y": float(sample["y"]),
                "lane_width_m": float(sample["lane_width_m"]),
                "road_id": int(sample["road_id"]),
                "section_id": int(sample["section_id"]),
                "lane_id": int(sample["lane_id"]),
                "s": float(sample["s"]),
            }
        except (KeyError, TypeError, ValueError):
            continue
        if item["lane_width_m"] <= 0.0:
            continue
        index = len(normalized)
        normalized.append(item)
        key = (
            int(math.floor(item["x"] / cell_size_m)),
            int(math.floor(item["y"] / cell_size_m)),
        )
        grid.setdefault(key, []).append(index)

    if not normalized:
        raise RuntimeError(
            "Driving lane sample file contains no valid samples: {}".format(
                path
            )
        )
    return normalized, grid, cell_size_m


def load_osm_traffic_signal_points():
    if not PRUNE_TRAFFIC_CONTROLS_BY_OSM:
        return [], {}, max(1.0, TRAFFIC_CONTROL_OSM_MATCH_RADIUS_M)
    if not TRAFFIC_SIGNAL_PLAN_PATH:
        raise RuntimeError(
            "MOSAIC_PRUNE_TRAFFIC_SIGNAL_PLAN is required when "
            "traffic-control OSM pruning is enabled"
        )

    path = Path(TRAFFIC_SIGNAL_PLAN_PATH)
    payload = json.loads(path.read_text(encoding="utf-8"))
    raw_items = payload.get("planned_traffic_signals", []) or []
    cell_size_m = max(1.0, TRAFFIC_CONTROL_OSM_MATCH_RADIUS_M)
    points = []
    grid = {}
    for item in raw_items:
        for mode, x_key, y_key in (
            ("opendrive", "opendrive_x_m", "opendrive_y_m"),
            ("ue", "ue_x_m", "ue_y_m"),
        ):
            try:
                x = float(item[x_key])
                y = float(item[y_key])
            except (KeyError, TypeError, ValueError):
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            point = {
                "osm_node_id": str(item.get("osm_node_id", "")),
                "kind": str(item.get("kind", "")),
                "coordinate_mode": mode,
                "x_m": x,
                "y_m": y,
                "latitude": item.get("latitude"),
                "longitude": item.get("longitude"),
                "tags": item.get("tags", {}),
            }
            index = len(points)
            points.append(point)
            key = (
                int(math.floor(x / cell_size_m)),
                int(math.floor(y / cell_size_m)),
            )
            grid.setdefault(key, []).append(index)
    return points, grid, cell_size_m


def _nearest_osm_traffic_signal_at_xy(
        x, y, points, grid, cell_size_m):
    if not points:
        return None
    radius_m = max(0.0, TRAFFIC_CONTROL_OSM_MATCH_RADIUS_M)
    cell_x = int(math.floor(x / cell_size_m))
    cell_y = int(math.floor(y / cell_size_m))
    cell_radius = int(math.ceil(radius_m / cell_size_m)) + 1
    best = None
    for offset_x in range(-cell_radius, cell_radius + 1):
        for offset_y in range(-cell_radius, cell_radius + 1):
            for index in grid.get(
                    (cell_x + offset_x, cell_y + offset_y), []):
                point = points[index]
                distance_m = math.hypot(x - point["x_m"], y - point["y_m"])
                if distance_m > radius_m:
                    continue
                if best is None or distance_m < best["distance_m"]:
                    best = dict(point)
                    best["distance_m"] = distance_m
                    best["actor_x_m"] = x
                    best["actor_y_m"] = y
    return best


def traffic_control_osm_match(
        actor, points, grid, cell_size_m):
    primary = actor_location_xy_m(actor, apply_tile_offset=True)
    if primary is not None:
        match = _nearest_osm_traffic_signal_at_xy(
            primary[0], primary[1], points, grid, cell_size_m
        )
        if match is not None:
            match["actor_coordinate_mode"] = "tile_offset"
            return match

    raw = actor_location_xy_m(actor, apply_tile_offset=False)
    if raw is None or raw == primary:
        return None
    match = _nearest_osm_traffic_signal_at_xy(
        raw[0], raw[1], points, grid, cell_size_m
    )
    if match is not None:
        match["actor_coordinate_mode"] = "raw_global"
        return match
    return None


def _lane_intrusion_at_xy(
        x, y, lane_samples, lane_grid, cell_size_m):
    cell_x = int(math.floor(x / cell_size_m))
    cell_y = int(math.floor(y / cell_size_m))
    best = None
    for offset_x in (-1, 0, 1):
        for offset_y in (-1, 0, 1):
            for index in lane_grid.get(
                    (cell_x + offset_x, cell_y + offset_y), []):
                sample = lane_samples[index]
                distance_m = math.hypot(
                    x - sample["x"],
                    y - sample["y"],
                )
                interior_radius_m = max(
                    0.0,
                    sample["lane_width_m"] / 2.0
                    - STREETLIGHT_LANE_CLEARANCE_M,
                )
                intrusion_m = interior_radius_m - distance_m
                if intrusion_m < 0.0:
                    continue
                if best is None or intrusion_m > best["intrusion_m"]:
                    best = {
                        "actor_x_m": x,
                        "actor_y_m": y,
                        "nearest_waypoint_x_m": sample["x"],
                        "nearest_waypoint_y_m": sample["y"],
                        "lane_center_distance_m": distance_m,
                        "lane_width_m": sample["lane_width_m"],
                        "lane_interior_radius_m": interior_radius_m,
                        "intrusion_m": intrusion_m,
                        "road_id": sample["road_id"],
                        "section_id": sample["section_id"],
                        "lane_id": sample["lane_id"],
                        "s": sample["s"],
                    }
    return best


def streetlight_lane_intrusion(
        actor, lane_samples, lane_grid, cell_size_m):
    # Primary path preserves the DTT tile-offset convention already used by
    # this cleanup.  A second raw-coordinate check catches DTT Blueprint
    # variants whose actor transforms are already stored in global map space.
    # The raw fallback is deliberately required to be deeply inside a Driving
    # lane, preventing a legitimate roadside lamp from being removed because
    # of an ambiguous tile transform.
    primary = actor_location_xy_m(actor, apply_tile_offset=True)
    if primary is not None:
        intrusion = _lane_intrusion_at_xy(
            primary[0], primary[1], lane_samples, lane_grid, cell_size_m
        )
        if intrusion is not None:
            intrusion["coordinate_mode"] = "tile_offset"
            return intrusion

    raw = actor_location_xy_m(actor, apply_tile_offset=False)
    if raw is None or raw == primary:
        return None
    intrusion = _lane_intrusion_at_xy(
        raw[0], raw[1], lane_samples, lane_grid, cell_size_m
    )
    if (
        intrusion is not None
        and float(intrusion.get("intrusion_m", 0.0))
        >= STREETLIGHT_DEEP_INTRUSION_M
    ):
        intrusion["coordinate_mode"] = "raw_global_deep_fallback"
        return intrusion
    return None


def vegetation_lane_intrusion(
        actor, lane_samples, lane_grid, cell_size_m):
    return streetlight_lane_intrusion(
        actor, lane_samples, lane_grid, cell_size_m
    )


def vegetation_instance_lane_intrusion(
        location, lane_samples, lane_grid, cell_size_m):
    try:
        raw_x = float(location.x) / 100.0
        raw_y = float(location.y) / 100.0
    except Exception:
        return None
    if not math.isfinite(raw_x) or not math.isfinite(raw_y):
        return None

    primary = (raw_x + TILE_OFFSET_X_M, raw_y + TILE_OFFSET_Y_M)
    intrusion = _lane_intrusion_at_xy(
        primary[0], primary[1], lane_samples, lane_grid, cell_size_m
    )
    if intrusion is not None:
        intrusion["coordinate_mode"] = "tile_offset"
        return intrusion

    if primary == (raw_x, raw_y):
        return None
    intrusion = _lane_intrusion_at_xy(
        raw_x, raw_y, lane_samples, lane_grid, cell_size_m
    )
    if (
        intrusion is not None
        and float(intrusion.get("intrusion_m", 0.0))
        >= STREETLIGHT_DEEP_INTRUSION_M
    ):
        intrusion["coordinate_mode"] = "raw_global_deep_fallback"
        return intrusion
    return None


def actor_bounds_xy(actor):
    try:
        origin, extent = actor.get_actor_bounds(False)
    except Exception:
        return None

    try:
        center_x = float(origin.x)
        center_y = float(origin.y)
        extent_x = abs(float(extent.x))
        extent_y = abs(float(extent.y))
    except Exception:
        return None

    if not all(
        math.isfinite(value)
        for value in (center_x, center_y, extent_x, extent_y)
    ):
        return None
    if extent_x <= 0.0 or extent_y <= 0.0:
        return None

    return (
        center_x - extent_x,
        center_x + extent_x,
        center_y - extent_y,
        center_y + extent_y,
        center_x,
        center_y,
    )


def point_inside_xy(x, y, bounds, margin_cm):
    min_x, max_x, min_y, max_y, _center_x, _center_y = bounds
    return (
        min_x - margin_cm <= x <= max_x + margin_cm
        and min_y - margin_cm <= y <= max_y + margin_cm
    )


def bounds_overlap_xy(first, second, margin_cm=0.0):
    first_min_x, first_max_x, first_min_y, first_max_y, _x, _y = first
    second_min_x, second_max_x, second_min_y, second_max_y, _x2, _y2 = second
    return not (
        first_max_x < second_min_x - margin_cm
        or first_min_x > second_max_x + margin_cm
        or first_max_y < second_min_y - margin_cm
        or first_min_y > second_max_y + margin_cm
    )


def bounds_area_xy(bounds):
    min_x, max_x, min_y, max_y, _x, _y = bounds
    return max(0.0, max_x - min_x) * max(0.0, max_y - min_y)


def bounds_overlap_area_xy(first, second, margin_cm=0.0):
    first_min_x, first_max_x, first_min_y, first_max_y, _x, _y = first
    second_min_x, second_max_x, second_min_y, second_max_y, _x2, _y2 = second
    overlap_x = min(first_max_x, second_max_x + margin_cm) - max(
        first_min_x,
        second_min_x - margin_cm,
    )
    overlap_y = min(first_max_y, second_max_y + margin_cm) - max(
        first_min_y,
        second_min_y - margin_cm,
    )
    return max(0.0, overlap_x) * max(0.0, overlap_y)


def bounds_dict(bounds):
    return {
        "min_x": bounds[0],
        "max_x": bounds[1],
        "min_y": bounds[2],
        "max_y": bounds[3],
        "center_x": bounds[4],
        "center_y": bounds[5],
    }


def copy_vector(value):
    return unreal.Vector(
        float(value.x),
        float(value.y),
        float(value.z),
    )


def component_bounds_xy(component):
    if component is None:
        return None

    bounds = None
    for attr_name in ("bounds", "Bounds"):
        try:
            bounds = getattr(component, attr_name)
        except Exception:
            bounds = None
        if bounds is not None:
            break

    if bounds is None:
        try:
            bounds = component.get_editor_property("bounds")
        except Exception:
            bounds = None

    if bounds is None:
        return None

    origin = None
    extent = None
    for attr_name in ("origin", "Origin"):
        try:
            origin = getattr(bounds, attr_name)
        except Exception:
            origin = None
        if origin is not None:
            break
    for attr_name in ("box_extent", "BoxExtent"):
        try:
            extent = getattr(bounds, attr_name)
        except Exception:
            extent = None
        if extent is not None:
            break

    if origin is None or extent is None:
        return None

    try:
        center_x = float(origin.x)
        center_y = float(origin.y)
        extent_x = abs(float(extent.x))
        extent_y = abs(float(extent.y))
    except Exception:
        return None

    if not all(
        math.isfinite(value)
        for value in (center_x, center_y, extent_x, extent_y)
    ):
        return None
    if extent_x <= 0.0 or extent_y <= 0.0:
        return None

    return (
        center_x - extent_x,
        center_x + extent_x,
        center_y - extent_y,
        center_y + extent_y,
        center_x,
        center_y,
    )


def move_actor_z(actor, offset_cm):
    try:
        location = actor.get_actor_location()
    except Exception:
        return False
    try:
        location.z = float(location.z) + float(offset_cm)
        return bool(actor.set_actor_location(location, False, True))
    except TypeError:
        try:
            actor.set_actor_location(location, False)
            return True
        except Exception:
            return False
    except Exception:
        return False


def move_component_z(component, offset_cm):
    if component is None:
        return False

    try:
        component.modify()
    except Exception:
        pass
    try:
        owner = component.get_owner()
        if owner is not None:
            owner.modify()
    except Exception:
        pass

    for getter_name, setter_name in (
        ("get_relative_location", "set_relative_location"),
        ("get_component_location", "set_world_location"),
        ("get_world_location", "set_world_location"),
    ):
        getter = getattr(component, getter_name, None)
        setter = getattr(component, setter_name, None)
        if getter is None or setter is None:
            continue
        try:
            location = copy_vector(getter())
            location.z = float(location.z) + float(offset_cm)
        except Exception:
            continue
        try:
            setter(location, False, None, False)
            return True
        except TypeError:
            try:
                setter(location, False)
                return True
            except Exception:
                pass
        except Exception:
            pass

    try:
        location = copy_vector(component.get_editor_property("relative_location"))
        location.z = float(location.z) + float(offset_cm)
        component.set_editor_property("relative_location", location)
        return True
    except Exception:
        return False


def static_mesh_component_records(actor, component_classes):
    actor_path = safe_path(actor)
    actor_label = safe_actor_label(actor)
    records = []
    seen = set()

    for component_class in component_classes:
        if component_class is None:
            continue
        try:
            components = actor.get_components_by_class(component_class)
        except Exception:
            components = []

        for component in components:
            component_path = safe_path(component)
            component_key = component_path or ("object", id(component))
            if component_key in seen:
                continue
            seen.add(component_key)

            component_name = safe_name(component)
            try:
                mesh = component.get_editor_property("static_mesh")
            except Exception:
                mesh = None
            mesh_path = safe_path(mesh) if mesh is not None else ""
            tag = classify_context(
                mesh_path,
                actor_path,
                actor_label,
                component_name,
            )
            if tag is None:
                continue
            records.append(
                {
                    "component": component,
                    "component_name": component_name,
                    "component_path": component_path,
                    "mesh_path": mesh_path,
                    "tag": tag,
                }
            )

    if not records:
        fallback = classify_context("", actor_path, actor_label, "")
        if fallback is not None:
            records.append(
                {
                    "component": None,
                    "component_name": "",
                    "component_path": "",
                    "mesh_path": "",
                    "tag": fallback,
                }
            )

    return records


def actor_component_tags(actor, component_classes):
    records = static_mesh_component_records(actor, component_classes)
    return (
        [record["tag"] for record in records],
        [
            record["mesh_path"]
            for record in records
            if record.get("mesh_path")
        ],
    )


def instanced_component_classes():
    classes = []
    for name in (
        "InstancedStaticMeshComponent",
        "HierarchicalInstancedStaticMeshComponent",
    ):
        component_class = getattr(unreal, name, None)
        if component_class is not None:
            classes.append(component_class)
    return classes


def component_instance_count(component):
    for method_name in ("get_instance_count", "get_num_instances"):
        method = getattr(component, method_name, None)
        if method is None:
            continue
        try:
            return int(method())
        except Exception:
            pass
    return 0


def instance_world_location(component, index):
    try:
        transform = component.get_instance_transform(index, True)
    except Exception:
        return None

    for attr_name in ("translation", "location"):
        try:
            value = getattr(transform, attr_name)
        except Exception:
            value = None
        if value is not None:
            return value

    try:
        return transform.get_translation()
    except Exception:
        return None


def remove_component_instance(component, index):
    method = getattr(component, "remove_instance", None)
    if method is None:
        return False
    try:
        return bool(method(index))
    except Exception:
        return False


def normalize_material_path(value):
    value = str(value).strip()
    if not value:
        return ""
    if "." in value:
        return value
    asset_name = value.rsplit("/", 1)[-1]
    return value + "." + asset_name


def load_material_asset(value):
    material_path = normalize_material_path(value)
    if not material_path:
        return None
    try:
        return unreal.EditorAssetLibrary.load_asset(material_path)
    except Exception:
        return None


def component_material_slot_count(component):
    for method_name in ("get_num_materials", "get_material_slot_names"):
        method = getattr(component, method_name, None)
        if method is None:
            continue
        try:
            result = method()
        except Exception:
            continue
        if isinstance(result, int):
            return max(0, result)
        try:
            return len(result)
        except Exception:
            pass
    return 1


def assign_component_material(component, material):
    if component is None or material is None:
        return 0
    count = component_material_slot_count(component)
    changed = 0
    for index in range(max(1, count)):
        try:
            component.set_material(index, material)
            changed += 1
        except Exception:
            pass
    return changed


def save_current_level():
    try:
        if unreal.EditorLevelLibrary.save_current_level():
            return True
    except Exception:
        pass
    try:
        return bool(unreal.EditorLevelLibrary.save_all_dirty_levels())
    except Exception:
        return False


def run():
    save_report()

    world = unreal.EditorLevelLibrary.get_editor_world()
    if world is None:
        raise RuntimeError("Editor world is unavailable")

    loaded_world = package_path(world.get_path_name())
    STATE["loaded_world_package"] = loaded_world
    if loaded_world.lower() != TARGET_LEVEL.lower():
        raise RuntimeError(
            "Loaded Level mismatch: expected {}, got {}".format(
                TARGET_LEVEL,
                loaded_world,
            )
        )

    component_class = getattr(unreal, "StaticMeshComponent", None)
    instance_component_classes = instanced_component_classes()
    component_classes = [component_class] + instance_component_classes
    if component_class is None:
        raise RuntimeError("unreal.StaticMeshComponent is unavailable")

    lane_samples, lane_grid, lane_grid_cell_size_m = (
        load_driving_lane_samples()
    )
    traffic_signal_points, traffic_signal_grid, traffic_signal_grid_cell_size_m = (
        load_osm_traffic_signal_points()
    )
    actors = list(unreal.EditorLevelLibrary.get_all_level_actors())
    buildings = []
    vegetation_candidates = []
    streetlight_candidates = []
    traffic_control_candidates = []
    terrain_components = []
    terrain_component_candidates = []
    terrain_candidates = []
    road_surface_actors = []
    road_surface_components = []
    road_surface_component_bounds = []
    class_counts = Counter()
    examples = []
    building_lighting_candidates = []

    for actor in actors:
        actor_path = safe_path(actor)
        actor_label = safe_actor_label(actor)
        actor_class_path = safe_actor_class_path(actor)
        has_light_component = actor_has_light_component(actor)
        tags, mesh_paths = actor_component_tags(actor, component_classes)

        actor_info = {
            "actor_path": actor_path,
            "actor_label": actor_label,
            "bounds_xy_cm": {},
            "mesh_samples": mesh_paths[:5],
            "actor_class_path": actor_class_path,
            "has_light_component": bool(has_light_component),
            "streetlight_by_name": bool(is_streetlight_actor({
                "actor_path": "{} {}".format(actor_path, actor_class_path),
                "actor_label": actor_label,
                "mesh_samples": mesh_paths[:5],
            })),
            "traffic_control_by_name": bool(is_traffic_control_actor({
                "actor_path": "{} {}".format(actor_path, actor_class_path),
                "actor_label": actor_label,
                "mesh_samples": mesh_paths[:5],
            })),
        }
        light_class = lighting_classification(actor_info, tags)
        actor_info["lighting_classification"] = light_class
        actor_info["building_lighting_candidate"] = bool(
            building_night_lighting_classification(actor_info, tags)
        )

        if light_class == "dtt_official_streetlight" and "Pole" not in tags:
            tags.append("Pole")

        if (
            not tags
            and not actor_info.get("traffic_control_by_name")
            and not light_class
        ):
            continue

        tag_set = set(tags)
        for tag in tag_set:
            class_counts[tag] += 1

        bounds = actor_bounds_xy(actor)
        if (
            bounds is None
            and light_class != "dtt_official_streetlight"
            and not bool(actor_info.get("traffic_control_by_name"))
            and not bool(actor_info.get("building_lighting_candidate"))
        ):
            continue

        actor_info["bounds_xy_cm"] = (
            {}
            if bounds is None
            else {
                "min_x": bounds[0],
                "max_x": bounds[1],
                "min_y": bounds[2],
                "max_y": bounds[3],
                "center_x": bounds[4],
                "center_y": bounds[5],
            }
        )

        if "Building" in tag_set:
            buildings.append((actor, bounds, actor_info))

        if tag_set & {"Road", "RoadLine", "SideWalk"}:
            road_surface_actors.append((actor, bounds, actor_info, tag_set))
            for record in static_mesh_component_records(
                actor,
                component_classes,
            ):
                if record.get("tag") in {"Road", "RoadLine", "SideWalk"}:
                    record_bounds = component_bounds_xy(record.get("component"))
                    if record_bounds is not None:
                        road_surface_components.append(
                            (actor, record, record_bounds, actor_info)
                        )
                        road_surface_component_bounds.append(record_bounds)

        if "Terrain" in tag_set:
            for record in static_mesh_component_records(
                actor,
                component_classes,
            ):
                if record.get("tag") == "Terrain":
                    terrain_components.append((actor, record, actor_info))
                    record_bounds = component_bounds_xy(record.get("component"))
                    if record_bounds is not None:
                        terrain_component_candidates.append(
                            (actor, record, record_bounds, actor_info)
                        )

        # Move pure terrain actors while preserving the positions of mixed
        # actors containing road or building geometry.
        if tag_set == {"Terrain"}:
            terrain_candidates.append((actor, bounds, actor_info))

        # Select pure vegetation actors for deletion while preserving mixed
        # actors containing road or building geometry.
        if tag_set == {"Vegetation"}:
            vegetation_candidates.append((actor, bounds, actor_info))

        if bool(actor_info.get("building_lighting_candidate")):
            building_lighting_candidates.append((actor, bounds, actor_info))

        # Streetlight deletion requires an explicit official DTT streetlight
        # asset or class match. LightComponent ownership supplies diagnostics.
        if light_class == "dtt_official_streetlight":
            streetlight_candidates.append((actor, bounds, actor_info))
        if bool(actor_info.get("traffic_control_by_name")):
            traffic_control_candidates.append((actor, bounds, actor_info))

    building_bounds = [item[1] for item in buildings if item[1] is not None]
    actors_to_delete = []
    dtt_vegetation_delete_actor_count = 0
    vegetation_lane_overlap_actor_count = 0
    vegetation_building_overlap_actor_count = 0

    for actor, bounds, actor_info in vegetation_candidates:
        actor_info = dict(actor_info)
        center_x = bounds[4]
        center_y = bounds[5]
        building_overlap = any(
            point_inside_xy(
                center_x,
                center_y,
                building_bounds_item,
                PRUNE_MARGIN_CM,
            )
            for building_bounds_item in building_bounds
        )
        lane_intrusion = (
            vegetation_lane_intrusion(
                actor,
                lane_samples,
                lane_grid,
                lane_grid_cell_size_m,
            )
            if PRUNE_VEGETATION_IN_DRIVING_LANES
            else None
        )
        if lane_intrusion is not None:
            actor_info["driving_lane_intrusion"] = lane_intrusion
            vegetation_lane_overlap_actor_count += 1
        if building_overlap:
            vegetation_building_overlap_actor_count += 1

        if REMOVE_DTT_GENERATED_VEGETATION:
            actor_info["delete_reason"] = "dtt_generated_vegetation_cleanup"
            dtt_vegetation_delete_actor_count += 1
        elif KEEP_GENERATED_VEGETATION:
            continue
        elif lane_intrusion is not None:
            actor_info["delete_reason"] = "vegetation_intrudes_driving_lane"
        elif building_overlap:
            actor_info["delete_reason"] = "vegetation_overlaps_building"
        else:
            continue

        actors_to_delete.append((actor, actor_info))
        if len(examples) < 50:
            examples.append(actor_info)

    vegetation_actor_delete_paths = {
        safe_path(actor) for actor, _actor_info in actors_to_delete
    }

    streetlights_to_delete = []
    streetlights_to_tag = []
    streetlight_examples = []
    streetlight_template_candidates = []
    kept_dtt_official_streetlight_count = 0
    streetlight_lane_overlap_actor_count = 0
    for actor, _bounds, actor_info in streetlight_candidates:
        actor_info = dict(actor_info)
        intrusion = streetlight_lane_intrusion(
            actor,
            lane_samples,
            lane_grid,
            lane_grid_cell_size_m,
        )
        if intrusion is not None:
            actor_info["driving_lane_intrusion"] = intrusion
            streetlight_lane_overlap_actor_count += 1

        if REMOVE_PROCEDURAL_STREETLIGHTS:
            actor_info["delete_reason"] = (
                "procedural_streetlight_not_osm_authoritative"
            )
            streetlights_to_delete.append((actor, actor_info))
        else:
            # Preserve the complete actor once. Multimodal Stage 1 hides its
            # mesh/collision by default while retaining the light source.
            actor_info["delete_reason"] = "kept_for_runtime_streetlight_mode"
            streetlights_to_tag.append((actor, actor_info))
            kept_dtt_official_streetlight_count += 1

        if len(streetlight_examples) < 100:
            streetlight_examples.append(actor_info)
        if len(streetlight_template_candidates) < 50:
            streetlight_template_candidates.append({
                "actor_class_path": actor_info.get("actor_class_path", ""),
                "actor_path": actor_info.get("actor_path", ""),
                "actor_label": actor_info.get("actor_label", ""),
                "mesh_samples": list(actor_info.get("mesh_samples", []) or []),
                "has_light_component": bool(actor_info.get("has_light_component")),
                "lighting_classification": actor_info.get("lighting_classification", ""),
                "building_lighting_candidate": bool(actor_info.get("building_lighting_candidate")),
                "delete_reason": actor_info.get("delete_reason", ""),
            })

    traffic_controls_to_delete = []
    traffic_control_examples = []
    for actor, _bounds, actor_info in traffic_control_candidates:
        actor_info = dict(actor_info)
        intrusion = streetlight_lane_intrusion(
            actor,
            lane_samples,
            lane_grid,
            lane_grid_cell_size_m,
        )
        osm_match = traffic_control_osm_match(
            actor,
            traffic_signal_points,
            traffic_signal_grid,
            traffic_signal_grid_cell_size_m,
        )
        if intrusion is not None:
            actor_info["driving_lane_intrusion"] = intrusion
            actor_info["delete_reason"] = "traffic_control_intrudes_driving_lane"
        elif PRUNE_TRAFFIC_CONTROLS_BY_OSM and osm_match is None:
            actor_info["delete_reason"] = "traffic_control_not_osm_authoritative"
        else:
            if osm_match is not None:
                actor_info["nearest_osm_traffic_signal"] = osm_match
            continue
        if osm_match is not None:
            actor_info["nearest_osm_traffic_signal"] = osm_match
        traffic_controls_to_delete.append((actor, actor_info))
        if len(traffic_control_examples) < 100:
            traffic_control_examples.append(actor_info)

    building_lighting_examples = []
    for _actor, _bounds, actor_info in building_lighting_candidates[:100]:
        actor_info = dict(actor_info)
        actor_info["delete_reason"] = "protected_building_night_lighting"
        building_lighting_examples.append(actor_info)

    deleted_count = 0
    failed_delete_count = 0
    deleted_streetlight_count = 0
    failed_streetlight_delete_count = 0
    tagged_streetlight_count = 0
    failed_streetlight_tag_count = 0
    deleted_traffic_control_count = 0
    failed_traffic_control_delete_count = 0
    if not DRY_RUN:
        for actor, _actor_info in actors_to_delete:
            try:
                if unreal.EditorLevelLibrary.destroy_actor(actor):
                    deleted_count += 1
                else:
                    failed_delete_count += 1
            except Exception:
                failed_delete_count += 1
        for actor, _actor_info in streetlights_to_delete:
            try:
                if unreal.EditorLevelLibrary.destroy_actor(actor):
                    deleted_streetlight_count += 1
                else:
                    failed_streetlight_delete_count += 1
            except Exception:
                failed_streetlight_delete_count += 1
        for actor, _actor_info in streetlights_to_tag:
            if ensure_actor_tag(actor, DTT_STREETLIGHT_RUNTIME_TAG):
                tagged_streetlight_count += 1
            else:
                failed_streetlight_tag_count += 1
        for actor, _actor_info in traffic_controls_to_delete:
            try:
                if unreal.EditorLevelLibrary.destroy_actor(actor):
                    deleted_traffic_control_count += 1
                else:
                    failed_traffic_control_delete_count += 1
            except Exception:
                failed_traffic_control_delete_count += 1

    instanced_component_count = 0
    instanced_vegetation_component_count = 0
    overlapping_instance_count = 0
    deleted_instance_count = 0
    failed_delete_instance_count = 0
    instance_examples = []
    dtt_vegetation_delete_instance_count = 0
    vegetation_lane_overlap_instance_count = 0
    vegetation_building_overlap_instance_count = 0

    for actor in actors:
        if safe_path(actor) in vegetation_actor_delete_paths:
            continue
        actor_records = static_mesh_component_records(
            actor,
            instance_component_classes,
        )
        for record in actor_records:
            component = record.get("component")
            if component is None:
                continue
            instanced_component_count += 1
            if record.get("tag") != "Vegetation":
                continue
            instanced_vegetation_component_count += 1
            instance_count = component_instance_count(component)
            if instance_count <= 0:
                continue

            remove_indices = []
            for index in range(instance_count):
                location = instance_world_location(component, index)
                if location is None:
                    continue
                try:
                    x = float(location.x)
                    y = float(location.y)
                except Exception:
                    continue

                building_overlap = any(
                    point_inside_xy(
                        x,
                        y,
                        building_bounds_item,
                        PRUNE_MARGIN_CM,
                    )
                    for building_bounds_item in building_bounds
                )
                lane_intrusion = (
                    vegetation_instance_lane_intrusion(
                        location,
                        lane_samples,
                        lane_grid,
                        lane_grid_cell_size_m,
                    )
                    if PRUNE_VEGETATION_IN_DRIVING_LANES
                    else None
                )
                if lane_intrusion is not None:
                    vegetation_lane_overlap_instance_count += 1
                if building_overlap:
                    vegetation_building_overlap_instance_count += 1

                delete_reason = ""
                if REMOVE_DTT_GENERATED_VEGETATION:
                    delete_reason = "dtt_generated_vegetation_cleanup"
                    dtt_vegetation_delete_instance_count += 1
                elif KEEP_GENERATED_VEGETATION:
                    continue
                elif lane_intrusion is not None:
                    delete_reason = "vegetation_intrudes_driving_lane"
                elif building_overlap:
                    delete_reason = "vegetation_overlaps_building"
                if not delete_reason:
                    continue

                remove_indices.append(index)
                if len(instance_examples) < 50:
                    example = {
                        "actor_path": safe_path(actor),
                        "actor_label": safe_actor_label(actor),
                        "component_path": record.get(
                            "component_path",
                            "",
                        ),
                        "component_name": record.get(
                            "component_name",
                            "",
                        ),
                        "mesh_path": record.get("mesh_path", ""),
                        "instance_index": int(index),
                        "x": x,
                        "y": y,
                        "delete_reason": delete_reason,
                    }
                    if lane_intrusion is not None:
                        example["driving_lane_intrusion"] = lane_intrusion
                    instance_examples.append(example)

            overlapping_instance_count += len(remove_indices)
            if not DRY_RUN:
                for index in sorted(remove_indices, reverse=True):
                    if remove_component_instance(component, index):
                        deleted_instance_count += 1
                    else:
                        failed_delete_instance_count += 1

    road_surface_bounds = road_surface_component_bounds or [
        item[1] for item in road_surface_actors
    ]
    terrain_bounds = [item[2] for item in terrain_component_candidates] or [
        item[1] for item in terrain_candidates
    ]
    road_terrain_overlap_components = []
    for actor, record, bounds, actor_info in road_surface_components:
        if any(
            bounds_overlap_xy(bounds, terrain_bounds_item, TERRAIN_ROAD_MARGIN_CM)
            for terrain_bounds_item in terrain_bounds
        ):
            road_terrain_overlap_components.append(
                (actor, record, bounds, actor_info)
            )

    road_terrain_overlap_actors = []
    road_layer_tags = {"Road", "RoadLine", "SideWalk"}
    for actor, bounds, actor_info, tag_set in road_surface_actors:
        if not tag_set or not tag_set.issubset(road_layer_tags):
            continue
        if any(
            bounds_overlap_xy(
                bounds,
                terrain_bounds_item,
                TERRAIN_ROAD_MARGIN_CM,
            )
            for terrain_bounds_item in terrain_bounds
        ):
            road_terrain_overlap_actors.append(
                (actor, bounds, actor_info, tag_set)
            )

    terrain_road_overlap_components = []
    skipped_large_terrain_components = []
    for actor, record, bounds, actor_info in terrain_component_candidates:
        overlap_area = sum(
            bounds_overlap_area_xy(
                bounds,
                road_bounds,
                TERRAIN_ROAD_MARGIN_CM,
            )
            for road_bounds in road_surface_bounds
        )
        if overlap_area <= 0.0:
            continue
        terrain_area = bounds_area_xy(bounds)
        if (
            overlap_area > 0.0
            and terrain_area > 0.0
            and terrain_area
            > overlap_area * TERRAIN_ROAD_MAX_COMPONENT_AREA_RATIO
        ):
            skipped_large_terrain_components.append(
                (actor, record, bounds, actor_info, terrain_area, overlap_area)
            )
            continue
        terrain_road_overlap_components.append(
            (actor, record, bounds, actor_info)
        )

    terrain_road_overlap_actors = []
    for actor, bounds, actor_info in terrain_candidates:
        if any(
            bounds_overlap_xy(
                bounds,
                road_bounds,
                TERRAIN_ROAD_MARGIN_CM,
            )
            for road_bounds in road_surface_bounds
        ):
            terrain_road_overlap_actors.append((actor, actor_info))

    skipped_large_terrain_actor_paths = {
        safe_path(item[0]) for item in skipped_large_terrain_components
    }

    terrain_lowered_count = 0
    failed_terrain_lower_count = 0
    terrain_lowered_component_count = 0
    failed_terrain_component_lower_count = 0
    road_lifted_component_count = 0
    failed_road_component_lift_count = 0
    road_lifted_actor_count = 0
    failed_road_actor_lift_count = 0
    terrain_layer_examples = []
    road_layer_examples = []
    lowered_component_paths = set()
    lowered_component_actor_paths = set()
    lifted_road_component_paths = set()
    lifted_road_component_actor_paths = set()
    if ROAD_SURFACE_Z_OFFSET_CM and not DRY_RUN:
        for actor, record, bounds, actor_info in road_terrain_overlap_components:
            component = record.get("component")
            component_path = record.get("component_path") or safe_path(component)
            component_key = component_path or "{}:{}".format(
                safe_path(actor),
                record.get("component_name", ""),
            )
            if component_key in lifted_road_component_paths:
                continue
            if move_component_z(component, ROAD_SURFACE_Z_OFFSET_CM):
                road_lifted_component_count += 1
                lifted_road_component_paths.add(component_key)
                lifted_road_component_actor_paths.add(safe_path(actor))
                if len(road_layer_examples) < 50:
                    example = dict(actor_info)
                    example.update(
                        {
                            "component_path": component_path,
                            "component_name": record.get("component_name", ""),
                            "mesh_path": record.get("mesh_path", ""),
                            "bounds_xy_cm": bounds_dict(bounds),
                            "z_offset_cm": ROAD_SURFACE_Z_OFFSET_CM,
                            "lifted_scope": "component",
                        }
                    )
                    road_layer_examples.append(example)
            else:
                failed_road_component_lift_count += 1

        for actor, bounds, actor_info, tag_set in road_terrain_overlap_actors:
            actor_path = safe_path(actor)
            if actor_path in lifted_road_component_actor_paths:
                continue
            if move_actor_z(actor, ROAD_SURFACE_Z_OFFSET_CM):
                road_lifted_actor_count += 1
                if len(road_layer_examples) < 50:
                    example = dict(actor_info)
                    example.update(
                        {
                            "semantic_tags": sorted(tag_set),
                            "bounds_xy_cm": bounds_dict(bounds),
                            "z_offset_cm": ROAD_SURFACE_Z_OFFSET_CM,
                            "lifted_scope": "actor_bounds_fallback",
                        }
                    )
                    road_layer_examples.append(example)
            else:
                failed_road_actor_lift_count += 1

    if TERRAIN_ROAD_Z_OFFSET_CM and not DRY_RUN:
        for actor, record, bounds, actor_info in terrain_road_overlap_components:
            component = record.get("component")
            component_path = record.get("component_path") or safe_path(component)
            component_key = component_path or "{}:{}".format(
                safe_path(actor),
                record.get("component_name", ""),
            )
            if component_key in lowered_component_paths:
                continue
            if move_component_z(component, TERRAIN_ROAD_Z_OFFSET_CM):
                terrain_lowered_component_count += 1
                lowered_component_paths.add(component_key)
                lowered_component_actor_paths.add(safe_path(actor))
                if len(terrain_layer_examples) < 50:
                    example = dict(actor_info)
                    example.update(
                        {
                            "component_path": component_path,
                            "component_name": record.get("component_name", ""),
                            "mesh_path": record.get("mesh_path", ""),
                            "bounds_xy_cm": bounds_dict(bounds),
                            "z_offset_cm": TERRAIN_ROAD_Z_OFFSET_CM,
                            "lowered_scope": "component",
                        }
                    )
                    terrain_layer_examples.append(example)
            else:
                failed_terrain_component_lower_count += 1

        for actor, actor_info in terrain_road_overlap_actors:
            actor_path = safe_path(actor)
            if actor_path in lowered_component_actor_paths:
                continue
            if actor_path in skipped_large_terrain_actor_paths:
                continue
            if move_actor_z(actor, TERRAIN_ROAD_Z_OFFSET_CM):
                terrain_lowered_count += 1
                if len(terrain_layer_examples) < 50:
                    example = dict(actor_info)
                    example["z_offset_cm"] = TERRAIN_ROAD_Z_OFFSET_CM
                    example["lowered_scope"] = "actor"
                    terrain_layer_examples.append(example)
            else:
                failed_terrain_lower_count += 1

    terrain_material_asset = load_material_asset(TERRAIN_MATERIAL)
    terrain_material_assignment_count = 0
    terrain_material_component_count = 0
    terrain_material_examples = []
    if TERRAIN_MATERIAL:
        if terrain_material_asset is None:
            raise RuntimeError(
                "Unable to load terrain material: {}".format(
                    TERRAIN_MATERIAL
                )
            )
        terrain_material_component_count = len(terrain_components)
        if not DRY_RUN:
            for actor, record, actor_info in terrain_components:
                component = record.get("component")
                changed = assign_component_material(
                    component,
                    terrain_material_asset,
                )
                terrain_material_assignment_count += changed
                if changed and len(terrain_material_examples) < 50:
                    terrain_material_examples.append(
                        {
                            "actor_path": safe_path(actor),
                            "actor_label": safe_actor_label(actor),
                            "component_path": record.get(
                                "component_path",
                                "",
                            ),
                            "component_name": record.get(
                                "component_name",
                                "",
                            ),
                            "mesh_path": record.get("mesh_path", ""),
                            "material": normalize_material_path(
                                TERRAIN_MATERIAL
                            ),
                            "assigned_slots": int(changed),
                            "bounds_xy_cm": actor_info.get(
                                "bounds_xy_cm",
                                {},
                            ),
                        }
                    )

    save_result = {"save_ok": True}
    if (
        actors_to_delete
        or streetlights_to_delete
        or streetlights_to_tag
        or traffic_controls_to_delete
        or overlapping_instance_count
        or road_lifted_component_count
        or road_lifted_actor_count
        or terrain_lowered_component_count
        or terrain_lowered_count
        or terrain_material_assignment_count
    ) and not DRY_RUN:
        save_result["save_ok"] = save_current_level()

    STATE.update(
        {
            "actor_count": len(actors),
            "class_actor_counts": dict(class_counts),
            "building_actor_count": len(buildings),
            "road_surface_actor_count": len(road_surface_actors),
            "road_surface_component_count": len(road_surface_component_bounds),
            "vegetation_candidate_actor_count": len(vegetation_candidates),
            "streetlight_candidate_actor_count": len(streetlight_candidates),
            "traffic_control_candidate_actor_count": len(
                traffic_control_candidates
            ),
            "osm_traffic_signal_reference_count": len(
                traffic_signal_points
            ),
            "osm_traffic_signal_plan_point_count": (
                len(traffic_signal_points) // 2
            ),
            "procedural_streetlight_policy": (
                "osm_only_remove_dtt_official_streetlights"
                if REMOVE_PROCEDURAL_STREETLIGHTS
                else "keep_and_tag_dtt_streetlights_for_runtime_modes"
            ),
            "building_night_lighting_policy": "always_keep_in_generate_save",
            "vegetation_cleanup_policy": (
                "remove_all_dtt_generated_vegetation"
                if REMOVE_DTT_GENERATED_VEGETATION
                else (
                    "keep_all_dtt_generated_vegetation"
                    if KEEP_GENERATED_VEGETATION
                    else "remove_building_and_driving_lane_intrusions_only"
                )
            ),
            "streetlight_template_candidates": streetlight_template_candidates,
            "building_lighting_candidate_actor_count": len(
                building_lighting_candidates
            ),
            "building_lighting_examples": building_lighting_examples,
            "driving_lane_sample_count": len(lane_samples),
            "terrain_candidate_actor_count": len(terrain_candidates),
            "terrain_component_count": len(terrain_components),
            "terrain_component_candidate_count": (
                len(terrain_component_candidates)
            ),
            "terrain_material": normalize_material_path(TERRAIN_MATERIAL),
            "road_surface_z_offset_cm": ROAD_SURFACE_Z_OFFSET_CM,
            "road_terrain_overlap_component_count": (
                len(road_terrain_overlap_components)
            ),
            "road_terrain_overlap_actor_count": (
                len(road_terrain_overlap_actors)
            ),
            "road_lifted_component_count": road_lifted_component_count,
            "road_lifted_actor_count": road_lifted_actor_count,
            "failed_road_component_lift_count": (
                failed_road_component_lift_count
            ),
            "failed_road_actor_lift_count": failed_road_actor_lift_count,
            "road_layer_examples": road_layer_examples,
            "terrain_material_component_count": (
                terrain_material_component_count
            ),
            "terrain_material_assignment_count": (
                terrain_material_assignment_count
            ),
            "terrain_material_examples": terrain_material_examples,
            "terrain_road_margin_cm": TERRAIN_ROAD_MARGIN_CM,
            "terrain_road_z_offset_cm": TERRAIN_ROAD_Z_OFFSET_CM,
            "terrain_road_overlap_actor_count": (
                len(terrain_road_overlap_actors)
            ),
            "terrain_road_overlap_component_count": (
                len(terrain_road_overlap_components)
            ),
            "skipped_large_terrain_component_count": (
                len(skipped_large_terrain_components)
            ),
            "skipped_large_terrain_actor_count": (
                len(skipped_large_terrain_actor_paths)
            ),
            "terrain_lowered_component_count": (
                terrain_lowered_component_count
            ),
            "failed_terrain_component_lower_count": (
                failed_terrain_component_lower_count
            ),
            "terrain_lowered_actor_count": terrain_lowered_count,
            "failed_terrain_lower_count": failed_terrain_lower_count,
            "terrain_layer_examples": terrain_layer_examples,
            "overlapping_vegetation_actor_count": len(actors_to_delete),
            "vegetation_delete_target_actor_count": len(actors_to_delete),
            "dtt_vegetation_delete_actor_count": (
                dtt_vegetation_delete_actor_count
            ),
            "vegetation_lane_overlap_actor_count": (
                vegetation_lane_overlap_actor_count
            ),
            "vegetation_building_overlap_actor_count": (
                vegetation_building_overlap_actor_count
            ),
            "deleted_actor_count": deleted_count,
            "failed_delete_count": failed_delete_count,
            "streetlight_lane_overlap_actor_count": (
                streetlight_lane_overlap_actor_count
            ),
            "runtime_tagged_streetlight_actor_count": tagged_streetlight_count,
            "failed_streetlight_tag_count": failed_streetlight_tag_count,
            "kept_dtt_official_streetlight_count": (
                kept_dtt_official_streetlight_count
            ),
            "procedural_streetlight_delete_target_count": (
                len(streetlights_to_delete)
            ),
            "deleted_streetlight_actor_count": deleted_streetlight_count,
            "failed_streetlight_delete_count": (
                failed_streetlight_delete_count
            ),
            "streetlight_examples": streetlight_examples,
            "traffic_control_lane_overlap_actor_count": sum(
                1 for _actor, info in traffic_controls_to_delete
                if info.get("driving_lane_intrusion") is not None
            ),
            "traffic_control_not_osm_authoritative_count": sum(
                1 for _actor, info in traffic_controls_to_delete
                if info.get("delete_reason")
                == "traffic_control_not_osm_authoritative"
            ),
            "traffic_control_delete_target_count": len(
                traffic_controls_to_delete
            ),
            "deleted_traffic_control_actor_count": (
                deleted_traffic_control_count
            ),
            "failed_traffic_control_delete_count": (
                failed_traffic_control_delete_count
            ),
            "traffic_control_examples": traffic_control_examples,
            "instanced_component_count": instanced_component_count,
            "instanced_vegetation_component_count": (
                instanced_vegetation_component_count
            ),
            "overlapping_vegetation_instance_count": (
                overlapping_instance_count
            ),
            "vegetation_delete_target_instance_count": (
                overlapping_instance_count
            ),
            "dtt_vegetation_delete_instance_count": (
                dtt_vegetation_delete_instance_count
            ),
            "vegetation_lane_overlap_instance_count": (
                vegetation_lane_overlap_instance_count
            ),
            "vegetation_building_overlap_instance_count": (
                vegetation_building_overlap_instance_count
            ),
            "deleted_instance_count": deleted_instance_count,
            "failed_delete_instance_count": failed_delete_instance_count,
            "examples": examples,
            "instance_examples": instance_examples,
            "save_result": save_result,
            "success": (
                failed_delete_count == 0
                and failed_streetlight_delete_count == 0
                and failed_streetlight_tag_count == 0
                and failed_traffic_control_delete_count == 0
                and failed_delete_instance_count == 0
                and failed_road_component_lift_count == 0
                and failed_road_actor_lift_count == 0
                and failed_terrain_component_lower_count == 0
                and failed_terrain_lower_count == 0
                and bool(save_result.get("save_ok", True))
            ),
            "finish_reason": "generated_level_layer_cleanup_completed",
            "finished_at": time.time(),
        }
    )
    save_report()

    log(
        "TARGET={} BUILDINGS={} ROAD_SURFACES={} VEGETATION={} "
        "TERRAIN={} ROAD_COMPONENT_OVERLAP={} ROAD_COMPONENT_LIFTED={} "
        "ROAD_ACTOR_OVERLAP={} ROAD_ACTOR_LIFTED={} "
        "TERRAIN_COMPONENT_OVERLAP={} TERRAIN_COMPONENT_LOWERED={} "
        "TERRAIN_COMPONENT_SKIPPED={} TERRAIN_ROAD_OVERLAP={} "
        "TERRAIN_LOWERED={} TERRAIN_MATERIAL_ASSIGNMENTS={} "
        "ACTOR_OVERLAP={} ACTOR_DELETED={} STREETLIGHT_CANDIDATES={} "
        "STREETLIGHT_LANE_OVERLAP={} STREETLIGHT_DELETED={} "
        "TRAFFIC_CONTROL_CANDIDATES={} TRAFFIC_CONTROL_LANE_OVERLAP={} "
        "TRAFFIC_CONTROL_NOT_OSM={} TRAFFIC_CONTROL_DELETED={} "
        "INSTANCE_OVERLAP={} INSTANCE_DELETED={} SUCCESS={}".format(
            TARGET_LEVEL,
            len(buildings),
            len(road_surface_actors),
            len(vegetation_candidates),
            len(terrain_candidates),
            len(road_terrain_overlap_components),
            road_lifted_component_count,
            len(road_terrain_overlap_actors),
            road_lifted_actor_count,
            len(terrain_road_overlap_components),
            terrain_lowered_component_count,
            len(skipped_large_terrain_components),
            len(terrain_road_overlap_actors),
            terrain_lowered_count,
            terrain_material_assignment_count,
            len(actors_to_delete),
            deleted_count,
            len(streetlight_candidates),
            len(streetlights_to_delete),
            deleted_streetlight_count,
            len(traffic_control_candidates),
            sum(
                1 for _actor, info in traffic_controls_to_delete
                if info.get("driving_lane_intrusion") is not None
            ),
            sum(
                1 for _actor, info in traffic_controls_to_delete
                if info.get("delete_reason")
                == "traffic_control_not_osm_authoritative"
            ),
            deleted_traffic_control_count,
            overlapping_instance_count,
            deleted_instance_count,
            int(bool(STATE["success"])),
        )
    )

    if not STATE["success"]:
        raise RuntimeError("Generated map layer cleanup failed")


try:
    run()
except Exception:
    STATE["success"] = False
    STATE["finish_reason"] = "generated_level_layer_cleanup_exception"
    STATE["exception"] = traceback.format_exc()
    STATE["finished_at"] = time.time()
    save_report()
    unreal.log_error(STATE["exception"])
    raise
