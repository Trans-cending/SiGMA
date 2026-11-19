#!/usr/bin/env python3
"""Prepare an auditable OSM street-lamp placement plan for SiGMA.

Policy:
- OSM nodes tagged ``highway=street_lamp`` provide the authoritative locations.
- Node longitude/latitude are projected with the installed OpenDRIVE
  ``+proj=tmerc`` georeference, preserving their mapped X/Y positions.
- The nearest OpenDRIVE road supplies base elevation and a road-facing yaw
  for the lamp asset.

The script uses the Python standard library plus helpers shipped with SiGMA
in ``prepare_sidewalks.py``.

Standalone CLI: python -m sigma.map_construction.tools.prepare_streetlights --help
The unified Stage I entry is documented in docs/stage1.md.
"""
from __future__ import annotations

import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from sigma.map_construction.tools.prepare_sidewalks import (
    _build_road_samples,
    _lane_center_t,
    _road_pose,
    _tags,
    projection_from_xodr,
)

RELEASE_VERSION = "1.0.1"


def _nearest_road(
    x: float,
    y: float,
    samples: List[Dict[str, object]],
) -> Optional[Dict[str, object]]:
    best = None
    best_distance = float("inf")
    for sample in samples:
        dx = float(sample["center_x_m"]) - x
        dy = float(sample["center_y_m"]) - y
        distance = math.hypot(dx, dy)
        if distance >= best_distance:
            continue
        best_distance = distance
        best = dict(sample)
        best["distance_m"] = distance
    return best


def _build_driving_lane_samples(
    root: ET.Element,
    spacing_m: float,
) -> Tuple[List[Dict[str, object]], Dict[Tuple[int, int], List[int]], float]:
    cell_size_m = 10.0
    samples: List[Dict[str, object]] = []
    grid: Dict[Tuple[int, int], List[int]] = {}
    step = max(0.25, float(spacing_m))

    for road in root.findall("road"):
        road_id = str(road.get("id", "0"))
        road_name = str(road.get("name", ""))
        road_length = max(0.0, float(road.get("length", "0")))
        sections = road.findall("./lanes/laneSection")
        for section_index, section in enumerate(sections):
            section_s0 = float(section.get("s", "0"))
            section_s1 = (
                float(sections[section_index + 1].get("s", "0"))
                if section_index + 1 < len(sections)
                else road_length
            )
            section_length = max(0.0, section_s1 - section_s0)
            if section_length <= 0.05:
                continue
            sample_count = max(2, int(math.ceil(section_length / step)) + 1)
            for side_name in ("left", "right"):
                side = section.find(side_name)
                if side is None:
                    continue
                for lane in side.findall("lane"):
                    if str(lane.get("type", "")).lower() != "driving":
                        continue
                    lane_id = int(lane.get("id", "0"))
                    for sample_index in range(sample_count):
                        s = section_s0 + section_length * sample_index / max(1, sample_count - 1)
                        try:
                            t, width = _lane_center_t(road, section, side_name, lane_id, s)
                            x0, y0, _z, heading = _road_pose(road, s)
                        except Exception:
                            continue
                        if not math.isfinite(width) or width <= 0.0:
                            continue
                        x = x0 - math.sin(heading) * t
                        y = y0 + math.cos(heading) * t
                        if not (math.isfinite(x) and math.isfinite(y)):
                            continue
                        item = {
                            "x": x,
                            "y": y,
                            "lane_width_m": width,
                            "road_id": road_id,
                            "road_name": road_name,
                            "section_index": section_index,
                            "lane_id": lane_id,
                            "s_m": s,
                        }
                        index = len(samples)
                        samples.append(item)
                        key = (
                            int(math.floor(x / cell_size_m)),
                            int(math.floor(y / cell_size_m)),
                        )
                        grid.setdefault(key, []).append(index)
    return samples, grid, cell_size_m


def _driving_lane_intrusion_at_xy(
    x: float,
    y: float,
    samples: List[Dict[str, object]],
    grid: Dict[Tuple[int, int], List[int]],
    cell_size_m: float,
    clearance_m: float,
) -> Optional[Dict[str, object]]:
    if not samples:
        return None
    cell_x = int(math.floor(x / cell_size_m))
    cell_y = int(math.floor(y / cell_size_m))
    best = None
    for offset_x in (-1, 0, 1):
        for offset_y in (-1, 0, 1):
            for index in grid.get((cell_x + offset_x, cell_y + offset_y), []):
                sample = samples[index]
                distance_m = math.hypot(x - float(sample["x"]), y - float(sample["y"]))
                interior_radius_m = max(
                    0.0,
                    float(sample["lane_width_m"]) / 2.0 - max(0.0, float(clearance_m)),
                )
                intrusion_m = interior_radius_m - distance_m
                if intrusion_m < 0.0:
                    continue
                if best is None or intrusion_m > best["intrusion_m"]:
                    best = {
                        "street_lamp_x_m": x,
                        "street_lamp_y_m": y,
                        "nearest_lane_x_m": float(sample["x"]),
                        "nearest_lane_y_m": float(sample["y"]),
                        "lane_center_distance_m": distance_m,
                        "lane_width_m": float(sample["lane_width_m"]),
                        "lane_interior_radius_m": interior_radius_m,
                        "intrusion_m": intrusion_m,
                        "road_id": str(sample.get("road_id", "")),
                        "road_name": str(sample.get("road_name", "")),
                        "section_index": int(sample.get("section_index", 0)),
                        "lane_id": int(sample.get("lane_id", 0)),
                        "s_m": float(sample.get("s_m", 0.0)),
                    }
    return best


def build_plan(
    osm_path: Path,
    xodr_path: Path,
    map_name: str,
    bounds_margin_m: float,
    base_z_offset_m: float,
    driving_lane_clearance_m: float,
    driving_lane_sample_spacing_m: float,
) -> Dict[str, object]:
    osm_root = ET.parse(str(osm_path)).getroot()
    xodr_root = ET.parse(str(xodr_path)).getroot()
    projection, projection_parameters = projection_from_xodr(xodr_root)

    road_samples = _build_road_samples(xodr_root, spacing_m=2.0)
    driving_lane_samples, driving_lane_grid, driving_lane_cell_size_m = (
        _build_driving_lane_samples(
            xodr_root,
            spacing_m=driving_lane_sample_spacing_m,
        )
    )
    header = xodr_root.find("header")
    if header is None:
        raise ValueError("OpenDRIVE header is missing")
    xodr_bounds = {
        "west": float(header.get("west", "0")),
        "east": float(header.get("east", "0")),
        "south": float(header.get("south", "0")),
        "north": float(header.get("north", "0")),
    }

    source_nodes = []
    planned = []
    skipped = []
    tag_values = Counter()

    for node in osm_root.findall("node"):
        tags = _tags(node)
        if tags.get("highway") != "street_lamp":
            continue
        osm_id = str(node.get("id", ""))
        try:
            latitude = float(node.get("lat"))
            longitude = float(node.get("lon"))
        except (TypeError, ValueError):
            skipped.append({
                "osm_node_id": osm_id,
                "reason": "invalid_lat_lon",
            })
            continue

        for key in (
            "lamp_type", "light:colour", "light:method", "operator",
            "manufacturer", "model", "ref", "lit",
        ):
            if key in tags:
                tag_values["{}={}".format(key, tags[key])] += 1

        x, y = projection.forward(latitude, longitude)
        source = {
            "osm_node_id": osm_id,
            "latitude": latitude,
            "longitude": longitude,
            "tags": tags,
            "opendrive_x_m": x,
            "opendrive_y_m": y,
            "ue_x_m": x,
            "ue_y_m": -y,
        }
        source_nodes.append(source)

        in_bounds = (
            xodr_bounds["west"] - bounds_margin_m
            <= x
            <= xodr_bounds["east"] + bounds_margin_m
            and xodr_bounds["south"] - bounds_margin_m
            <= y
            <= xodr_bounds["north"] + bounds_margin_m
        )
        if not in_bounds:
            skipped.append({
                **source,
                "reason": "outside_generated_xodr_bounds",
                "xodr_bounds": xodr_bounds,
                "bounds_margin_m": bounds_margin_m,
            })
            continue

        lane_intrusion = _driving_lane_intrusion_at_xy(
            x,
            y,
            driving_lane_samples,
            driving_lane_grid,
            driving_lane_cell_size_m,
            driving_lane_clearance_m,
        )
        if lane_intrusion is not None:
            skipped.append({
                **source,
                "reason": "inside_driving_lane",
                "driving_lane_intrusion": lane_intrusion,
                "driving_lane_clearance_m": driving_lane_clearance_m,
            })
            continue

        nearest = _nearest_road(x, y, road_samples)
        if nearest is None:
            skipped.append({
                **source,
                "reason": "xodr_contains_no_road_samples_for_elevation",
            })
            continue

        # Preserve the OSM node X/Y exactly. OpenDRIVE supplies Z and an
        # orientation hint for the mapped street lamp.
        road_x = float(nearest["center_x_m"])
        road_y = float(nearest["center_y_m"])
        # UE y is mirrored relative to OpenDRIVE y.
        road_ue_x = road_x
        road_ue_y = -road_y
        toward_road_yaw_deg = math.degrees(
            math.atan2(road_ue_y - (-y), road_ue_x - x)
        )

        planned.append({
            **source,
            "ue_z_m": float(nearest.get("center_z_m", 0.0)) + base_z_offset_m,
            "nearest_road_id": str(nearest.get("road_id", "")),
            "nearest_road_name": str(nearest.get("road_name", "")),
            "nearest_road_s_m": float(nearest.get("s_m", 0.0)),
            "nearest_road_distance_m": float(nearest["distance_m"]),
            "nearest_road_heading_deg": float(nearest.get("heading_deg", 0.0)),
            "road_facing_yaw_deg": toward_road_yaw_deg,
        })

    return {
        "release_version": RELEASE_VERSION,
        "runner": "osm-street-lamp-fidelity-planner",
        "map_name": map_name,
        "policy": "osm_highway_street_lamp_only_no_procedural_fallback",
        "coordinate_conversion": "WGS84 -> OpenDRIVE +proj=tmerc -> CARLA/UE (x,-y)",
        "osm_path": str(osm_path),
        "xodr_path": str(xodr_path),
        "projection": projection_parameters,
        "xodr_bounds": xodr_bounds,
        "bounds_margin_m": bounds_margin_m,
        "base_z_offset_m": base_z_offset_m,
        "driving_lane_clearance_m": driving_lane_clearance_m,
        "driving_lane_sample_spacing_m": driving_lane_sample_spacing_m,
        "driving_lane_sample_count": len(driving_lane_samples),
        "osm_street_lamp_node_count": len(source_nodes),
        "planned_street_lamp_count": len(planned),
        "skipped_street_lamp_count": len(skipped),
        "tag_value_counts": dict(tag_values),
        "source_nodes": source_nodes,
        "planned_street_lamps": planned,
        "skipped_street_lamps": skipped,
        "created_at": time.time(),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--osm", required=True, type=Path)
    parser.add_argument("--xodr", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--bounds-margin-m", type=float, default=5.0)
    parser.add_argument("--base-z-offset-m", type=float, default=0.04)
    parser.add_argument("--driving-lane-clearance-m", type=float, default=0.20)
    parser.add_argument("--driving-lane-sample-spacing-m", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.osm.is_file():
        raise SystemExit("OSM file not found: {}".format(args.osm))
    if not args.xodr.is_file():
        raise SystemExit("OpenDRIVE file not found: {}".format(args.xodr))
    payload = build_plan(
        args.osm,
        args.xodr,
        args.map_name,
        max(0.0, float(args.bounds_margin_m)),
        float(args.base_z_offset_m),
        max(0.0, float(args.driving_lane_clearance_m)),
        max(0.25, float(args.driving_lane_sample_spacing_m)),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(json.dumps({
        "OSM_STREET_LAMP_NODES": payload["osm_street_lamp_node_count"],
        "PLANNED_STREET_LAMPS": payload["planned_street_lamp_count"],
        "SKIPPED_STREET_LAMPS": payload["skipped_street_lamp_count"],
        "OUTPUT": str(args.output),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
