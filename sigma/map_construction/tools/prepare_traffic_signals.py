#!/usr/bin/env python3
"""Prepare an auditable OSM traffic-signal reference plan for SiGMA.

Policy:
- OSM nodes tagged ``highway=traffic_signals`` are authoritative for vehicle
  traffic lights.
- Crossing nodes tagged ``crossing=traffic_signals`` provide secondary
  references for identifying pedestrian signal assets near crossings.
- The plan guides pruning of DTT-generated traffic-control actors. Functional
  traffic-light spawning belongs to CARLA/OpenDRIVE junction control logic.

Standalone CLI: python -m sigma.map_construction.tools.prepare_traffic_signals --help
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
from typing import Dict

from sigma.map_construction.tools.prepare_sidewalks import _tags, projection_from_xodr

RELEASE_VERSION = "1.0.0"


def _is_traffic_signal_node(tags: Dict[str, str]) -> bool:
    if tags.get("highway") == "traffic_signals":
        return True
    if tags.get("crossing") == "traffic_signals":
        return True
    if tags.get("traffic_signals"):
        return True
    return any(key.startswith("traffic_signals:") for key in tags)


def _signal_kind(tags: Dict[str, str]) -> str:
    if tags.get("highway") == "traffic_signals":
        return "vehicle_traffic_signal"
    if tags.get("crossing") == "traffic_signals":
        return "crossing_traffic_signal"
    return "traffic_signal_auxiliary"


def build_plan(
    osm_path: Path,
    xodr_path: Path,
    map_name: str,
    bounds_margin_m: float,
) -> Dict[str, object]:
    osm_root = ET.parse(str(osm_path)).getroot()
    xodr_root = ET.parse(str(xodr_path)).getroot()
    projection, projection_parameters = projection_from_xodr(xodr_root)

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
    kind_counts = Counter()
    tag_value_counts = Counter()

    for node in osm_root.findall("node"):
        tags = _tags(node)
        if not _is_traffic_signal_node(tags):
            continue
        osm_id = str(node.get("id", ""))
        kind = _signal_kind(tags)
        kind_counts[kind] += 1
        try:
            latitude = float(node.get("lat"))
            longitude = float(node.get("lon"))
        except (TypeError, ValueError):
            skipped.append({
                "osm_node_id": osm_id,
                "kind": kind,
                "reason": "invalid_lat_lon",
            })
            continue

        for key in (
            "highway", "traffic_signals", "traffic_signals:direction",
            "crossing", "crossing_ref", "traffic_signals:sound",
            "traffic_signals:vibration",
        ):
            if key in tags:
                tag_value_counts["{}={}".format(key, tags[key])] += 1

        x, y = projection.forward(latitude, longitude)
        source = {
            "osm_node_id": osm_id,
            "kind": kind,
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
        planned.append(source)

    return {
        "release_version": RELEASE_VERSION,
        "runner": "osm-traffic-signal-reference-planner",
        "map_name": map_name,
        "policy": "osm_traffic_signals_reference_no_procedural_fallback",
        "coordinate_conversion": "WGS84 -> OpenDRIVE +proj=tmerc -> CARLA/UE (x,-y)",
        "osm_path": str(osm_path),
        "xodr_path": str(xodr_path),
        "projection": projection_parameters,
        "xodr_bounds": xodr_bounds,
        "bounds_margin_m": bounds_margin_m,
        "osm_traffic_signal_node_count": len(source_nodes),
        "planned_traffic_signal_count": len(planned),
        "skipped_traffic_signal_count": len(skipped),
        "kind_counts": dict(kind_counts),
        "tag_value_counts": dict(tag_value_counts),
        "source_nodes": source_nodes,
        "planned_traffic_signals": planned,
        "skipped_traffic_signals": skipped,
        "created_at": time.time(),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--osm", required=True, type=Path)
    parser.add_argument("--xodr", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--bounds-margin-m", type=float, default=5.0)
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
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(json.dumps({
        "OSM_TRAFFIC_SIGNAL_NODES": payload["osm_traffic_signal_node_count"],
        "PLANNED_TRAFFIC_SIGNALS": payload["planned_traffic_signal_count"],
        "SKIPPED_TRAFFIC_SIGNALS": payload["skipped_traffic_signal_count"],
        "OUTPUT": str(args.output),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
