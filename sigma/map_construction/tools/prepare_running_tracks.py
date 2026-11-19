#!/usr/bin/env python3
"""Project OSM tartan running tracks into an auditable Unreal mesh plan.

Standalone CLI: python -m sigma.map_construction.tools.prepare_running_tracks --help
The unified Stage I entry is documented in docs/stage1.md.
"""

from __future__ import annotations

import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

from sigma.map_construction.tools.prepare_sidewalks import _build_road_samples, _tags, projection_from_xodr


RELEASE_VERSION = "1.0.2"
TRACK_SURFACES = {"tartan", "rubber", "synthetic"}
GRASS_SURFACES = {"grass", "turf", "grass_paver"}


def _nearest_z(x: float, y: float, samples: Sequence[Dict[str, object]]) -> float:
    if not samples:
        return 0.0
    sample = min(
        samples,
        key=lambda item: math.hypot(
            x - float(item["center_x_m"]), y - float(item["center_y_m"])
        ),
    )
    return float(sample.get("center_z_m", 0.0))


def _track_width(tags: Dict[str, str], default_width_m: float) -> float:
    try:
        value = float(str(tags.get("width", "")).strip())
    except ValueError:
        value = float(default_width_m)
    return min(20.0, max(1.0, value))


def _rectangular_surface(
    points: Sequence[Tuple[float, float]],
    *,
    osm_way_id: str,
    center_z_m: float,
) -> Dict[str, object]:
    unique = list(points)
    if len(unique) > 1 and unique[0] == unique[-1]:
        unique.pop()
    if len(unique) != 4:
        raise ValueError("sports-surface way {} must have four corners".format(osm_way_id))
    edges = []
    for index in range(4):
        x0, y0 = unique[index]
        x1, y1 = unique[(index + 1) % 4]
        edges.append((math.hypot(x1 - x0, y1 - y0), index, x0, y0, x1, y1))
    _edge_length, index, x0, y0, x1, y1 = max(edges)
    opposite = edges[(index + 2) % 4]
    length_m = 0.5 * (edges[index][0] + opposite[0])
    midpoint = ((x0 + x1) * 0.5, (y0 + y1) * 0.5)
    opposite_midpoint = (
        (opposite[2] + opposite[4]) * 0.5,
        (opposite[3] + opposite[5]) * 0.5,
    )
    width_m = math.hypot(
        midpoint[0] - opposite_midpoint[0],
        midpoint[1] - opposite_midpoint[1],
    )
    return {
        "osm_way_id": osm_way_id,
        "center_x_m": sum(point[0] for point in unique) / 4.0,
        "center_y_m": sum(point[1] for point in unique) / 4.0,
        "center_z_m": center_z_m,
        "heading_deg": math.degrees(math.atan2(y1 - y0, x1 - x0)),
        "length_m": length_m,
        "width_m": width_m,
    }


def build_plan(
    osm_path: Path,
    xodr_path: Path,
    map_name: str,
    *,
    default_width_m: float = 8.0,
    max_piece_length_m: float = 3.0,
) -> Dict[str, object]:
    osm_root = ET.parse(str(osm_path)).getroot()
    xodr_root = ET.parse(str(xodr_path)).getroot()
    projection, projection_parameters = projection_from_xodr(xodr_root)
    nodes = {
        str(node.get("id")): (float(node.get("lat")), float(node.get("lon")))
        for node in osm_root.findall("node")
        if node.get("id") and node.get("lat") and node.get("lon")
    }
    road_samples = _build_road_samples(xodr_root, spacing_m=4.0)
    tracks: List[Dict[str, object]] = []
    segments: List[Dict[str, object]] = []
    basketball_courts: List[Dict[str, object]] = []
    grass_surfaces: List[Dict[str, object]] = []

    for way in osm_root.findall("way"):
        tags = _tags(way)
        node_ids = [str(nd.get("ref")) for nd in way.findall("nd")]
        coordinates = [nodes[node_id] for node_id in node_ids if node_id in nodes]
        if len(coordinates) < 2:
            continue
        projected = [projection.forward(lat, lon) for lat, lon in coordinates]
        is_grass_pitch = (
            tags.get("leisure", "").casefold() == "pitch"
            and tags.get("sport", "").casefold() != "basketball"
            and (
                tags.get("surface", "").casefold() in GRASS_SURFACES
                or tags.get("landuse", "").casefold() == "grass"
                or tags.get("mosaic:surface_compatibility", "").casefold()
                == "grass_pitch"
            )
        )
        if is_grass_pitch:
            center_x = sum(point[0] for point in projected[:-1]) / max(1, len(projected) - 1)
            center_y = sum(point[1] for point in projected[:-1]) / max(1, len(projected) - 1)
            try:
                grass_surface = _rectangular_surface(
                    projected,
                    osm_way_id=str(way.get("id", "")),
                    center_z_m=_nearest_z(center_x, center_y, road_samples),
                )
            except ValueError:
                # The Unreal surface primitive accepts rectangular polygons;
                # preserve arbitrary polygons by leaving them out of this pass.
                continue
            grass_surface.update({
                "name": tags.get("name", ""),
                "surface": tags.get("surface", "grass"),
                "sport": tags.get("sport", ""),
            })
            grass_surfaces.append(grass_surface)
        if (
            tags.get("leisure", "").casefold() == "pitch"
            and tags.get("sport", "").casefold() == "basketball"
        ):
            center_x = sum(point[0] for point in projected[:-1]) / max(1, len(projected) - 1)
            center_y = sum(point[1] for point in projected[:-1]) / max(1, len(projected) - 1)
            try:
                court = _rectangular_surface(
                    projected,
                    osm_way_id=str(way.get("id", "")),
                    center_z_m=_nearest_z(center_x, center_y, road_samples),
                )
            except ValueError:
                continue
            court.update({
                "name": tags.get("name", ""),
                "surface": tags.get("surface", "unspecified"),
                "sport": "basketball",
            })
            basketball_courts.append(court)
        if tags.get("leisure", "").casefold() != "track":
            continue
        if tags.get("surface", "").casefold() not in TRACK_SURFACES:
            continue
        width_m = _track_width(tags, default_width_m)
        osm_way_id = str(way.get("id", ""))
        track_length_m = 0.0
        first_segment_index = len(segments)
        for source_index, ((x0, y0), (x1, y1)) in enumerate(zip(projected, projected[1:])):
            dx, dy = x1 - x0, y1 - y0
            length_m = math.hypot(dx, dy)
            if length_m <= 0.02:
                continue
            track_length_m += length_m
            piece_count = max(1, int(math.ceil(length_m / max(0.25, max_piece_length_m))))
            heading_deg = math.degrees(math.atan2(dy, dx))
            for piece_index in range(piece_count):
                t0 = piece_index / piece_count
                t1 = (piece_index + 1) / piece_count
                cx = x0 + dx * (t0 + t1) * 0.5
                cy = y0 + dy * (t0 + t1) * 0.5
                segments.append({
                    "osm_way_id": osm_way_id,
                    "source_segment_index": source_index,
                    "piece_index": piece_index,
                    "center_x_m": cx,
                    "center_y_m": cy,
                    "center_z_m": _nearest_z(cx, cy, road_samples),
                    "heading_deg": heading_deg,
                    "length_m": length_m / piece_count,
                    "width_m": width_m,
                })
        tracks.append({
            "osm_way_id": osm_way_id,
            "name": tags.get("name", ""),
            "surface": tags.get("surface", ""),
            "width_m": width_m,
            "length_m": track_length_m,
            "closed": bool(node_ids and node_ids[0] == node_ids[-1]),
            "segment_start": first_segment_index,
            "segment_count": len(segments) - first_segment_index,
        })

    return {
        "release_version": RELEASE_VERSION,
        "map_name": map_name,
        "source_osm": str(osm_path),
        "source_xodr": str(xodr_path),
        "projection": projection_parameters,
        "coordinate_conversion": "OpenDRIVE (x,+north y) -> Unreal (x,-y)",
        "selection": "leisure=track and surface in tartan/rubber/synthetic",
        "track_count": len(tracks),
        "planned_segment_count": len(segments),
        "basketball_court_count": len(basketball_courts),
        "grass_surface_count": len(grass_surfaces),
        "tracks": tracks,
        "segments": segments,
        "basketball_courts": basketball_courts,
        "grass_surfaces": grass_surfaces,
        "created_at": time.time(),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--osm", type=Path, required=True)
    parser.add_argument("--xodr", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--default-width-m", type=float, default=8.0)
    parser.add_argument("--max-piece-length-m", type=float, default=3.0)
    args = parser.parse_args()
    report = build_plan(
        args.osm,
        args.xodr,
        args.map_name,
        default_width_m=args.default_width_m,
        max_piece_length_m=args.max_piece_length_m,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("RUNNING_TRACK_COUNT={}".format(report["track_count"]))
    print("RUNNING_TRACK_SEGMENTS={}".format(report["planned_segment_count"]))
    print("BASKETBALL_COURTS={}".format(report["basketball_court_count"]))
    print("GRASS_SURFACES={}".format(report["grass_surface_count"]))
    print("RUNNING_TRACK_PLAN={}".format(args.output))


if __name__ == "__main__":
    main()
