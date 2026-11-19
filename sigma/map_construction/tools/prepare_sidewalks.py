#!/usr/bin/env python3
"""Prepare an auditable SiGMA sidewalk fidelity/reconstruction plan.

SiGMA preserves two complementary sidewalk sources:

1. Sidewalk lanes that survived OSM -> OpenDRIVE (``lane type=\"sidewalk\"``).
2. Explicit OSM sidewalk ways (normally ``highway=footway`` +
   ``footway=sidewalk``) that CARLA/DTT can omit during OSM -> OpenDRIVE.

The planner keeps all surviving OpenDRIVE sidewalk geometry, audits each
explicit OSM sidewalk segment against it, and emits fallback geometry only for
OSM segments inside the generated road area with a gap in OpenDRIVE sidewalk
coverage. When the generated SideWalk asset set is empty, surviving OpenDRIVE
sidewalk lanes are also emitted as a fallback.

The script uses the Python standard library. SiGMA's OpenDRIVE preparation
step writes a WGS84 ``+proj=tmerc`` georeference, which the planner handles
with a compact transverse-Mercator implementation.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import time
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

RELEASE_VERSION = "1.0.1"
SIDEWALK_LANE_TYPES = {"sidewalk", "walking", "pedestrian"}
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
WGS84_EP2 = WGS84_E2 / (1.0 - WGS84_E2)


def _tags(element: ET.Element) -> Dict[str, str]:
    return {
        str(tag.get("k")): str(tag.get("v"))
        for tag in element.findall("tag")
        if tag.get("k") is not None and tag.get("v") is not None
    }


def _angle180(value_deg: float) -> float:
    return float(value_deg) % 180.0


def _heading_error_deg(first: float, second: float) -> float:
    delta = abs((_angle180(first) - _angle180(second)) % 180.0)
    return min(delta, 180.0 - delta)


def _meridional_arc(phi: float) -> float:
    e2 = WGS84_E2
    e4 = e2 * e2
    e6 = e4 * e2
    return WGS84_A * (
        (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * phi
        - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0)
        * math.sin(2.0 * phi)
        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * math.sin(4.0 * phi)
        - (35.0 * e6 / 3072.0) * math.sin(6.0 * phi)
    )


class TransverseMercator:
    """Small-area WGS84 transverse Mercator matching PROJ ``+proj=tmerc``."""

    def __init__(
        self,
        latitude_origin_deg: float,
        longitude_origin_deg: float,
        scale: float = 1.0,
        false_easting_m: float = 0.0,
        false_northing_m: float = 0.0,
    ) -> None:
        self.latitude_origin_deg = float(latitude_origin_deg)
        self.longitude_origin_deg = float(longitude_origin_deg)
        self.scale = float(scale)
        self.false_easting_m = float(false_easting_m)
        self.false_northing_m = float(false_northing_m)
        self._lat0 = math.radians(self.latitude_origin_deg)
        self._lon0 = math.radians(self.longitude_origin_deg)
        self._m0 = _meridional_arc(self._lat0)

    def forward(self, latitude_deg: float, longitude_deg: float) -> Tuple[float, float]:
        phi = math.radians(float(latitude_deg))
        lam = math.radians(float(longitude_deg))
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
        tan_phi = math.tan(phi)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
        t = tan_phi * tan_phi
        c = WGS84_EP2 * cos_phi * cos_phi
        a = cos_phi * (lam - self._lon0)
        m = _meridional_arc(phi)
        x = n * (
            a
            + (1.0 - t + c) * a**3 / 6.0
            + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * WGS84_EP2)
            * a**5
            / 120.0
        )
        y = (m - self._m0) + n * tan_phi * (
            a * a / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c * c) * a**4 / 24.0
            + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * WGS84_EP2)
            * a**6
            / 720.0
        )
        return (
            self.false_easting_m + self.scale * x,
            self.false_northing_m + self.scale * y,
        )


def _parse_proj_parameter(text: str, name: str, default: Optional[float] = None) -> float:
    match = re.search(r"(?:^|\s)\+{}=([^\s]+)".format(re.escape(name)), text)
    if match:
        return float(match.group(1))
    if default is None:
        raise ValueError("OpenDRIVE geoReference is missing +{}".format(name))
    return float(default)


def projection_from_xodr(root: ET.Element) -> Tuple[TransverseMercator, Dict[str, float]]:
    header = root.find("header")
    if header is None:
        raise ValueError("OpenDRIVE header is missing")
    georef = (header.findtext("geoReference") or "").strip()
    if "+proj=tmerc" not in georef:
        raise ValueError(
            "MOSAIC sidewalk planner requires +proj=tmerc OpenDRIVE; got {!r}".format(georef)
        )
    parameters = {
        "lat_0": _parse_proj_parameter(georef, "lat_0"),
        "lon_0": _parse_proj_parameter(georef, "lon_0"),
        "k": _parse_proj_parameter(georef, "k", _parse_proj_parameter(georef, "k_0", 1.0)),
        "x_0": _parse_proj_parameter(georef, "x_0", 0.0),
        "y_0": _parse_proj_parameter(georef, "y_0", 0.0),
    }
    return (
        TransverseMercator(
            parameters["lat_0"],
            parameters["lon_0"],
            parameters["k"],
            parameters["x_0"],
            parameters["y_0"],
        ),
        parameters,
    )


def audit_osm(osm_path: Path) -> Dict[str, object]:
    if not osm_path.exists():
        return {"available": False}
    root = ET.parse(str(osm_path)).getroot()
    counts = Counter()
    sidewalk_values = Counter()
    footway_values = Counter()
    surfaces = Counter()
    for way in root.findall("way"):
        tags = _tags(way)
        highway = tags.get("highway", "")
        if highway == "footway":
            counts["highway_footway_way_count"] += 1
            footway_values[tags.get("footway", "(unspecified)")] += 1
        if tags.get("footway") == "sidewalk":
            counts["footway_sidewalk_way_count"] += 1
            surfaces[tags.get("surface", "(unspecified)")] += 1
        if highway == "pedestrian":
            counts["highway_pedestrian_way_count"] += 1
        if "sidewalk" in tags:
            counts["road_with_sidewalk_tag_count"] += 1
            sidewalk_values[tags["sidewalk"]] += 1
        if any(key.startswith("sidewalk:") for key in tags):
            counts["road_with_sidewalk_subtag_count"] += 1
            for key, value in tags.items():
                if key.startswith("sidewalk:"):
                    sidewalk_values["{}={}".format(key, value)] += 1
    return {
        "available": True,
        **dict(counts),
        "sidewalk_values": dict(sidewalk_values),
        "footway_values": dict(footway_values),
        "explicit_sidewalk_surfaces": dict(surfaces),
    }


def _poly_value(element: Optional[ET.Element], ds: float, default: float = 0.0) -> float:
    if element is None:
        return float(default)
    return sum(
        float(element.get(name, "0")) * float(ds) ** power
        for power, name in enumerate(("a", "b", "c", "d"))
    )


def _active_record(elements: Sequence[ET.Element], s: float, coordinate: str) -> Optional[ET.Element]:
    active = None
    for element in elements:
        start = float(element.get(coordinate, "0"))
        if start <= s + 1e-9:
            active = element
        else:
            break
    return active


def _evaluate_geometry(geometry: ET.Element, local_s: float) -> Tuple[float, float, float]:
    x0 = float(geometry.get("x", "0"))
    y0 = float(geometry.get("y", "0"))
    hdg0 = float(geometry.get("hdg", "0"))
    length = max(0.0, float(geometry.get("length", "0")))
    local_s = min(max(0.0, float(local_s)), length)
    child = next(iter(geometry), None)
    if child is None or child.tag == "line":
        u, v, local_hdg = local_s, 0.0, 0.0
    elif child.tag == "arc":
        curvature = float(child.get("curvature", "0"))
        if abs(curvature) < 1e-12:
            u, v, local_hdg = local_s, 0.0, 0.0
        else:
            u = math.sin(curvature * local_s) / curvature
            v = (1.0 - math.cos(curvature * local_s)) / curvature
            local_hdg = curvature * local_s
    elif child.tag == "paramPoly3":
        if child.get("pRange", "normalized") == "arcLength":
            p = local_s
            dp_ds = 1.0
        else:
            p = local_s / length if length > 1e-12 else 0.0
            dp_ds = 1.0 / length if length > 1e-12 else 0.0
        cu = [float(child.get(name, "0")) for name in ("aU", "bU", "cU", "dU")]
        cv = [float(child.get(name, "0")) for name in ("aV", "bV", "cV", "dV")]
        u = sum(value * p**power for power, value in enumerate(cu))
        v = sum(value * p**power for power, value in enumerate(cv))
        du_dp = cu[1] + 2.0 * cu[2] * p + 3.0 * cu[3] * p * p
        dv_dp = cv[1] + 2.0 * cv[2] * p + 3.0 * cv[3] * p * p
        du_ds = du_dp * dp_ds
        dv_ds = dv_dp * dp_ds
        local_hdg = math.atan2(dv_ds, du_ds) if abs(du_ds) + abs(dv_ds) > 1e-12 else 0.0
    else:
        raise ValueError("Unsupported OpenDRIVE geometry type: {}".format(child.tag))
    cosine = math.cos(hdg0)
    sine = math.sin(hdg0)
    return (
        x0 + u * cosine - v * sine,
        y0 + u * sine + v * cosine,
        hdg0 + local_hdg,
    )


def _road_pose(road: ET.Element, s: float) -> Tuple[float, float, float, float]:
    geometries = road.findall("./planView/geometry")
    if not geometries:
        raise ValueError("Road {} has no planView geometry".format(road.get("id")))
    active = geometries[0]
    for geometry in geometries:
        if float(geometry.get("s", "0")) <= s + 1e-9:
            active = geometry
        else:
            break
    x, y, heading = _evaluate_geometry(active, s - float(active.get("s", "0")))
    elevations = road.findall("./elevationProfile/elevation")
    elevation = _active_record(elevations, s, "s")
    z = _poly_value(
        elevation,
        s - float(elevation.get("s", "0")) if elevation is not None else 0.0,
    )
    return x, y, z, heading


def _lane_width(lane: ET.Element, section_s: float, s: float) -> float:
    ds = max(0.0, s - section_s)
    records = lane.findall("width")
    if not records:
        return 0.0
    active = records[0]
    for record in records:
        if float(record.get("sOffset", "0")) <= ds + 1e-9:
            active = record
        else:
            break
    local = ds - float(active.get("sOffset", "0"))
    return max(0.0, _poly_value(active, local))


def _lane_offset(road: ET.Element, s: float) -> float:
    offsets = road.findall("./lanes/laneOffset")
    active = _active_record(offsets, s, "s")
    if active is None:
        return 0.0
    return _poly_value(active, s - float(active.get("s", "0")))


def _lane_center_t(
    road: ET.Element,
    section: ET.Element,
    side_name: str,
    lane_id: int,
    s: float,
) -> Tuple[float, float]:
    section_s = float(section.get("s", "0"))
    side = section.find(side_name)
    if side is None:
        raise ValueError("Missing {} lanes".format(side_name))
    lanes = sorted(side.findall("lane"), key=lambda lane: abs(int(lane.get("id", "0"))))
    cumulative = 0.0
    for lane in lanes:
        width = _lane_width(lane, section_s, s)
        current_id = int(lane.get("id", "0"))
        if current_id == lane_id:
            center_from_lane_zero = cumulative + 0.5 * width
            sign = 1.0 if lane_id > 0 else -1.0
            return _lane_offset(road, s) + sign * center_from_lane_zero, width
        cumulative += width
    raise ValueError("Lane {} not found".format(lane_id))


def build_xodr_sidewalk_plan(
    root: ET.Element,
    max_segment_m: float,
) -> Tuple[Dict[str, object], List[Dict[str, object]]]:
    lane_type_counts = Counter()
    sidewalk_lane_count = 0
    sidewalk_lane_length_m = 0.0
    segments: List[Dict[str, object]] = []
    skipped = Counter()

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
            for side_name in ("left", "right"):
                side = section.find(side_name)
                if side is None:
                    continue
                for lane in side.findall("lane"):
                    lane_type = str(lane.get("type", ""))
                    lane_type_counts[lane_type] += 1
                    if lane_type.lower() not in SIDEWALK_LANE_TYPES:
                        continue
                    lane_id = int(lane.get("id", "0"))
                    sidewalk_lane_count += 1
                    sidewalk_lane_length_m += section_length
                    if section_length <= 0.05:
                        skipped["zero_length_lane_section"] += 1
                        continue
                    piece_count = max(1, int(math.ceil(section_length / max_segment_m)))
                    for piece_index in range(piece_count):
                        piece_s0 = section_s0 + section_length * piece_index / piece_count
                        piece_s1 = section_s0 + section_length * (piece_index + 1) / piece_count
                        mid_s = 0.5 * (piece_s0 + piece_s1)
                        try:
                            t, width = _lane_center_t(road, section, side_name, lane_id, mid_s)
                            x, y, z, heading = _road_pose(road, mid_s)
                        except Exception:
                            skipped["geometry_evaluation_failure"] += 1
                            continue
                        if not math.isfinite(width) or width < 0.35:
                            skipped["invalid_or_too_narrow_width"] += 1
                            continue
                        center_x = x - math.sin(heading) * t
                        center_y = y + math.cos(heading) * t
                        segments.append(
                            {
                                "source": "xodr_sidewalk_lane",
                                "road_id": road_id,
                                "road_name": road_name,
                                "section_index": section_index,
                                "lane_id": lane_id,
                                "lane_type": lane_type,
                                "segment_index": piece_index,
                                "s_m": mid_s,
                                "center_x_m": center_x,
                                "center_y_m": center_y,
                                "center_z_m": z,
                                "heading_deg": math.degrees(heading),
                                "length_m": max(0.1, piece_s1 - piece_s0),
                                "width_m": width,
                            }
                        )
    sidewalk_widths = sorted(
        float(item["width_m"]) for item in segments
        if math.isfinite(float(item.get("width_m", 0.0)))
        and float(item.get("width_m", 0.0)) > 0.0
    )
    median_width = None
    if sidewalk_widths:
        middle = len(sidewalk_widths) // 2
        if len(sidewalk_widths) % 2:
            median_width = sidewalk_widths[middle]
        else:
            median_width = 0.5 * (
                sidewalk_widths[middle - 1] + sidewalk_widths[middle]
            )

    summary = {
        "xodr_lane_type_counts": dict(lane_type_counts),
        "xodr_sidewalk_lane_count": sidewalk_lane_count,
        "xodr_sidewalk_lane_length_m": sidewalk_lane_length_m,
        "xodr_sidewalk_segment_count": len(segments),
        "xodr_sidewalk_width_median_m": median_width,
        "xodr_sidewalk_width_min_m": min(sidewalk_widths) if sidewalk_widths else None,
        "xodr_sidewalk_width_max_m": max(sidewalk_widths) if sidewalk_widths else None,
        "skipped_segment_counts": dict(skipped),
    }
    return summary, segments


def _grid_key(x: float, y: float, cell_size: float) -> Tuple[int, int]:
    return int(math.floor(x / cell_size)), int(math.floor(y / cell_size))


def _build_grid(records: Sequence[Dict[str, object]], cell_size: float) -> Dict[Tuple[int, int], List[int]]:
    grid: Dict[Tuple[int, int], List[int]] = defaultdict(list)
    for index, record in enumerate(records):
        grid[_grid_key(float(record["center_x_m"]), float(record["center_y_m"]), cell_size)].append(index)
    return grid


def _grid_candidates(
    x: float,
    y: float,
    grid: Dict[Tuple[int, int], List[int]],
    cell_size: float,
    radius_m: float,
) -> Iterable[int]:
    gx, gy = _grid_key(x, y, cell_size)
    cells = max(1, int(math.ceil(radius_m / cell_size)))
    for dx in range(-cells, cells + 1):
        for dy in range(-cells, cells + 1):
            for index in grid.get((gx + dx, gy + dy), ()):
                yield index


def _build_road_samples(root: ET.Element, spacing_m: float = 2.0) -> List[Dict[str, object]]:
    records: List[Dict[str, object]] = []
    for road in root.findall("road"):
        length = max(0.0, float(road.get("length", "0")))
        if length <= 0.01:
            continue
        count = max(2, int(math.ceil(length / spacing_m)) + 1)
        for index in range(count):
            s = length * index / max(1, count - 1)
            try:
                x, y, z, heading = _road_pose(road, s)
            except Exception:
                continue
            records.append(
                {
                    "center_x_m": x,
                    "center_y_m": y,
                    "center_z_m": z,
                    "heading_deg": math.degrees(heading),
                    "road_id": str(road.get("id", "0")),
                    "road_name": str(road.get("name", "")),
                    "s_m": s,
                }
            )
    return records


def _nearest_road_sample(
    x: float,
    y: float,
    heading_deg: float,
    road_samples: Sequence[Dict[str, object]],
    road_grid: Dict[Tuple[int, int], List[int]],
    cell_size: float,
    max_distance_m: float,
) -> Optional[Dict[str, object]]:
    best = None
    best_score = float("inf")
    for index in _grid_candidates(x, y, road_grid, cell_size, max_distance_m):
        item = road_samples[index]
        dx = float(item["center_x_m"]) - x
        dy = float(item["center_y_m"]) - y
        distance = math.hypot(dx, dy)
        if distance > max_distance_m:
            continue
        heading_error = _heading_error_deg(heading_deg, float(item["heading_deg"]))
        # Distance dominates; heading gently disambiguates intersection samples.
        score = distance + min(heading_error, 90.0) * 0.03
        if score < best_score:
            best_score = score
            best = dict(item)
            best["distance_m"] = distance
            best["heading_error_deg"] = heading_error
    return best


def _nearest_xodr_sidewalk_match(
    x: float,
    y: float,
    heading_deg: float,
    xodr_segments: Sequence[Dict[str, object]],
    sidewalk_grid: Dict[Tuple[int, int], List[int]],
    cell_size: float,
    max_distance_m: float,
    max_heading_error_deg: float,
) -> Optional[Dict[str, object]]:
    best = None
    best_score = float("inf")
    for index in _grid_candidates(x, y, sidewalk_grid, cell_size, max_distance_m):
        item = xodr_segments[index]
        dx = float(item["center_x_m"]) - x
        dy = float(item["center_y_m"]) - y
        distance = math.hypot(dx, dy)
        if distance > max_distance_m:
            continue
        heading_error = _heading_error_deg(heading_deg, float(item["heading_deg"]))
        if heading_error > max_heading_error_deg:
            continue
        score = distance + heading_error * 0.02
        if score < best_score:
            best_score = score
            best = dict(item)
            best["match_distance_m"] = distance
            best["match_heading_error_deg"] = heading_error
    return best


def _parse_width_m(
    tags: Dict[str, str],
    default_width_m: float,
    default_width_source: str = "default",
) -> Tuple[float, str]:
    value = tags.get("width", "").strip()
    if value:
        match = re.search(r"[-+]?\d+(?:\.\d+)?", value.replace(",", "."))
        if match:
            width = float(match.group(0))
            if "ft" in value.lower() or "feet" in value.lower() or "foot" in value.lower():
                width *= 0.3048
            if 0.6 <= width <= 8.0:
                return width, "osm_width"
    return float(default_width_m), str(default_width_source)


def build_osm_missing_sidewalk_plan(
    osm_path: Path,
    projection: TransverseMercator,
    xodr_root: ET.Element,
    xodr_sidewalk_segments: Sequence[Dict[str, object]],
    max_segment_m: float,
    coverage_distance_m: float,
    coverage_heading_error_deg: float,
    max_road_distance_m: float,
    default_width_m: float,
    default_width_source: str = "default",
) -> Tuple[Dict[str, object], List[Dict[str, object]]]:
    if not osm_path.is_file():
        return {"available": False}, []

    root = ET.parse(str(osm_path)).getroot()
    nodes: Dict[str, Tuple[float, float]] = {}
    for node in root.findall("node"):
        try:
            nodes[str(node.get("id"))] = (float(node.get("lat")), float(node.get("lon")))
        except Exception:
            continue

    explicit_ways = []
    for way in root.findall("way"):
        tags = _tags(way)
        if tags.get("footway") != "sidewalk":
            continue
        refs = [str(nd.get("ref")) for nd in way.findall("nd") if nd.get("ref")]
        coords = [nodes[ref] for ref in refs if ref in nodes]
        if len(coords) < 2:
            continue
        explicit_ways.append((str(way.get("id")), coords, tags))

    road_samples = _build_road_samples(xodr_root, spacing_m=2.0)
    road_cell = 40.0
    road_grid = _build_grid(road_samples, road_cell)
    sidewalk_cell = 12.0
    sidewalk_grid = _build_grid(xodr_sidewalk_segments, sidewalk_cell)

    counts = Counter()
    total_length_m = 0.0
    in_map_length_m = 0.0
    covered_length_m = 0.0
    missing_length_m = 0.0
    missing_segments: List[Dict[str, object]] = []
    way_summaries = []

    for way_id, coords, tags in explicit_ways:
        projected = [projection.forward(lat, lon) for lat, lon in coords]
        width_m, width_source = _parse_width_m(tags, default_width_m, default_width_source)
        way_counts = Counter()
        way_total = 0.0
        way_in_map = 0.0
        way_covered = 0.0
        way_missing = 0.0

        for source_segment_index, ((x0, y0), (x1, y1)) in enumerate(zip(projected, projected[1:])):
            dx = x1 - x0
            dy = y1 - y0
            length = math.hypot(dx, dy)
            if length <= 0.05:
                counts["zero_length_osm_segment"] += 1
                continue
            piece_count = max(1, int(math.ceil(length / max_segment_m)))
            heading_deg = math.degrees(math.atan2(dy, dx))
            for piece_index in range(piece_count):
                t0 = piece_index / piece_count
                t1 = (piece_index + 1) / piece_count
                ax = x0 + dx * t0
                ay = y0 + dy * t0
                bx = x0 + dx * t1
                by = y0 + dy * t1
                piece_length = math.hypot(bx - ax, by - ay)
                mx = 0.5 * (ax + bx)
                my = 0.5 * (ay + by)
                total_length_m += piece_length
                way_total += piece_length
                counts["explicit_osm_piece_count"] += 1
                way_counts["piece_count"] += 1

                road = _nearest_road_sample(
                    mx,
                    my,
                    heading_deg,
                    road_samples,
                    road_grid,
                    road_cell,
                    max_road_distance_m,
                )
                if road is None:
                    counts["outside_generated_road_area_piece_count"] += 1
                    way_counts["outside_generated_road_area_piece_count"] += 1
                    continue

                in_map_length_m += piece_length
                way_in_map += piece_length
                counts["in_generated_road_area_piece_count"] += 1
                way_counts["in_generated_road_area_piece_count"] += 1
                match = _nearest_xodr_sidewalk_match(
                    mx,
                    my,
                    heading_deg,
                    xodr_sidewalk_segments,
                    sidewalk_grid,
                    sidewalk_cell,
                    coverage_distance_m,
                    coverage_heading_error_deg,
                )
                if match is not None:
                    covered_length_m += piece_length
                    way_covered += piece_length
                    counts["covered_by_xodr_piece_count"] += 1
                    way_counts["covered_by_xodr_piece_count"] += 1
                    continue

                missing_length_m += piece_length
                way_missing += piece_length
                counts["missing_from_xodr_piece_count"] += 1
                way_counts["missing_from_xodr_piece_count"] += 1
                missing_segments.append(
                    {
                        "source": "osm_missing_sidewalk",
                        "osm_way_id": way_id,
                        "source_segment_index": source_segment_index,
                        "piece_index": piece_index,
                        "center_x_m": mx,
                        "center_y_m": my,
                        "center_z_m": float(road["center_z_m"]),
                        "heading_deg": heading_deg,
                        "length_m": max(0.1, piece_length),
                        "width_m": width_m,
                        "width_source": width_source,
                        "surface": tags.get("surface", ""),
                        "bicycle": tags.get("bicycle", ""),
                        "lit": tags.get("lit", ""),
                        "road_id": road.get("road_id", ""),
                        "road_name": road.get("road_name", ""),
                        "road_distance_m": float(road["distance_m"]),
                        "road_heading_error_deg": float(road["heading_error_deg"]),
                    }
                )

        way_summaries.append(
            {
                "osm_way_id": way_id,
                "total_length_m": way_total,
                "in_generated_road_area_length_m": way_in_map,
                "covered_by_xodr_length_m": way_covered,
                "missing_from_xodr_length_m": way_missing,
                "coverage_ratio_in_generated_area": (
                    way_covered / way_in_map if way_in_map > 1e-9 else None
                ),
                "surface": tags.get("surface", ""),
                "width_m": width_m,
                "width_source": width_source,
                "counts": dict(way_counts),
            }
        )

    summary = {
        "available": True,
        "explicit_sidewalk_way_count": len(explicit_ways),
        "explicit_sidewalk_total_length_m": total_length_m,
        "explicit_sidewalk_in_generated_road_area_length_m": in_map_length_m,
        "explicit_sidewalk_covered_by_xodr_length_m": covered_length_m,
        "explicit_sidewalk_missing_from_xodr_length_m": missing_length_m,
        "explicit_sidewalk_coverage_ratio_in_generated_area": (
            covered_length_m / in_map_length_m if in_map_length_m > 1e-9 else None
        ),
        "missing_segment_count": len(missing_segments),
        "coverage_distance_m": coverage_distance_m,
        "coverage_heading_error_deg": coverage_heading_error_deg,
        "max_road_distance_m": max_road_distance_m,
        "default_width_m": default_width_m,
        "default_width_source": default_width_source,
        "counts": dict(counts),
        "way_summaries": way_summaries,
    }
    return summary, missing_segments


def detect_dtt_sidewalk_assets(map_dir: Path) -> Dict[str, object]:
    matches = []
    if map_dir.exists():
        for path in map_dir.rglob("*.uasset"):
            folded = str(path).replace("\\", "/").lower()
            if "sidewalk" in folded or "side_walk" in folded:
                matches.append(path)
    return {
        "map_dir": str(map_dir),
        "dtt_sidewalk_asset_count": len(matches),
        "dtt_sidewalk_asset_samples": [str(path) for path in matches[:50]],
    }


def _dedupe_fallback_segments(segments: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    """Drop near-identical fallbacks while preserving distinct road sides."""
    kept: List[Dict[str, object]] = []
    grid: Dict[Tuple[int, int], List[int]] = defaultdict(list)
    cell = 1.5
    for segment in segments:
        x = float(segment["center_x_m"])
        y = float(segment["center_y_m"])
        heading = float(segment["heading_deg"])
        duplicate = False
        for index in _grid_candidates(x, y, grid, cell, 1.2):
            other = kept[index]
            distance = math.hypot(
                x - float(other["center_x_m"]),
                y - float(other["center_y_m"]),
            )
            if distance <= 0.8 and _heading_error_deg(heading, float(other["heading_deg"])) <= 8.0:
                duplicate = True
                break
        if duplicate:
            continue
        kept.append(segment)
        grid[_grid_key(x, y, cell)].append(len(kept) - 1)
    return kept


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--osm", required=True, type=Path)
    parser.add_argument("--xodr", required=True, type=Path)
    parser.add_argument("--map-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-segment-m", type=float, default=4.0)
    parser.add_argument("--coverage-distance-m", type=float, default=5.5)
    parser.add_argument("--coverage-heading-error-deg", type=float, default=35.0)
    parser.add_argument("--max-road-distance-m", type=float, default=30.0)
    parser.add_argument(
        "--default-width-m",
        type=float,
        default=0.0,
        help=(
            "Fallback width when OSM width=* is absent. 0=auto: use the "
            "median surviving XODR SideWalk width, then 1.8 m if unavailable."
        ),
    )
    args = parser.parse_args()

    if not args.xodr.is_file():
        raise SystemExit("OpenDRIVE not found: {}".format(args.xodr))
    if not 1.0 <= args.max_segment_m <= 12.0:
        raise SystemExit("--max-segment-m must be in [1, 12]")
    if not 1.0 <= args.coverage_distance_m <= 12.0:
        raise SystemExit("--coverage-distance-m must be in [1, 12]")
    if not 5.0 <= args.coverage_heading_error_deg <= 90.0:
        raise SystemExit("--coverage-heading-error-deg must be in [5, 90]")
    if not 5.0 <= args.max_road_distance_m <= 80.0:
        raise SystemExit("--max-road-distance-m must be in [5, 80]")
    if args.default_width_m != 0.0 and not 0.8 <= args.default_width_m <= 5.0:
        raise SystemExit("--default-width-m must be 0(auto) or in [0.8, 5]")

    xodr_root = ET.parse(str(args.xodr)).getroot()
    projection, projection_parameters = projection_from_xodr(xodr_root)
    xodr_summary, xodr_segments = build_xodr_sidewalk_plan(
        xodr_root,
        args.max_segment_m,
    )
    osm_summary = audit_osm(args.osm)
    xodr_median_width = xodr_summary.get("xodr_sidewalk_width_median_m")
    if args.default_width_m > 0.0:
        effective_default_width_m = float(args.default_width_m)
        effective_default_width_source = "configured_default"
    elif xodr_median_width is not None and 0.8 <= float(xodr_median_width) <= 5.0:
        effective_default_width_m = float(xodr_median_width)
        effective_default_width_source = "xodr_sidewalk_median"
    else:
        effective_default_width_m = 1.8
        effective_default_width_source = "release_default"

    osm_fidelity, osm_missing_segments = build_osm_missing_sidewalk_plan(
        args.osm,
        projection,
        xodr_root,
        xodr_segments,
        args.max_segment_m,
        args.coverage_distance_m,
        args.coverage_heading_error_deg,
        args.max_road_distance_m,
        effective_default_width_m,
        effective_default_width_source,
    )
    dtt_summary = detect_dtt_sidewalk_assets(args.map_dir)

    xodr_count = int(xodr_summary["xodr_sidewalk_lane_count"])
    dtt_count = int(dtt_summary["dtt_sidewalk_asset_count"])
    xodr_mesh_fallback_required = xodr_count > 0 and dtt_count == 0
    xodr_mesh_fallback_segments = xodr_segments if xodr_mesh_fallback_required else []
    fallback_segments = _dedupe_fallback_segments(
        list(xodr_mesh_fallback_segments) + list(osm_missing_segments)
    )
    fallback_required = bool(fallback_segments)

    source_counts = Counter(str(item.get("source", "unknown")) for item in fallback_segments)
    payload = {
        "release_version": RELEASE_VERSION,
        "map_name": args.map_name,
        "created_at": time.time(),
        "osm_path": str(args.osm),
        "xodr_path": str(args.xodr),
        "projection": projection_parameters,
        "osm": osm_summary,
        "xodr": xodr_summary,
        "osm_fidelity": osm_fidelity,
        "dtt": dtt_summary,
        "xodr_mesh_fallback_required": xodr_mesh_fallback_required,
        "osm_missing_sidewalk_fallback_required": bool(osm_missing_segments),
        "fallback_required": fallback_required,
        "fallback_segment_count": len(fallback_segments),
        "fallback_source_counts": dict(source_counts),
        "checks": {
            "xodr_has_sidewalk_geometry": xodr_count > 0,
            "dtt_has_sidewalk_assets": dtt_count > 0,
            "explicit_osm_sidewalks_audited": bool(osm_fidelity.get("available")),
            "missing_osm_sidewalks_have_fallback": (
                int(osm_fidelity.get("missing_segment_count", 0) or 0) ==
                int(source_counts.get("osm_missing_sidewalk", 0))
            ),
        },
        "fallback_segments": fallback_segments,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    print("SIDEWALK_OSM_FOOTWAY_SIDEWALK_WAYS={}".format(osm_summary.get("footway_sidewalk_way_count", 0)))
    print("SIDEWALK_OSM_IN_MAP_LENGTH_M={:.3f}".format(float(osm_fidelity.get("explicit_sidewalk_in_generated_road_area_length_m", 0.0) or 0.0)))
    print("SIDEWALK_OSM_COVERED_BY_XODR_LENGTH_M={:.3f}".format(float(osm_fidelity.get("explicit_sidewalk_covered_by_xodr_length_m", 0.0) or 0.0)))
    print("SIDEWALK_OSM_MISSING_FROM_XODR_LENGTH_M={:.3f}".format(float(osm_fidelity.get("explicit_sidewalk_missing_from_xodr_length_m", 0.0) or 0.0)))
    print("SIDEWALK_XODR_LANES={}".format(xodr_count))
    print("SIDEWALK_XODR_TOTAL_LENGTH_M={:.3f}".format(float(xodr_summary["xodr_sidewalk_lane_length_m"])))
    print("SIDEWALK_DTT_ASSETS={}".format(dtt_count))
    print("SIDEWALK_FALLBACK_SEGMENTS={}".format(len(fallback_segments)))
    print("SIDEWALK_FALLBACK_REQUIRED={}".format(1 if fallback_required else 0))
    print("SIDEWALK_PLAN={}".format(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
