#!/usr/bin/env python3
"""Convert sanitized OSM to a DTT-safe, validated OpenDRIVE file."""

from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
import time
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import carla


METERS_PER_DEGREE_LATITUDE = 111320.0
DTT_TILE_BOUNDARY_EPSILON_M = 5.0
MIN_PRIMARY_TOPOLOGY_FRACTION = 0.95


def dtt_negative_axis_end_extent(extent_m, tile_size_m):
    """Compensate for DTT applying Ceil before Abs on its negative Y axis."""
    if not math.isfinite(extent_m) or extent_m < 0.0:
        raise ValueError("DTT extent must be a finite non-negative value")
    if not math.isfinite(tile_size_m) or tile_size_m <= 0.0:
        raise ValueError("DTT tile size must be a finite positive value")
    tile_count = max(1, int(math.ceil(extent_m / tile_size_m)))
    return tile_count * tile_size_m + DTT_TILE_BOUNDARY_EPSILON_M


def convert(osm_text, latitude, longitude):
    settings = carla.Osm2OdrSettings()
    settings.proj_string = (
        "+proj=tmerc +lat_0={:.9f} +lon_0={:.9f}".format(
            latitude, longitude
        )
    )
    settings.center_map = False
    settings.use_offsets = False
    return carla.Osm2Odr.convert(osm_text, settings)


def bounds(root):
    header = root.find("header")
    if header is None:
        raise ValueError("OpenDRIVE header is missing")
    return {
        name: float(header.get(name))
        for name in ("west", "south", "east", "north")
    }


def remove_dangling_references(root):
    roads = root.findall("road")
    junctions = root.findall("junction")
    road_ids = {road.get("id") for road in roads}
    junction_ids = {junction.get("id") for junction in junctions}
    removed = []

    for road in roads:
        link = road.find("link")
        if link is None:
            continue
        for reference in list(link):
            element_type = reference.get("elementType")
            element_id = reference.get("elementId")
            valid = (
                (element_type == "road" and element_id in road_ids)
                or (element_type == "junction" and element_id in junction_ids)
            )
            if valid:
                continue
            removed.append(
                {
                    "kind": "road_link",
                    "road_id": road.get("id"),
                    "direction": reference.tag,
                    "element_type": element_type,
                    "element_id": element_id,
                }
            )
            link.remove(reference)

    for junction in junctions:
        for connection in list(junction.findall("connection")):
            incoming = connection.get("incomingRoad")
            connecting = connection.get("connectingRoad")
            if incoming in road_ids and connecting in road_ids:
                continue
            removed.append(
                {
                    "kind": "junction_connection",
                    "junction_id": junction.get("id"),
                    "incoming_road": incoming,
                    "connecting_road": connecting,
                }
            )
            junction.remove(connection)
    return removed


def lane_key(waypoint):
    return (
        int(waypoint.road_id),
        int(waypoint.section_id),
        int(waypoint.lane_id),
    )


def driving_topology_components(carla_map):
    neighbors = defaultdict(set)
    nodes = set()
    for start, end in carla_map.get_topology():
        source = lane_key(start)
        target = lane_key(end)
        nodes.update((source, target))
        neighbors[source].add(target)
        neighbors[target].add(source)
        for continuation in end.next(0.5):
            continuation_key = lane_key(continuation)
            nodes.add(continuation_key)
            neighbors[target].add(continuation_key)
            neighbors[continuation_key].add(target)

    remaining = set(nodes)
    components = []
    while remaining:
        seed = remaining.pop()
        component = {seed}
        queue = [seed]
        while queue:
            current = queue.pop()
            for neighbor in neighbors[current]:
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return sorted(components, key=len, reverse=True)


def demote_isolated_driving_lanes(root, components):
    demoted = []
    for component in components[1:]:
        if len(component) != 1:
            continue
        road_id, section_index, lane_id = next(iter(component))
        road = root.find("./road[@id='{}']".format(road_id))
        if road is None:
            continue
        sections = road.findall("./lanes/laneSection")
        if section_index >= len(sections):
            continue
        lane = sections[section_index].find(
            ".//lane[@id='{}']".format(lane_id)
        )
        if lane is None or lane.get("type") != "driving":
            continue
        lane.set("type", "shoulder")
        demoted.append(
            {
                "road_id": road_id,
                "section_id": section_index,
                "lane_id": lane_id,
                "previous_type": "driving",
                "new_type": "shoulder",
                "reason": "isolated_single_lane_topology_component",
            }
        )
    return demoted


def validate_driving_topology_components(components, context):
    """Reject substantial splits while allowing small clipped edge networks."""
    component_sizes = [len(component) for component in components]
    total_size = sum(component_sizes)
    if total_size <= 0:
        raise RuntimeError("CARLA parsed no driving topology from {}".format(context))

    primary_fraction = component_sizes[0] / float(total_size)
    if primary_fraction < MIN_PRIMARY_TOPOLOGY_FRACTION:
        raise RuntimeError(
            "CARLA driving topology has substantial disconnected components "
            "after {}: component sizes={}, primary fraction={:.4f}".format(
                context,
                component_sizes,
                primary_fraction,
            )
        )
    return primary_fraction


def write_atomic(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--osm", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--margin-m", type=float, default=25.0)
    parser.add_argument(
        "--dtt-tile-size-m",
        type=float,
        default=500.0,
        help="DTT tile edge length; official UI default is 500m",
    )
    args = parser.parse_args()

    started = time.time()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    osm_text = args.osm.read_text(encoding="utf-8")
    latitude = float(config["min_latitude"])
    longitude = float(config["min_longitude"])
    attempts = []

    for index in range(4):
        xodr_text = convert(osm_text, latitude, longitude)
        root = ET.fromstring(xodr_text)
        current_bounds = bounds(root)
        attempts.append(
            {
                "index": index + 1,
                "origin_latitude": latitude,
                "origin_longitude": longitude,
                "bounds": current_bounds,
            }
        )
        east_shift = max(0.0, args.margin_m - current_bounds["west"])
        north_shift = max(0.0, args.margin_m - current_bounds["south"])
        if east_shift <= 0.01 and north_shift <= 0.01:
            break
        latitude -= north_shift / METERS_PER_DEGREE_LATITUDE
        meters_per_degree_longitude = (
            METERS_PER_DEGREE_LATITUDE
            * max(0.05, math.cos(math.radians(latitude)))
        )
        longitude -= east_shift / meters_per_degree_longitude
    else:
        raise RuntimeError("OpenDRIVE origin padding did not converge")

    removed = remove_dangling_references(root)
    candidate_text = ET.tostring(root, encoding="unicode") + "\n"
    parsed_map = carla.Map(str(config["map_name"]), candidate_text)
    components_before = driving_topology_components(parsed_map)
    demoted = demote_isolated_driving_lanes(root, components_before)
    final_text = ET.tostring(root, encoding="unicode") + "\n"
    final_bounds = bounds(root)
    if final_bounds["west"] < args.margin_m - 0.1:
        raise RuntimeError("OpenDRIVE west bound remains outside DTT tile grid")
    if final_bounds["south"] < args.margin_m - 0.1:
        raise RuntimeError("OpenDRIVE south bound remains outside DTT tile grid")

    parsed_map = carla.Map(str(config["map_name"]), final_text)
    components_after = driving_topology_components(parsed_map)
    topology_count = len(parsed_map.get_topology())
    waypoint_count = len(parsed_map.generate_waypoints(2.0))
    if topology_count <= 0 or waypoint_count <= 0:
        raise RuntimeError("CARLA parsed no driving topology from prepared xodr")
    primary_topology_fraction = validate_driving_topology_components(
        components_after,
        "isolated-lane repair",
    )

    xodr_bounds_end_geolocation = parsed_map.transform_to_geolocation(
        carla.Location(
            x=final_bounds["east"],
            y=final_bounds["north"],
            z=0.0,
        )
    )
    dtt_required_y_tile_count = max(
        1,
        int(math.ceil(final_bounds["north"] / args.dtt_tile_size_m)),
    )
    dtt_required_x_tile_count = max(
        1,
        int(math.ceil(final_bounds["east"] / args.dtt_tile_size_m)),
    )
    dtt_required_total_tile_count = (
        dtt_required_x_tile_count * dtt_required_y_tile_count
    )
    max_dtt_tiles = config.get("max_dtt_tiles")
    if (
        max_dtt_tiles is not None
        and dtt_required_total_tile_count > int(max_dtt_tiles)
    ):
        raise RuntimeError(
            "Prepared OpenDRIVE requires {} DTT tiles ({}x{}), exceeding "
            "configured max_dtt_tiles={}".format(
                dtt_required_total_tile_count,
                dtt_required_x_tile_count,
                dtt_required_y_tile_count,
                int(max_dtt_tiles),
            )
        )
    dtt_tile_end_north_m = dtt_negative_axis_end_extent(
        final_bounds["north"],
        args.dtt_tile_size_m,
    )
    dtt_expected_y_tile_count = abs(
        int(math.ceil(-dtt_tile_end_north_m / args.dtt_tile_size_m))
    )
    if dtt_expected_y_tile_count != dtt_required_y_tile_count:
        raise RuntimeError(
            "DTT negative-axis tile workaround produced {} rows; "
            "required {}".format(
                dtt_expected_y_tile_count,
                dtt_required_y_tile_count,
            )
        )
    end_geolocation = parsed_map.transform_to_geolocation(
        carla.Location(
            x=final_bounds["east"],
            y=dtt_tile_end_north_m,
            z=0.0,
        )
    )

    write_atomic(args.output.resolve(), final_text)
    config["xodr_origin_latitude"] = latitude
    config["xodr_origin_longitude"] = longitude
    config["xodr_end_latitude"] = float(end_geolocation.latitude)
    config["xodr_end_longitude"] = float(end_geolocation.longitude)
    config["xodr_projection_margin_m"] = float(args.margin_m)
    config["geo_reference"] = {
        "method": "local_tangent_plane",
        "origin": {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": 0.0,
        },
        "axes": {"x": "east", "y": "south"},
        "source": "prepared OpenDRIVE geoReference",
    }
    write_atomic(
        args.config.resolve(),
        json.dumps(config, indent=2, ensure_ascii=False) + "\n",
    )

    report = {
        "success": True,
        "map_name": config["map_name"],
        "osm": str(args.osm.resolve()),
        "output": str(args.output.resolve()),
        "margin_m": float(args.margin_m),
        "attempts": attempts,
        "final_origin": {"latitude": latitude, "longitude": longitude},
        "final_end": {
            "latitude": float(end_geolocation.latitude),
            "longitude": float(end_geolocation.longitude),
        },
        "xodr_bounds_end": {
            "latitude": float(xodr_bounds_end_geolocation.latitude),
            "longitude": float(xodr_bounds_end_geolocation.longitude),
        },
        "dtt_tile_grid": {
            "tile_size_m": float(args.dtt_tile_size_m),
            "required_x_tile_count": dtt_required_x_tile_count,
            "negative_y_extent_m": float(dtt_tile_end_north_m),
            "required_y_tile_count": dtt_required_y_tile_count,
            "required_total_tile_count": dtt_required_total_tile_count,
            "configured_max_tile_count": max_dtt_tiles,
            "expected_y_tile_count": dtt_expected_y_tile_count,
            "workaround": "ceil_before_abs_negative_axis",
        },
        "final_bounds": final_bounds,
        "removed_dangling_references": removed,
        "driving_topology_component_sizes_before": [
            len(component) for component in components_before
        ],
        "demoted_isolated_driving_lanes": demoted,
        "driving_topology_component_sizes_after": [
            len(component) for component in components_after
        ],
        "primary_driving_topology_fraction": primary_topology_fraction,
        "minimum_primary_topology_fraction": MIN_PRIMARY_TOPOLOGY_FRACTION,
        "topology_edge_count": topology_count,
        "driving_waypoint_count_2m": waypoint_count,
        "elapsed_seconds": round(time.time() - started, 3),
    }
    write_atomic(
        args.report.resolve(),
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
