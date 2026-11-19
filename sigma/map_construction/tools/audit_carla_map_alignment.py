#!/usr/bin/env python3
"""Audit OpenDRIVE, CARLA driving waypoints, spawn points, and road meshes."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict, deque
from pathlib import Path


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[int(round((len(ordered) - 1) * fraction))]


def yaw_error(first, second):
    return abs((float(first) - float(second) + 180.0) % 360.0 - 180.0)


def distance_2d(first, second):
    return math.hypot(float(first.x - second.x), float(first.y - second.y))


def lane_key(waypoint):
    return (
        int(waypoint.road_id),
        int(waypoint.section_id),
        int(waypoint.lane_id),
    )


def waypoint_key(waypoint):
    return lane_key(waypoint) + (round(float(waypoint.s), 1),)


def parse_georeference(text):
    result = {}
    for name in ("lat_0", "lon_0", "x_0", "y_0"):
        match = re.search(r"(?:^|\s)\+{}=([^\s<]+)".format(name), text or "")
        if match:
            result[name] = float(match.group(1))
    return result


def geodetic_error_m(lat_a, lon_a, lat_b, lon_b):
    north = (lat_a - lat_b) * 111320.0
    east = (lon_a - lon_b) * 111320.0 * math.cos(math.radians((lat_a + lat_b) / 2.0))
    return math.hypot(east, north)


def weak_component_sizes(nodes, edges):
    neighbors = defaultdict(set)
    for node in nodes:
        neighbors[node]
    for source, target in edges:
        neighbors[source].add(target)
        neighbors[target].add(source)
    remaining = set(nodes)
    sizes = []
    while remaining:
        root = remaining.pop()
        queue = [root]
        size = 0
        while queue:
            node = queue.pop()
            size += 1
            for neighbor in neighbors[node]:
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
        sizes.append(size)
    return sorted(sizes, reverse=True)


def static_xodr_audit(xodr_path, config):
    root = ET.parse(str(xodr_path)).getroot()
    roads = root.findall("road")
    junctions = root.findall("junction")
    road_id_counts = Counter(road.get("id") for road in roads)
    junction_id_counts = Counter(junction.get("id") for junction in junctions)
    road_ids = set(road_id_counts)
    junction_ids = set(junction_id_counts)
    header = root.find("header")
    georef_text = "" if header is None else (header.findtext("geoReference") or "")
    georef = parse_georeference(georef_text)

    expected_lat = float(config.get("xodr_origin_latitude", config["min_latitude"]))
    expected_lon = float(config.get("xodr_origin_longitude", config["min_longitude"]))
    origin_error = None
    if "lat_0" in georef and "lon_0" in georef:
        origin_error = geodetic_error_m(
            georef["lat_0"], georef["lon_0"], expected_lat, expected_lon
        )

    widths = []
    driving_lanes = []
    lanes_without_links = 0
    for lane in root.findall('.//lane[@type="driving"]'):
        driving_lanes.append(lane)
        lane_widths = lane.findall("width")
        widths.extend(float(width.get("a", "0")) for width in lane_widths)
        if lane.find("link") is None:
            lanes_without_links += 1

    invalid_road_refs = []
    road_edges = []
    for road in roads:
        road_id = road.get("id")
        link = road.find("link")
        if link is None:
            continue
        for direction in ("predecessor", "successor"):
            ref = link.find(direction)
            if ref is None:
                continue
            element_type = ref.get("elementType")
            element_id = ref.get("elementId")
            valid_ids = road_ids if element_type == "road" else junction_ids
            if element_type not in ("road", "junction") or element_id not in valid_ids:
                invalid_road_refs.append(
                    {"road_id": road_id, "direction": direction,
                     "element_type": element_type, "element_id": element_id}
                )
            elif element_type == "road":
                road_edges.append((road_id, element_id))

    road_by_id = {road.get("id"): road for road in roads}
    lane_id_gaps = []
    for road in roads:
        for section_index, section in enumerate(road.findall("./lanes/laneSection")):
            for side_name in ("left", "right"):
                side = section.find(side_name)
                if side is None:
                    continue
                lane_ids = sorted(abs(int(lane.get("id"))) for lane in side.findall("lane"))
                expected = list(range(1, len(lane_ids) + 1))
                if lane_ids != expected:
                    lane_id_gaps.append({
                        "road_id": road.get("id"), "section_index": section_index,
                        "side": side_name, "lane_ids": lane_ids, "expected": expected,
                    })

    def road_lane_ids(road_id):
        road = road_by_id.get(road_id)
        return set() if road is None else {
            int(lane.get("id")) for lane in road.findall(".//lane")
        }

    invalid_junction_refs = []
    invalid_junction_lane_refs = []
    for junction in junctions:
        for connection in junction.findall("connection"):
            incoming = connection.get("incomingRoad")
            connecting = connection.get("connectingRoad")
            if incoming not in road_ids or connecting not in road_ids:
                invalid_junction_refs.append(
                    {"junction_id": junction.get("id"), "incoming_road": incoming,
                     "connecting_road": connecting}
                )
                continue
            road_edges.append((incoming, connecting))
            incoming_lanes = road_lane_ids(incoming)
            connecting_lanes = road_lane_ids(connecting)
            for lane_link in connection.findall("laneLink"):
                from_lane = int(lane_link.get("from"))
                to_lane = int(lane_link.get("to"))
                if from_lane not in incoming_lanes or to_lane not in connecting_lanes:
                    invalid_junction_lane_refs.append({
                        "junction_id": junction.get("id"),
                        "connection_id": connection.get("id"),
                        "incoming_road": incoming,
                        "connecting_road": connecting,
                        "from_lane": from_lane,
                        "to_lane": to_lane,
                    })

    components = weak_component_sizes(road_ids, road_edges)
    checks = {
        "georeference_present": "lat_0" in georef and "lon_0" in georef,
        "georeference_matches_bbox_origin": origin_error is not None and origin_error <= 1.0,
        "driving_lanes_present": bool(driving_lanes),
        "driving_lane_width_at_least_2_5m": bool(widths) and min(widths) >= 2.5,
        "road_ids_unique": all(count == 1 for count in road_id_counts.values()),
        "junction_ids_unique": all(count == 1 for count in junction_id_counts.values()),
        "lane_ids_contiguous_per_side": not lane_id_gaps,
        "road_references_valid": not invalid_road_refs,
        "junction_references_valid": not invalid_junction_refs,
        "junction_lane_references_valid": not invalid_junction_lane_refs,
    }
    return {
        "xodr_path": str(xodr_path),
        "success": all(checks.values()),
        "checks": checks,
        "header_bounds": dict(header.attrib) if header is not None else {},
        "georeference": georef,
        "georeference_text": georef_text.strip(),
        "expected_origin": {"latitude": expected_lat, "longitude": expected_lon},
        "georeference_origin_error_m": origin_error,
        "road_count": len(roads),
        "junction_count": len(junctions),
        "driving_lane_count": len(driving_lanes),
        "driving_lane_width_m": {
            "minimum": min(widths) if widths else None,
            "maximum": max(widths) if widths else None,
            "mean": statistics.mean(widths) if widths else None,
        },
        "driving_lanes_without_lane_link": lanes_without_links,
        "duplicate_road_ids": [key for key, count in road_id_counts.items() if count > 1],
        "duplicate_junction_ids": [key for key, count in junction_id_counts.items() if count > 1],
        "lane_id_gaps": lane_id_gaps,
        "invalid_road_references": invalid_road_refs,
        "invalid_junction_references": invalid_junction_refs,
        "invalid_junction_lane_references": invalid_junction_lane_refs,
        "road_graph_weak_component_count": len(components),
        "road_graph_weak_component_sizes": components,
    }


def sample_xodr_references(xodr_path, interval=3.0):
    root = ET.parse(str(xodr_path)).getroot()
    lines = []
    for road in root.findall("road"):
        points = []
        for geometry in road.findall("./planView/geometry"):
            x0 = float(geometry.get("x", "0"))
            y0 = float(geometry.get("y", "0"))
            heading = float(geometry.get("hdg", "0"))
            length = float(geometry.get("length", "0"))
            count = max(2, int(math.ceil(length / interval)) + 1)
            child = next(iter(geometry), None)
            for index in range(count):
                arc = min(length, index * length / max(1, count - 1))
                u = arc
                v = 0.0
                local_heading = heading
                if child is not None and child.tag == "arc":
                    curvature = float(child.get("curvature", "0"))
                    if abs(curvature) > 1e-12:
                        u = math.sin(curvature * arc) / curvature
                        v = (1.0 - math.cos(curvature * arc)) / curvature
                elif child is not None and child.tag == "paramPoly3":
                    parameter = arc
                    if child.get("pRange", "normalized") == "normalized" and length > 0:
                        parameter = arc / length
                    u = sum(float(child.get(name, "0")) * parameter ** power for power, name in enumerate(("aU", "bU", "cU", "dU")))
                    v = sum(float(child.get(name, "0")) * parameter ** power for power, name in enumerate(("aV", "bV", "cV", "dV")))
                elif child is not None and child.tag not in ("line", "arc", "paramPoly3"):
                    continue
                x = x0 + u * math.cos(local_heading) - v * math.sin(local_heading)
                opendrive_y = y0 + u * math.sin(local_heading) + v * math.cos(local_heading)
                points.append((x, -opendrive_y))
        if points:
            lines.append(points)
    return lines


def bbox_corners_xy(bbox):
    angle = math.radians(float(bbox.rotation.yaw))
    cosine = math.cos(angle)
    sine = math.sin(angle)
    corners = []
    for local_x, local_y in (
        (-float(bbox.extent.x), -float(bbox.extent.y)),
        (float(bbox.extent.x), -float(bbox.extent.y)),
        (float(bbox.extent.x), float(bbox.extent.y)),
        (-float(bbox.extent.x), float(bbox.extent.y)),
    ):
        corners.append((
            float(bbox.location.x) + local_x * cosine - local_y * sine,
            float(bbox.location.y) + local_x * sine + local_y * cosine,
        ))
    return corners


def point_in_bbox_xy(location, bbox, margin=0.35):
    dx = float(location.x - bbox.location.x)
    dy = float(location.y - bbox.location.y)
    angle = math.radians(-float(bbox.rotation.yaw))
    local_x = dx * math.cos(angle) - dy * math.sin(angle)
    local_y = dx * math.sin(angle) + dy * math.cos(angle)
    return (
        abs(local_x) <= float(bbox.extent.x) + margin
        and abs(local_y) <= float(bbox.extent.y) + margin
    )


def trace_forward(waypoint, target_distance, step_distance, max_states=800):
    queue = deque([(waypoint, 0.0, float(waypoint.transform.rotation.yaw), 0)])
    best_by_key = {waypoint_key(waypoint): 0.0}
    maximum = 0.0
    maximum_yaw_jump = 0.0
    shortest_dead_end = None
    reached = False
    explored = 0
    while queue and explored < max_states:
        current, travelled, previous_yaw, depth = queue.popleft()
        explored += 1
        maximum = max(maximum, travelled)
        if travelled >= target_distance:
            reached = True
            continue
        if depth >= int(math.ceil(target_distance / step_distance)) + 8:
            continue
        next_items = list(current.next(step_distance))
        if not next_items:
            shortest_dead_end = travelled if shortest_dead_end is None else min(shortest_dead_end, travelled)
            continue
        for item in next_items:
            maximum_yaw_jump = max(
                maximum_yaw_jump,
                yaw_error(item.transform.rotation.yaw, previous_yaw),
            )
            segment = distance_2d(current.transform.location, item.transform.location)
            next_distance = travelled + (segment if segment > 0.05 else step_distance)
            key = waypoint_key(item)
            if best_by_key.get(key, float("inf")) <= next_distance:
                continue
            best_by_key[key] = next_distance
            queue.append((item, next_distance, float(item.transform.rotation.yaw), depth + 1))
    return {
        "maximum_distance_m": min(maximum, target_distance),
        "reaches_target": reached,
        "shortest_dead_end_m": shortest_dead_end,
        "maximum_step_yaw_jump_deg": maximum_yaw_jump,
        "all_observed_branches_reach_target": reached and shortest_dead_end is None,
        "states_explored": explored,
        "state_limit_reached": bool(queue),
    }


def route_location(carla, config, latitude, longitude):
    origin_latitude = float(
        config.get("xodr_origin_latitude", config["min_latitude"])
    )
    origin_longitude = float(
        config.get("xodr_origin_longitude", config["min_longitude"])
    )
    mean_latitude = math.radians((float(latitude) + origin_latitude) / 2.0)
    x = (
        (float(longitude) - origin_longitude)
        * 111320.0
        * math.cos(mean_latitude)
    )
    y = -(float(latitude) - origin_latitude) * 111320.0
    return carla.Location(x=x, y=y, z=0.0)


def directional_waypoint(waypoints, location, desired_yaw, radius=20.0):
    candidates = [
        waypoint for waypoint in waypoints
        if distance_2d(waypoint.transform.location, location) <= radius
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda waypoint: (
            distance_2d(waypoint.transform.location, location)
            + 0.25 * yaw_error(waypoint.transform.rotation.yaw, desired_yaw)
        ),
    )


def route_reachable(start_wp, end_location, step_distance=5.0, tolerance=12.0, max_distance=3000.0):
    queue = deque([(start_wp, 0.0)])
    best = {waypoint_key(start_wp): 0.0}
    nearest = distance_2d(start_wp.transform.location, end_location)
    explored = 0
    while queue and explored < 100000:
        waypoint, travelled = queue.popleft()
        explored += 1
        current_distance = distance_2d(waypoint.transform.location, end_location)
        nearest = min(nearest, current_distance)
        if current_distance <= tolerance:
            return True, travelled, nearest, explored
        if travelled >= max_distance:
            continue
        for item in waypoint.next(step_distance):
            segment = distance_2d(waypoint.transform.location, item.transform.location)
            next_distance = travelled + (segment if segment > 0.05 else step_distance)
            key = waypoint_key(item)
            if best.get(key, float("inf")) <= next_distance:
                continue
            best[key] = next_distance
            queue.append((item, next_distance))
    return False, None, nearest, explored


def runtime_audit(args, config, xodr_path, output_dir):
    import carla

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    carla_map = world.get_map()
    spawns = list(carla_map.get_spawn_points())
    waypoints = list(carla_map.generate_waypoints(args.waypoint_step))
    topology = list(carla_map.get_topology())
    road_bboxes = list(world.get_level_bbs(carla.CityObjectLabel.Roads))

    rows = []
    distances = []
    yaw_errors = []
    for index, spawn in enumerate(spawns):
        nearest = carla_map.get_waypoint(
            spawn.location, project_to_road=True, lane_type=carla.LaneType.Driving
        )
        if nearest is None:
            row = {"spawn_index": index, "x": spawn.location.x, "y": spawn.location.y,
                   "z": spawn.location.z, "yaw": spawn.rotation.yaw,
                   "nearest_waypoint_x": None, "nearest_waypoint_y": None,
                   "lane_center_distance": None, "road_id": None, "lane_id": None,
                   "lane_type": None, "lane_width_m": None, "yaw_error_deg": None,
                   "next_chain_length_m": 0.0, "shortest_dead_end_m": 0.0,
                   "previous_exists": False, "mesh_covered": False, "valid": False,
                   "invalid_reasons": "no_driving_waypoint"}
            rows.append(row)
            continue
        center_distance = distance_2d(spawn.location, nearest.transform.location)
        heading_error = yaw_error(spawn.rotation.yaw, nearest.transform.rotation.yaw)
        trace = trace_forward(nearest, args.chain_distance, args.chain_step)
        previous_exists = bool(nearest.previous(args.chain_step))
        mesh_covered = any(point_in_bbox_xy(nearest.transform.location, bbox) for bbox in road_bboxes)
        reasons = []
        if center_distance > args.max_lane_distance:
            reasons.append("lane_center_distance")
        if heading_error > args.max_yaw_error:
            reasons.append("yaw_error")
        if not trace["reaches_target"]:
            reasons.append("next_chain_short")
        if not previous_exists:
            reasons.append("previous_empty")
        if not mesh_covered:
            reasons.append("outside_road_mesh_bounds")
        distances.append(center_distance)
        yaw_errors.append(heading_error)
        rows.append({
            "spawn_index": index, "x": float(spawn.location.x), "y": float(spawn.location.y),
            "z": float(spawn.location.z), "yaw": float(spawn.rotation.yaw),
            "nearest_waypoint_x": float(nearest.transform.location.x),
            "nearest_waypoint_y": float(nearest.transform.location.y),
            "lane_center_distance": center_distance, "road_id": int(nearest.road_id),
            "lane_id": int(nearest.lane_id), "lane_type": str(nearest.lane_type),
            "lane_width_m": float(nearest.lane_width), "yaw_error_deg": heading_error,
            "next_chain_length_m": trace["maximum_distance_m"],
            "shortest_dead_end_m": trace["shortest_dead_end_m"],
            "maximum_step_yaw_jump_deg": trace["maximum_step_yaw_jump_deg"],
            "all_observed_branches_reach_target": trace["all_observed_branches_reach_target"],
            "previous_exists": previous_exists, "mesh_covered": mesh_covered,
            "valid": not reasons, "invalid_reasons": ";".join(reasons),
        })

    topology_nodes = set()
    topology_edges = set()
    for start, end in topology:
        source = lane_key(start)
        target = lane_key(end)
        topology_nodes.update((source, target))
        topology_edges.add((source, target))
        for continuation in end.next(0.5):
            topology_nodes.add(lane_key(continuation))
            topology_edges.add((target, lane_key(continuation)))
    component_sizes = weak_component_sizes(topology_nodes, topology_edges)

    route_reports = []
    route_config = config.get("map_audit_routes", {})
    for name, route in sorted(route_config.items()):
        start_location = route_location(carla, config, route["start_latitude"], route["start_longitude"])
        end_location = route_location(carla, config, route["end_latitude"], route["end_longitude"])
        desired_yaw = math.degrees(math.atan2(
            end_location.y - start_location.y,
            end_location.x - start_location.x,
        ))
        start_wp = directional_waypoint(waypoints, start_location, desired_yaw)
        end_wp = directional_waypoint(waypoints, end_location, desired_yaw)
        candidates = sorted(rows, key=lambda row: math.hypot(row["x"] - start_location.x, row["y"] - start_location.y))
        nearest_valid = next((row for row in candidates if row["valid"]), None)
        reachable, route_length, nearest_end, explored = (False, None, None, 0)
        if start_wp is not None and end_wp is not None:
            reachable, route_length, nearest_end, explored = route_reachable(start_wp, end_wp.transform.location)
        route_reports.append({
            "name": name,
            "start": {"x": start_location.x, "y": start_location.y,
                      "road_id": None if start_wp is None else int(start_wp.road_id),
                      "lane_id": None if start_wp is None else int(start_wp.lane_id)},
            "end": {"x": end_location.x, "y": end_location.y,
                    "road_id": None if end_wp is None else int(end_wp.road_id),
                    "lane_id": None if end_wp is None else int(end_wp.lane_id)},
            "reachable": reachable, "route_length_m": route_length,
            "nearest_end_distance_m": nearest_end, "states_explored": explored,
            "nearest_valid_spawn_index": None if nearest_valid is None else nearest_valid["spawn_index"],
            "nearest_valid_spawn_distance_m": None if nearest_valid is None else math.hypot(nearest_valid["x"] - start_location.x, nearest_valid["y"] - start_location.y),
        })

    csv_path = output_dir / "spawn_point_audit.csv"
    fieldnames = list(rows[0].keys()) if rows else ["spawn_index"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    outside_waypoints = sum(
        not any(point_in_bbox_xy(waypoint.transform.location, bbox) for bbox in road_bboxes)
        for waypoint in waypoints
    )
    reason_counts = Counter(
        reason for row in rows for reason in str(row["invalid_reasons"]).split(";") if reason
    )
    valid_spawn_count = sum(bool(row["valid"]) for row in rows)
    checks = {
        "runtime_map_name_matches": carla_map.name.rstrip("/").endswith("/" + config["map_name"]),
        "spawn_points_present": bool(rows),
        "all_spawns_near_driving_lane": not reason_counts["lane_center_distance"] and not reason_counts["no_driving_waypoint"],
        "all_spawn_yaws_match_lane": not reason_counts["yaw_error"],
        "all_spawns_inside_road_mesh_bounds": not reason_counts["outside_road_mesh_bounds"],
        "enough_safe_spawns_for_requested_traffic": valid_spawn_count >= int(config.get("vehicle_count", 1)) + 1,
        "configured_routes_reachable": all(item["reachable"] for item in route_reports),
    }
    diagnostics = {
        "all_spawns_have_100m_chain": not reason_counts["next_chain_short"],
        "all_spawns_have_previous_waypoint": not reason_counts["previous_empty"],
        "topology_is_single_weak_component": len(component_sizes) == 1,
    }
    runtime = {
        "success": all(checks.values()), "checks": checks,
        "diagnostics": diagnostics,
        "map_name": carla_map.name, "spawn_point_count": len(rows),
        "topology_edge_count": len(topology), "driving_waypoint_count": len(waypoints),
        "road_mesh_bbox_count": len(road_bboxes),
        "spawn_lane_center_distance_m": {"maximum": max(distances) if distances else None,
                                         "mean": statistics.mean(distances) if distances else None,
                                         "p95": percentile(distances, 0.95)},
        "spawn_yaw_error_deg": {"maximum": max(yaw_errors) if yaw_errors else None,
                                "mean": statistics.mean(yaw_errors) if yaw_errors else None,
                                "p95": percentile(yaw_errors, 0.95)},
        "valid_spawn_count": valid_spawn_count,
        "valid_spawn_indices": [row["spawn_index"] for row in rows if row["valid"]],
        "invalid_spawn_count": sum(not bool(row["valid"]) for row in rows),
        "invalid_reason_counts": dict(reason_counts),
        "driving_waypoints_outside_road_mesh_bounds": outside_waypoints,
        "topology_lane_node_count": len(topology_nodes),
        "topology_weak_component_count": len(component_sizes),
        "topology_weak_component_sizes": component_sizes,
        "routes": route_reports,
        "csv_path": str(csv_path),
        "spawns": rows,
        "invalid_spawns": [row for row in rows if not row["valid"]],
    }
    alignment_overlay_path = output_dir / "map_alignment_overlay.png"
    spawn_overlay_path = output_dir / "spawn_point_overlay.png"
    plot_overlay(
        alignment_overlay_path,
        xodr_path,
        road_bboxes,
        waypoints,
        rows,
        route_reports,
    )
    plot_overlay(
        spawn_overlay_path,
        xodr_path,
        road_bboxes,
        waypoints,
        rows,
        route_reports,
        annotate_invalid_spawns=True,
    )
    runtime["overlay_path"] = str(alignment_overlay_path)
    runtime["spawn_overlay_path"] = str(spawn_overlay_path)
    return runtime


def plot_overlay(
        path, xodr_path, road_bboxes, waypoints, rows, routes,
        annotate_invalid_spawns=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    figure, axis = plt.subplots(figsize=(14, 12), dpi=180)
    for bbox in road_bboxes:
        axis.add_patch(Polygon(
            bbox_corners_xy(bbox), closed=True, linewidth=0.25,
            edgecolor="#666666", facecolor="#d0d0d0", alpha=0.35,
        ))
    for points in sample_xodr_references(xodr_path):
        axis.plot([p[0] for p in points], [p[1] for p in points], color="#d62728", linewidth=0.35, alpha=0.65)
    axis.scatter([wp.transform.location.x for wp in waypoints],
                 [wp.transform.location.y for wp in waypoints], s=0.35,
                 color="#1f77b4", alpha=0.42, label="Driving waypoints")
    def category_for(row):
        if row["valid"]:
            return "valid"
        reasons = {
            item
            for item in str(row.get("invalid_reasons", "")).split(";")
            if item
        }
        if reasons & {"no_driving_waypoint", "lane_center_distance"}:
            return "offroad"
        if "outside_road_mesh_bounds" in reasons:
            return "outside_road_mesh_bounds"
        if "yaw_error" in reasons:
            return "yaw_error"
        if "next_chain_short" in reasons:
            return "next_chain_short"
        if "previous_empty" in reasons:
            return "previous_empty"
        return "other_invalid"

    categories = [
        ("valid", "#2ca02c", "Valid spawn", "o", 8),
        ("offroad", "#d62728", "Offroad / no Driving lane", "X", 22),
        ("outside_road_mesh_bounds", "#111111", "Outside road mesh", "s", 20),
        ("yaw_error", "#9467bd", "Yaw mismatch", "D", 18),
        ("next_chain_short", "#ff7f0e", "Forward chain < 100m", "^", 18),
        ("previous_empty", "#17becf", "No previous waypoint", "v", 18),
        ("other_invalid", "#7f7f7f", "Other invalid spawn", "P", 18),
    ]
    categorized = {key: [] for key, _c, _l, _m, _s in categories}
    for row in rows:
        categorized[category_for(row)].append(row)
    for key, color, label, marker, size in categories:
        selected = categorized[key]
        if selected:
            axis.scatter(
                [row["x"] for row in selected],
                [row["y"] for row in selected],
                s=size,
                color=color,
                marker=marker,
                label="{} ({})".format(label, len(selected)),
                zorder=5,
            )

    if annotate_invalid_spawns:
        for row in rows:
            if row["valid"]:
                continue
            axis.annotate(
                str(row["spawn_index"]),
                (row["x"], row["y"]),
                xytext=(2, 2),
                textcoords="offset points",
                fontsize=3.5,
                color="#111111",
                zorder=7,
            )
    for route in routes:
        axis.scatter([route["start"]["x"]], [route["start"]["y"]], marker="*", s=65,
                     color="#17becf", edgecolor="black", linewidth=0.3, zorder=6)
        axis.scatter([route["end"]["x"]], [route["end"]["y"]], marker="X", s=40,
                     color="#17becf", edgecolor="black", linewidth=0.3, zorder=6)
    axis.set_aspect("equal", adjustable="box")
    axis.invert_yaxis()
    title = "CARLA road mesh / OpenDRIVE / waypoint / spawn alignment (north up)"
    if annotate_invalid_spawns:
        title += " - invalid spawn indices labeled"
    axis.set_title(title)
    axis.set_xlabel("CARLA X (east, m)")
    axis.set_ylabel("CARLA Y (south, m; inverted for north-up)")
    axis.legend(loc="best", fontsize=7, markerscale=1.5)
    axis.grid(True, linewidth=0.2, alpha=0.25)
    figure.tight_layout()
    figure.savefig(str(path))
    plt.close(figure)


def find_xodr(config):
    map_name = config["map_name"]
    configured = config.get("xodr_path")
    if configured:
        return Path(configured)
    root = Path(config["carla_root"]) / "Unreal" / "CarlaUE4" / "Content" / "CustomMaps" / map_name
    candidates = sorted(root.rglob("*.xodr"))
    if not candidates:
        raise FileNotFoundError("No xodr found below {}".format(root))
    exact = [path for path in candidates if path.stem == map_name]
    return exact[0] if exact else candidates[0]


def export_driving_lane_samples(config, xodr_path, output_path, step_m):
    import carla

    carla_map = carla.Map(
        str(config["map_name"]),
        xodr_path.read_text(encoding="utf-8"),
    )
    samples = []
    for waypoint in carla_map.generate_waypoints(float(step_m)):
        if waypoint.lane_type != carla.LaneType.Driving:
            continue
        location = waypoint.transform.location
        samples.append({
            "x": float(location.x),
            "y": float(location.y),
            "z": float(location.z),
            "lane_width_m": float(waypoint.lane_width),
            "road_id": int(waypoint.road_id),
            "section_id": int(waypoint.section_id),
            "lane_id": int(waypoint.lane_id),
            "s": float(waypoint.s),
        })

    if not samples:
        raise RuntimeError("OpenDRIVE produced no Driving lane samples")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(
            {
                "map_name": config["map_name"],
                "xodr_path": str(xodr_path),
                "sample_step_m": float(step_m),
                "sample_count": len(samples),
                "samples": samples,
            },
            separators=(",", ":"),
        ),
        encoding="utf-8",
    )
    return {
        "path": str(output_path),
        "sample_step_m": float(step_m),
        "sample_count": len(samples),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--static-only", action="store_true")
    parser.add_argument("--lane-samples-out")
    parser.add_argument("--lane-sample-step", type=float, default=0.5)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--waypoint-step", type=float, default=2.0)
    parser.add_argument("--chain-distance", type=float, default=100.0)
    parser.add_argument("--chain-step", type=float, default=5.0)
    parser.add_argument("--max-lane-distance", type=float, default=4.0)
    parser.add_argument("--max-yaw-error", type=float, default=30.0)
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    xodr_path = find_xodr(config)
    static = static_xodr_audit(xodr_path, config)
    payload = {"config_path": str(config_path), "static": static, "success": static["success"]}
    if args.lane_samples_out:
        payload["driving_lane_samples"] = export_driving_lane_samples(
            config,
            xodr_path,
            Path(args.lane_samples_out).resolve(),
            args.lane_sample_step,
        )
    if not args.static_only:
        runtime = runtime_audit(args, config, xodr_path, output_dir)
        payload["runtime"] = runtime
        payload["success"] = static["success"] and runtime["success"]
    report_path = output_dir / ("opendrive_static_audit.json" if args.static_only else "map_alignment_audit.json")
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    print("MAP_ALIGNMENT_AUDIT_REPORT={}".format(report_path))
    return 0 if payload["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
