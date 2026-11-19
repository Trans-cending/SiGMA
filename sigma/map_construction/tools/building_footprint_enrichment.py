#!/usr/bin/env python3
"""Conservatively merge Overture-only buildings into an OSM document."""

from __future__ import annotations

import binascii
import json
import math
import struct
import xml.etree.ElementTree as ET
import zlib
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import numpy as np
from shapely.affinity import affine_transform
from shapely.geometry import LineString, Point, Polygon, box, mapping, shape
from shapely.ops import polygonize, unary_union


MIN_BUILDING_AREA_M2 = 20.0
BOUNDARY_MARGIN_M = 2.0
MAX_EXISTING_OVERLAP_FRACTION = 0.10
MIN_EXISTING_CENTROID_DISTANCE_M = 3.0
MAX_ROAD_OVERLAP_FRACTION = 0.02
MAX_WATER_OVERLAP_FRACTION = 0.01

BACKGROUND = (250, 250, 248)
OSM_BUILDING = (55, 102, 157)
OVERTURE_BUILDING = (226, 132, 48)
MERGED_BUILDING = (205, 88, 44)
ROAD = (180, 180, 176)
WATER = (137, 196, 226)
VEGETATION = (191, 222, 174)
GRASS_PITCH = (91, 158, 83)
TARTAN_TRACK = (112, 32, 32)
BASKETBALL_COURT = (49, 86, 76)
TREE = (36, 125, 66)
OUTLINE = (45, 45, 45)


def _project_geometry(geometry: Any, center: Tuple[float, float]) -> Any:
    center_lon, center_lat = center
    x_scale = 111_320.0 * max(0.05, math.cos(math.radians(center_lat)))
    y_scale = 111_320.0
    return affine_transform(
        geometry,
        [x_scale, 0.0, 0.0, y_scale, -x_scale * center_lon, -y_scale * center_lat],
    )


def _tags(element: ET.Element) -> Dict[str, str]:
    return {
        tag.attrib["k"]: tag.attrib.get("v", "")
        for tag in element.findall("tag")
        if "k" in tag.attrib
    }


def _node_coordinates(root: ET.Element) -> Dict[str, Tuple[float, float]]:
    return {
        node.attrib["id"]: (float(node.attrib["lon"]), float(node.attrib["lat"]))
        for node in root.findall("node")
    }


def _way_coordinates(
    way: ET.Element,
    nodes: Dict[str, Tuple[float, float]],
) -> List[Tuple[float, float]]:
    return [
        nodes[nd.attrib["ref"]]
        for nd in way.findall("nd")
        if nd.attrib.get("ref") in nodes
    ]


def _polygon_from_coordinates(
    coordinates: Sequence[Tuple[float, float]],
) -> Any:
    if len(coordinates) < 4 or coordinates[0] != coordinates[-1]:
        return None
    polygon = Polygon(coordinates)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    return polygon if not polygon.is_empty else None


def _relation_polygon(
    relation: ET.Element,
    ways: Dict[str, ET.Element],
    nodes: Dict[str, Tuple[float, float]],
) -> Any:
    rings = {"outer": [], "inner": []}
    for member in relation.findall("member"):
        if member.attrib.get("type") != "way":
            continue
        way = ways.get(member.attrib.get("ref", ""))
        if way is None:
            continue
        coordinates = _way_coordinates(way, nodes)
        if len(coordinates) < 2:
            continue
        role = "inner" if member.attrib.get("role") == "inner" else "outer"
        rings[role].append(LineString(coordinates))
    if not rings["outer"]:
        return None
    outer = unary_union(list(polygonize(unary_union(rings["outer"]))))
    if outer.is_empty:
        return None
    if rings["inner"]:
        inner = unary_union(list(polygonize(unary_union(rings["inner"]))))
        if not inner.is_empty:
            outer = outer.difference(inner)
    return outer if not outer.is_empty else None


def _highway_half_width_m(tags: Dict[str, str]) -> float:
    try:
        explicit_width = float(tags.get("width", ""))
    except ValueError:
        explicit_width = 0.0
    if explicit_width > 0.0:
        return min(15.0, max(0.75, explicit_width / 2.0))
    highway = tags.get("highway", "").casefold()
    return {
        "motorway": 7.0,
        "trunk": 6.0,
        "primary": 5.0,
        "secondary": 4.5,
        "tertiary": 4.0,
        "residential": 3.5,
        "unclassified": 3.5,
        "service": 2.5,
        "living_street": 3.0,
        "cycleway": 1.25,
        "footway": 1.0,
        "path": 0.75,
    }.get(highway, 3.0)


def _track_half_width_m(tags: Dict[str, str]) -> float:
    try:
        width = float(tags.get("width", ""))
    except ValueError:
        width = 8.0
    return min(10.0, max(0.5, width / 2.0))


def _osm_context(
    tree: ET.ElementTree,
    bbox: Sequence[float],
    center: Tuple[float, float],
) -> Dict[str, Any]:
    root = tree.getroot()
    nodes = _node_coordinates(root)
    ways = {way.attrib["id"]: way for way in root.findall("way")}
    roads = []
    road_buffers = []
    water = []
    vegetation = []
    grass_pitches = []
    tartan_tracks = []
    basketball_courts = []
    trees = []

    for node in root.findall("node"):
        tags = _tags(node)
        if tags.get("natural", "").casefold() == "tree":
            point = _project_geometry(
                Point(float(node.attrib["lon"]), float(node.attrib["lat"])), center
            )
            trees.append((point.x, point.y))

    vegetation_natural = {
        "wood", "scrub", "heath", "grassland", "wetland", "mangrove"
    }
    vegetation_landuse = {
        "forest", "orchard", "vineyard", "meadow", "grass", "farmland"
    }
    water_natural = {"water", "bay"}
    water_landuse = {"reservoir", "basin"}
    for way in root.findall("way"):
        tags = _tags(way)
        coordinates = _way_coordinates(way, nodes)
        if len(coordinates) >= 2 and tags.get("highway"):
            line = _project_geometry(LineString(coordinates), center)
            roads.append(line)
            road_buffers.append(line.buffer(_highway_half_width_m(tags)))
        polygon = _polygon_from_coordinates(coordinates)
        if polygon is None:
            continue
        projected = _project_geometry(polygon, center)
        natural = tags.get("natural", "").casefold()
        landuse = tags.get("landuse", "").casefold()
        leisure = tags.get("leisure", "").casefold()
        surface = tags.get("surface", "").casefold()
        if natural in water_natural or landuse in water_landuse or tags.get("water"):
            water.append(projected)
        if natural in vegetation_natural or landuse in vegetation_landuse:
            vegetation.append(projected)
        if leisure == "pitch" and surface in {"grass", "turf", "grass_paver"}:
            grass_pitches.append(projected)
        if leisure == "track" and surface in {"tartan", "rubber", "synthetic"}:
            tartan_tracks.append(projected.boundary.buffer(_track_half_width_m(tags)))
        if leisure == "pitch" and tags.get("sport", "").casefold() == "basketball":
            basketball_courts.append(projected)

    for relation in root.findall("relation"):
        tags = _tags(relation)
        natural = tags.get("natural", "").casefold()
        landuse = tags.get("landuse", "").casefold()
        is_water = (
            natural in water_natural
            or landuse in water_landuse
            or bool(tags.get("water"))
        )
        is_vegetation = (
            natural in vegetation_natural
            or landuse in vegetation_landuse
        )
        leisure = tags.get("leisure", "").casefold()
        surface = tags.get("surface", "").casefold()
        is_grass_pitch = (
            leisure == "pitch"
            and surface in {"grass", "turf", "grass_paver"}
        )
        is_tartan_track = (
            leisure == "track"
            and surface in {"tartan", "rubber", "synthetic"}
        )
        is_basketball_court = (
            leisure == "pitch"
            and tags.get("sport", "").casefold() == "basketball"
        )
        if not is_water and not is_vegetation and not is_grass_pitch and not is_tartan_track and not is_basketball_court:
            continue
        polygon = _relation_polygon(relation, ways, nodes)
        if polygon is None:
            continue
        projected = _project_geometry(polygon, center)
        if is_water:
            water.append(projected)
        if is_vegetation:
            vegetation.append(projected)
        if is_grass_pitch:
            grass_pitches.append(projected)
        if is_tartan_track:
            tartan_tracks.append(projected.boundary.buffer(_track_half_width_m(tags)))
        if is_basketball_court:
            basketball_courts.append(projected)

    min_lon, min_lat, max_lon, max_lat = map(float, bbox)
    boundary = _project_geometry(
        box(min_lon, min_lat, max_lon, max_lat), center
    )
    safe_boundary = boundary.buffer(-BOUNDARY_MARGIN_M)
    return {
        "boundary": boundary,
        "safe_boundary": safe_boundary,
        "roads": roads,
        "road_exclusion": unary_union(road_buffers) if road_buffers else None,
        "water": water,
        "water_exclusion": unary_union(water) if water else None,
        "vegetation": vegetation,
        "grass_pitches": grass_pitches,
        "tartan_tracks": tartan_tracks,
        "basketball_courts": basketball_courts,
        "trees": trees,
    }


def _existing_metrics(
    geometry: Any,
    osm_buildings: Sequence[Dict[str, Any]],
) -> Dict[str, float]:
    best_cover = 0.0
    best_iou = 0.0
    best_area_ratio = 0.0
    nearest = float("inf")
    for building in osm_buildings:
        other = building.get("projected")
        if other is None or other.is_empty or other.area <= 0.0:
            continue
        distance = geometry.centroid.distance(other.centroid)
        nearest = min(nearest, distance)
        if not geometry.envelope.intersects(other.envelope):
            continue
        intersection = geometry.intersection(other).area
        if intersection <= 0.0:
            continue
        union = geometry.union(other).area
        cover = intersection / geometry.area
        iou = intersection / union if union > 0.0 else 0.0
        area_ratio = min(geometry.area, other.area) / max(geometry.area, other.area)
        if (cover, iou) > (best_cover, best_iou):
            best_cover = cover
            best_iou = iou
            best_area_ratio = area_ratio
    return {
        "best_overture_cover": best_cover,
        "best_iou": best_iou,
        "best_area_ratio": best_area_ratio,
        "nearest_osm_centroid_m": nearest,
    }


def classify_overture_buildings(
    osm_buildings: Sequence[Dict[str, Any]],
    overture: Sequence[Dict[str, Any]],
    context: Dict[str, Any],
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Return conservative additions and rejected/audited Overture records."""
    osm_ids = {str(item.get("id") or "") for item in osm_buildings}
    accepted = []
    rejected = []
    safe_boundary = context["safe_boundary"]
    road_exclusion = context.get("road_exclusion")
    water_exclusion = context.get("water_exclusion")

    for source in sorted(overture, key=lambda item: str(item.get("id") or "")):
        geometry = source.get("projected")
        record = dict(source)
        reason = None
        if geometry is None or geometry.is_empty or geometry.area <= 0.0:
            reason = "invalid_geometry"
            metrics = {}
        else:
            if not geometry.is_valid:
                geometry = geometry.buffer(0)
                record["projected"] = geometry
            metrics = _existing_metrics(geometry, osm_buildings)
            source_linked = bool(set(source.get("osm_ids") or ()) & osm_ids)
            existing_geometry = (
                metrics["best_overture_cover"] >= 0.50
                or metrics["best_iou"] >= 0.35
                or (
                    metrics["nearest_osm_centroid_m"] <= 2.0
                    and metrics["best_area_ratio"] >= 0.50
                )
            )
            road_overlap = (
                geometry.intersection(road_exclusion).area / geometry.area
                if road_exclusion is not None and not road_exclusion.is_empty
                else 0.0
            )
            water_overlap = (
                geometry.intersection(water_exclusion).area / geometry.area
                if water_exclusion is not None and not water_exclusion.is_empty
                else 0.0
            )
            metrics.update({
                "area_m2": geometry.area,
                "source_linked_to_osm": source_linked,
                "road_overlap_fraction": road_overlap,
                "water_overlap_fraction": water_overlap,
            })
            if source_linked:
                reason = "existing_osm_source"
            elif existing_geometry:
                reason = "existing_osm_geometry"
            elif geometry.area < MIN_BUILDING_AREA_M2:
                reason = "below_minimum_area"
            elif safe_boundary.is_empty or not safe_boundary.contains(geometry):
                reason = "map_boundary"
            elif metrics["best_overture_cover"] >= MAX_EXISTING_OVERLAP_FRACTION:
                reason = "ambiguous_osm_overlap"
            elif metrics["nearest_osm_centroid_m"] <= MIN_EXISTING_CENTROID_DISTANCE_M:
                reason = "too_close_to_osm_building"
            elif road_overlap > MAX_ROAD_OVERLAP_FRACTION:
                reason = "road_conflict"
            elif water_overlap > MAX_WATER_OVERLAP_FRACTION:
                reason = "water_conflict"
        record["audit"] = {
            key: (round(value, 6) if isinstance(value, float) and math.isfinite(value) else value)
            for key, value in metrics.items()
        }
        if reason is None:
            record["audit"]["decision"] = "accepted"
            accepted.append(record)
        else:
            record["audit"]["decision"] = "rejected"
            record["audit"]["reason"] = reason
            rejected.append(record)
    return accepted, rejected


def _next_negative_id(elements: Iterable[ET.Element]) -> int:
    identifiers = []
    for element in elements:
        try:
            identifiers.append(int(element.attrib["id"]))
        except (KeyError, ValueError):
            continue
    return min([0, *identifiers]) - 1


def _append_tags(element: ET.Element, tags: Dict[str, str]) -> None:
    for key, value in tags.items():
        ET.SubElement(element, "tag", {"k": str(key), "v": str(value)})


def _building_tags(source: Dict[str, Any]) -> Dict[str, str]:
    datasets = sorted(str(value) for value in source.get("source_datasets") or ())
    tags = {
        "building": "yes",
        "source": "Overture Maps",
        "mosaic:footprint_source": ",".join(datasets) or "Overture Maps",
        "mosaic:footprint_method": "overture_conservative",
        "mosaic:footprint_confidence": "medium",
        "mosaic:overture_id": str(source.get("id") or ""),
    }
    subtype = str(source.get("subtype") or "").strip()
    if subtype:
        tags["mosaic:overture_subtype"] = subtype
    height = source.get("height")
    if height is not None and math.isfinite(float(height)):
        tags["height"] = f"{float(height):.2f}"
        tags["mosaic:height_source"] = str(source.get("height_source") or "Overture")
    floors = source.get("num_floors")
    if floors is not None and float(floors) > 0:
        tags["building:levels"] = str(int(round(float(floors))))
    return tags


def _geometry_polygons(geometry: Any) -> List[Any]:
    if geometry.geom_type == "Polygon":
        return [geometry]
    if geometry.geom_type == "MultiPolygon":
        return [item for item in geometry.geoms if not item.is_empty]
    return []


def _append_polygon_records(
    tree: ET.ElementTree,
    accepted: Sequence[Dict[str, Any]],
    center: Tuple[float, float],
    tag_builder: Any,
) -> List[Dict[str, Any]]:
    """Append polygon records with deterministic negative OSM IDs."""
    root = tree.getroot()
    node_id = _next_negative_id(root.findall("node"))
    way_id = _next_negative_id(root.findall("way"))
    relation_id = _next_negative_id(root.findall("relation"))
    new_nodes = []
    new_ways = []
    new_relations = []
    added_buildings = []

    def append_ring(coordinates: Sequence[Tuple[float, float]]) -> int:
        nonlocal node_id, way_id
        way = ET.Element("way", {"id": str(way_id), "visible": "true"})
        this_way_id = way_id
        way_id -= 1
        refs = []
        for lon, lat in list(coordinates)[:-1]:
            node = ET.Element("node", {
                "id": str(node_id),
                "visible": "true",
                "lat": f"{float(lat):.9f}",
                "lon": f"{float(lon):.9f}",
            })
            new_nodes.append(node)
            refs.append(node_id)
            node_id -= 1
        for ref in [*refs, refs[0]]:
            ET.SubElement(way, "nd", {"ref": str(ref)})
        new_ways.append(way)
        return this_way_id

    for source in accepted:
        geometry = source["geometry"]
        polygons = _geometry_polygons(geometry)
        if not polygons:
            continue
        tags = tag_builder(source)
        has_relation = len(polygons) > 1 or any(poly.interiors for poly in polygons)
        if not has_relation:
            way_ref = append_ring(list(polygons[0].exterior.coords))
            element = new_ways[-1]
            _append_tags(element, tags)
            identifier = f"w{way_ref}"
        else:
            relation = ET.Element(
                "relation", {"id": str(relation_id), "visible": "true"}
            )
            identifier = f"r{relation_id}"
            relation_id -= 1
            for polygon in polygons:
                outer = append_ring(list(polygon.exterior.coords))
                ET.SubElement(relation, "member", {
                    "type": "way", "ref": str(outer), "role": "outer"
                })
                for interior in polygon.interiors:
                    inner = append_ring(list(interior.coords))
                    ET.SubElement(relation, "member", {
                        "type": "way", "ref": str(inner), "role": "inner"
                    })
            _append_tags(relation, {"type": "multipolygon", **tags})
            new_relations.append(relation)
            element = relation
        added_buildings.append({
            "id": identifier,
            "element": element,
            "tags": tags,
            "geometry": geometry,
            "projected": _project_geometry(geometry, center),
        })

    children = list(root)
    node_insert = 1 + max(
        (index for index, child in enumerate(children) if child.tag in {"bounds", "node"}),
        default=-1,
    )
    for offset, node in enumerate(new_nodes):
        root.insert(node_insert + offset, node)
    for way in new_ways:
        root.append(way)
    for relation in new_relations:
        root.append(relation)
    return added_buildings


def append_overture_buildings(
    tree: ET.ElementTree,
    accepted: Sequence[Dict[str, Any]],
    center: Tuple[float, float],
) -> List[Dict[str, Any]]:
    """Append accepted footprints with deterministic negative OSM IDs."""
    return _append_polygon_records(tree, accepted, center, _building_tags)


VEGETATION_TAGS = {
    "forest": {"natural": "wood"},
    "mangrove": {"natural": "wood", "leaf_type": "broadleaved"},
    "shrub": {"natural": "scrub"},
    "grass": {"landuse": "grass"},
    "crop": {"landuse": "farmland"},
    "wetland": {"natural": "wetland"},
    "moss": {"natural": "heath"},
}
MIN_VEGETATION_AREA_M2 = 100.0
MIN_LAND_COVER_MAX_ZOOM = 15
EXCLUSION_MARGIN_M = 1.0
HEIGHT_ENRICHED_BUILDING = (132, 76, 173)
OVERTURE_VEGETATION = (92, 173, 74)


def _unproject_geometry(geometry: Any, center: Tuple[float, float]) -> Any:
    center_lon, center_lat = center
    x_scale = 111_320.0 * max(0.05, math.cos(math.radians(center_lat)))
    y_scale = 111_320.0
    return affine_transform(
        geometry,
        [
            1.0 / x_scale,
            0.0,
            0.0,
            1.0 / y_scale,
            center_lon,
            center_lat,
        ],
    )


def _polygonal_geometry(geometry: Any) -> Any:
    polygons = _geometry_polygons(geometry)
    if not polygons and hasattr(geometry, "geoms"):
        for item in geometry.geoms:
            polygons.extend(_geometry_polygons(item))
    kept = [polygon for polygon in polygons if polygon.area >= MIN_VEGETATION_AREA_M2]
    return unary_union(kept) if kept else None


def load_overture_land_cover(
    path: Path,
    center: Tuple[float, float],
) -> List[Dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    records = []
    for feature in payload.get("features", []):
        properties = feature.get("properties") or {}
        geometry = shape(feature["geometry"])
        cartography = properties.get("cartography") or {}
        records.append({
            "id": str(feature.get("id") or ""),
            "subtype": str(properties.get("subtype") or "").casefold(),
            "max_zoom": int(cartography.get("max_zoom") or 0),
            "source_datasets": {
                str(source.get("dataset") or "")
                for source in properties.get("sources") or []
                if source.get("dataset")
            },
            "geometry": geometry,
            "projected": _project_geometry(geometry, center),
        })
    return records


def classify_overture_vegetation(
    land_cover: Sequence[Dict[str, Any]],
    osm_buildings: Sequence[Dict[str, Any]],
    context: Dict[str, Any],
    center: Tuple[float, float],
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    existing_vegetation = (
        unary_union(context["vegetation"])
        if context["vegetation"]
        else None
    )
    building_exclusion = unary_union([
        building["projected"].buffer(EXCLUSION_MARGIN_M)
        for building in osm_buildings
        if building.get("projected") is not None
        and not building["projected"].is_empty
    ])
    exclusions = [
        geometry
        for geometry in (
            existing_vegetation,
            building_exclusion,
            context.get("road_exclusion"),
            context.get("water_exclusion"),
        )
        if geometry is not None and not geometry.is_empty
    ]
    exclusion = unary_union(exclusions) if exclusions else None
    accepted = []
    rejected = []

    for source in sorted(land_cover, key=lambda item: str(item.get("id") or "")):
        record = dict(source)
        subtype = record["subtype"]
        geometry = record.get("projected")
        reason = None
        retained = None
        if subtype not in VEGETATION_TAGS:
            reason = "non_vegetation_subtype"
        elif record["max_zoom"] < MIN_LAND_COVER_MAX_ZOOM:
            reason = "low_resolution"
        elif geometry is None or geometry.is_empty:
            reason = "invalid_geometry"
        else:
            retained = geometry.intersection(context["safe_boundary"])
            if exclusion is not None and not retained.is_empty:
                retained = retained.difference(exclusion)
            if not retained.is_empty:
                retained = retained.simplify(0.25, preserve_topology=True)
                retained = _polygonal_geometry(retained)
            if retained is None or retained.is_empty:
                reason = "fully_covered_or_excluded"
        original_area = float(geometry.area) if geometry is not None else 0.0
        retained_area = float(retained.area) if retained is not None else 0.0
        record["audit"] = {
            "decision": "accepted" if reason is None else "rejected",
            "reason": reason,
            "original_area_m2": round(original_area, 3),
            "retained_area_m2": round(retained_area, 3),
            "retained_fraction": (
                round(retained_area / original_area, 6)
                if original_area > 0.0 else 0.0
            ),
        }
        if reason is None:
            record["projected"] = retained
            record["geometry"] = _unproject_geometry(retained, center)
            accepted.append(record)
        else:
            rejected.append(record)
    return accepted, rejected


def _vegetation_tags(source: Dict[str, Any]) -> Dict[str, str]:
    subtype = str(source["subtype"])
    datasets = sorted(source.get("source_datasets") or {"ESA WorldCover"})
    return {
        **VEGETATION_TAGS[subtype],
        "source": "Overture Maps / ESA WorldCover",
        "mosaic:vegetation_source": ",".join(datasets),
        "mosaic:vegetation_method": "overture_land_cover_conservative",
        "mosaic:vegetation_confidence": "medium",
        "mosaic:overture_id": str(source.get("id") or ""),
        "mosaic:overture_subtype": subtype,
    }


def append_overture_vegetation(
    tree: ET.ElementTree,
    accepted: Sequence[Dict[str, Any]],
    center: Tuple[float, float],
) -> List[Dict[str, Any]]:
    return _append_polygon_records(tree, accepted, center, _vegetation_tags)


class _TopView:
    def __init__(self, boundary: Any, max_size: int = 1400) -> None:
        min_x, min_y, max_x, max_y = boundary.bounds
        span_x = max(1.0, max_x - min_x)
        span_y = max(1.0, max_y - min_y)
        self.margin = 24
        if span_x >= span_y:
            self.width = max_size
            self.height = max(256, int(round(max_size * span_y / span_x)))
        else:
            self.height = max_size
            self.width = max(256, int(round(max_size * span_x / span_y)))
        self.min_x = min_x
        self.max_y = max_y
        self.scale = min(
            (self.width - 2 * self.margin) / span_x,
            (self.height - 2 * self.margin) / span_y,
        )
        self.image = np.empty((self.height, self.width, 3), dtype=np.uint8)
        self.image[:, :] = BACKGROUND

    def point(self, x: float, y: float) -> Tuple[int, int]:
        px = int(round(self.margin + (x - self.min_x) * self.scale))
        py = int(round(self.margin + (self.max_y - y) * self.scale))
        return px, py

    def line(self, start: Tuple[int, int], end: Tuple[int, int], color: Tuple[int, int, int], width: int = 1) -> None:
        x0, y0 = start
        x1, y1 = end
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        xs = np.rint(np.linspace(x0, x1, steps + 1)).astype(int)
        ys = np.rint(np.linspace(y0, y1, steps + 1)).astype(int)
        radius = max(0, width // 2)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                xx = xs + dx
                yy = ys + dy
                mask = (xx >= 0) & (xx < self.width) & (yy >= 0) & (yy < self.height)
                self.image[yy[mask], xx[mask]] = color

    def polygon(self, polygon: Any, fill: Tuple[int, int, int], outline: Tuple[int, int, int] = OUTLINE) -> None:
        # Apply the even-odd rule across the exterior and every interior ring.
        # Combining all ring intersections preserves holes in ring-shaped
        # surfaces, such as running tracks, and keeps the underlying grass
        # pitch visible through the interior.
        rings = [
            [self.point(x, y) for x, y in ring.coords]
            for ring in (polygon.exterior, *polygon.interiors)
        ]
        rings = [points for points in rings if len(points) >= 4]
        if not rings:
            return
        min_y = max(0, min(y for points in rings for _, y in points))
        max_y = min(self.height - 1, max(y for points in rings for _, y in points))
        for y in range(min_y, max_y + 1):
            intersections = []
            scan_y = y + 0.5
            for points in rings:
                for (x0, y0), (x1, y1) in zip(points, points[1:]):
                    if y0 == y1 or not (min(y0, y1) <= scan_y < max(y0, y1)):
                        continue
                    intersections.append(x0 + (scan_y - y0) * (x1 - x0) / (y1 - y0))
            intersections.sort()
            for left, right in zip(intersections[0::2], intersections[1::2]):
                x0 = max(0, int(math.ceil(left)))
                x1 = min(self.width - 1, int(math.floor(right)))
                if x1 >= x0:
                    self.image[y, x0:x1 + 1] = fill
        for points in rings:
            for start, end in zip(points, points[1:]):
                self.line(start, end, outline)

    def geometry(self, geometry: Any, fill: Tuple[int, int, int], outline: Tuple[int, int, int] = OUTLINE) -> None:
        for polygon in _geometry_polygons(geometry):
            self.polygon(polygon, fill, outline)

    def polyline(self, geometry: Any, color: Tuple[int, int, int], width: int = 1) -> None:
        lines = [geometry] if geometry.geom_type == "LineString" else list(getattr(geometry, "geoms", []))
        for line in lines:
            points = [self.point(x, y) for x, y in line.coords]
            for start, end in zip(points, points[1:]):
                self.line(start, end, color, width)

    def circle(self, point: Tuple[float, float], color: Tuple[int, int, int], radius: int = 3) -> None:
        cx, cy = self.point(*point)
        y0 = max(0, cy - radius)
        y1 = min(self.height - 1, cy + radius)
        x0 = max(0, cx - radius)
        x1 = min(self.width - 1, cx + radius)
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                if (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2:
                    self.image[y, x] = color

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        scanlines = b"".join(b"\x00" + row.tobytes() for row in self.image)
        payload = zlib.compress(scanlines, level=7)

        def chunk(kind: bytes, data: bytes) -> bytes:
            body = kind + data
            return struct.pack(">I", len(data)) + body + struct.pack(">I", binascii.crc32(body) & 0xFFFFFFFF)

        png = b"\x89PNG\r\n\x1a\n"
        png += chunk(b"IHDR", struct.pack(">IIBBBBB", self.width, self.height, 8, 2, 0, 0, 0))
        png += chunk(b"IDAT", payload)
        png += chunk(b"IEND", b"")
        path.write_bytes(png)


def _draw_context(view: _TopView, context: Dict[str, Any]) -> None:
    for geometry in context["water"]:
        view.geometry(geometry, WATER, WATER)
    for geometry in context["vegetation"]:
        view.geometry(geometry, VEGETATION, VEGETATION)
    for geometry in context.get("grass_pitches", []):
        view.geometry(geometry, GRASS_PITCH, GRASS_PITCH)
    for geometry in context.get("basketball_courts", []):
        view.geometry(geometry, BASKETBALL_COURT, BASKETBALL_COURT)
    for geometry in context.get("tartan_tracks", []):
        view.geometry(geometry, TARTAN_TRACK, TARTAN_TRACK)
    for road in context["roads"]:
        view.polyline(road, ROAD, width=2)


def render_osm_topview(
    output_path: Path,
    osm_buildings: Sequence[Dict[str, Any]],
    context: Dict[str, Any],
) -> str:
    """Render one OSM state, including holes and all classified surfaces."""
    view = _TopView(context["boundary"])
    _draw_context(view, context)
    for building in osm_buildings:
        view.geometry(building["projected"], OSM_BUILDING)
    for tree in context["trees"]:
        view.circle(tree, TREE)
    view.save(output_path)
    return str(output_path)


def render_topviews(
    output_dir: Path,
    osm_buildings: Sequence[Dict[str, Any]],
    overture: Sequence[Dict[str, Any]],
    accepted: Sequence[Dict[str, Any]],
    context: Dict[str, Any],
    *,
    overture_vegetation: Sequence[Dict[str, Any]] = (),
    accepted_vegetation: Sequence[Dict[str, Any]] = (),
    height_enriched_osm_ids: Iterable[str] = (),
) -> Dict[str, str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    osm_path = output_dir / "osm_source_topview.png"
    overture_path = output_dir / "overture_source_topview.png"
    merged_path = output_dir / "merged_topview.png"

    render_osm_topview(osm_path, osm_buildings, context)

    view = _TopView(context["boundary"])
    for vegetation in overture_vegetation:
        geometry = vegetation.get("projected")
        if (
            vegetation.get("subtype") in VEGETATION_TAGS
            and int(vegetation.get("max_zoom") or 0) >= MIN_LAND_COVER_MAX_ZOOM
            and geometry is not None
            and not geometry.is_empty
        ):
            view.geometry(geometry, OVERTURE_VEGETATION, OVERTURE_VEGETATION)
    for building in overture:
        geometry = building.get("projected")
        if geometry is not None and not geometry.is_empty:
            view.geometry(geometry, OVERTURE_BUILDING)
    view.save(overture_path)

    view = _TopView(context["boundary"])
    _draw_context(view, context)
    for vegetation in accepted_vegetation:
        view.geometry(
            vegetation["projected"], OVERTURE_VEGETATION, OVERTURE_VEGETATION
        )
    enriched_ids = {str(identifier) for identifier in height_enriched_osm_ids}
    for building in osm_buildings:
        color = (
            HEIGHT_ENRICHED_BUILDING
            if str(building.get("id") or "") in enriched_ids
            else OSM_BUILDING
        )
        view.geometry(building["projected"], color)
    for building in accepted:
        view.geometry(building["projected"], MERGED_BUILDING)
    for tree in context["trees"]:
        view.circle(tree, TREE)
    view.save(merged_path)
    return {
        "osm_source_topview": str(osm_path),
        "overture_source_topview": str(overture_path),
        "merged_topview": str(merged_path),
    }


def _geojson_feature(record: Dict[str, Any]) -> Dict[str, Any]:
    audit = record.get("audit") or {}
    return {
        "type": "Feature",
        "id": str(record.get("id") or ""),
        "geometry": mapping(record["geometry"]),
        "properties": {
            "decision": audit.get("decision"),
            "reason": audit.get("reason"),
            "area_m2": audit.get("area_m2"),
            "height_m": record.get("height"),
            "source_datasets": sorted(record.get("source_datasets") or ()),
            "nearest_osm_centroid_m": audit.get("nearest_osm_centroid_m"),
            "best_overture_cover": audit.get("best_overture_cover"),
            "road_overlap_fraction": audit.get("road_overlap_fraction"),
            "water_overlap_fraction": audit.get("water_overlap_fraction"),
        },
    }


def _write_geojson(path: Path, records: Sequence[Dict[str, Any]]) -> None:
    path.write_text(json.dumps({
        "type": "FeatureCollection",
        "features": [_geojson_feature(record) for record in records],
    }, indent=2), encoding="utf-8")




def _write_vegetation_geojson(
    path: Path,
    records: Sequence[Dict[str, Any]],
) -> None:
    features = []
    for record in records:
        audit = record.get("audit") or {}
        features.append({
            "type": "Feature",
            "id": str(record.get("id") or ""),
            "geometry": mapping(record["geometry"]),
            "properties": {
                "subtype": record.get("subtype"),
                "max_zoom": record.get("max_zoom"),
                "source_datasets": sorted(record.get("source_datasets") or ()),
                **audit,
            },
        })
    path.write_text(json.dumps({
        "type": "FeatureCollection",
        "features": features,
    }, indent=2), encoding="utf-8")


def enrich_overture_vegetation(
    tree: ET.ElementTree,
    land_cover: Sequence[Dict[str, Any]],
    osm_buildings: Sequence[Dict[str, Any]],
    context: Dict[str, Any],
    center: Tuple[float, float],
    output_dir: Path,
    mode: str,
) -> Dict[str, Any]:
    if mode not in {"none", "overture-conservative"}:
        raise ValueError(f"Unsupported vegetation enrichment mode: {mode}")
    accepted, rejected = classify_overture_vegetation(
        land_cover, osm_buildings, context, center
    )
    additions = accepted if mode == "overture-conservative" else []
    added = append_overture_vegetation(tree, additions, center)
    output_dir.mkdir(parents=True, exist_ok=True)
    accepted_path = output_dir / "accepted_overture_vegetation.geojson"
    rejected_path = output_dir / "rejected_overture_land_cover.geojson"
    _write_vegetation_geojson(accepted_path, accepted)
    _write_vegetation_geojson(rejected_path, rejected)
    rejected_counts = Counter(
        str((record.get("audit") or {}).get("reason") or "unknown")
        for record in rejected
    )
    added_area_by_subtype: Counter[str] = Counter()
    for record in additions:
        added_area_by_subtype[str(record["subtype"])] += float(
            record["projected"].area
        )
    report = {
        "mode": mode,
        "overture_land_cover_count": len(land_cover),
        "high_resolution_vegetation_count": sum(
            record.get("subtype") in VEGETATION_TAGS
            and int(record.get("max_zoom") or 0) >= MIN_LAND_COVER_MAX_ZOOM
            for record in land_cover
        ),
        "accepted_candidate_count": len(accepted),
        "added_area_count": len(added),
        "added_area_m2_by_subtype": {
            key: round(value, 3)
            for key, value in sorted(added_area_by_subtype.items())
        },
        "rejected_count": len(rejected),
        "rejected_reason_counts": dict(rejected_counts),
        "osm_tree_point_count": len(context["trees"]),
        "osm_vegetation_area_count": len(context["vegetation"]),
        "resolution_policy": {
            "minimum_cartography_max_zoom": MIN_LAND_COVER_MAX_ZOOM,
            "minimum_retained_area_m2": MIN_VEGETATION_AREA_M2,
            "note": (
                "Overture land_cover supplies vegetation regions, not "
                "individual tree coordinates. OSM natural=tree remains the "
                "individual-tree source."
            ),
        },
        "accepted_geojson": str(accepted_path),
        "rejected_geojson": str(rejected_path),
    }
    report_path = output_dir / "vegetation_merge_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    report["report"] = str(report_path)
    return {"accepted": additions, "added": added, "report": report}


def enrich_overture_footprints(
    tree: ET.ElementTree,
    osm_buildings: List[Dict[str, Any]],
    overture: Sequence[Dict[str, Any]],
    bbox: Sequence[float],
    center: Tuple[float, float],
    output_dir: Path,
    mode: str,
) -> Dict[str, Any]:
    if mode not in {"none", "overture-conservative"}:
        raise ValueError(f"Unsupported footprint enrichment mode: {mode}")
    context = _osm_context(tree, bbox, center)
    accepted, rejected = classify_overture_buildings(
        osm_buildings, overture, context
    )
    additions = accepted if mode == "overture-conservative" else []
    added_buildings = append_overture_buildings(tree, additions, center)
    output_dir.mkdir(parents=True, exist_ok=True)
    accepted_path = output_dir / "accepted_overture_buildings.geojson"
    rejected_path = output_dir / "rejected_overture_buildings.geojson"
    _write_geojson(accepted_path, accepted)
    _write_geojson(rejected_path, rejected)
    images = render_topviews(
        output_dir, osm_buildings, overture, additions, context
    )
    rejected_counts = Counter(
        str((record.get("audit") or {}).get("reason") or "unknown")
        for record in rejected
    )
    source_counts = Counter()
    for record in accepted:
        for source in record.get("source_datasets") or {"Overture Maps"}:
            source_counts[str(source)] += 1
    report = {
        "mode": mode,
        "osm_building_count": len(osm_buildings),
        "overture_building_count": len(overture),
        "accepted_overture_candidate_count": len(accepted),
        "added_overture_count": len(added_buildings),
        "merged_building_count": len(osm_buildings) + len(added_buildings),
        "rejected_overture_count": len(rejected),
        "rejected_reason_counts": dict(rejected_counts),
        "accepted_source_counts": dict(source_counts),
        "osm_tree_point_count": len(context["trees"]),
        "osm_vegetation_area_count": len(context["vegetation"]),
        "overture_land_cover_layer_available": False,
        "accepted_geojson": str(accepted_path),
        "rejected_geojson": str(rejected_path),
        "images": images,
        "color_legend": {
            "osm_buildings": list(OSM_BUILDING),
            "overture_buildings": list(OVERTURE_BUILDING),
            "accepted_overture_buildings_in_merge": list(MERGED_BUILDING),
            "externally_height_enriched_osm_buildings": list(
                HEIGHT_ENRICHED_BUILDING
            ),
            "accepted_overture_vegetation": list(OVERTURE_VEGETATION),
            "osm_trees": list(TREE),
            "osm_vegetation": list(VEGETATION),
            "osm_roads": list(ROAD),
            "osm_water": list(WATER),
        },
        "thresholds": {
            "minimum_building_area_m2": MIN_BUILDING_AREA_M2,
            "boundary_margin_m": BOUNDARY_MARGIN_M,
            "maximum_existing_overlap_fraction": MAX_EXISTING_OVERLAP_FRACTION,
            "minimum_existing_centroid_distance_m": MIN_EXISTING_CENTROID_DISTANCE_M,
            "maximum_road_overlap_fraction": MAX_ROAD_OVERLAP_FRACTION,
            "maximum_water_overlap_fraction": MAX_WATER_OVERLAP_FRACTION,
        },
    }
    report_path = output_dir / "footprint_merge_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    report["report"] = str(report_path)
    return {
        "accepted": additions,
        "context": context,
        "added_buildings": added_buildings,
        "report": report,
    }
