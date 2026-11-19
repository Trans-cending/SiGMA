#!/usr/bin/env python3
"""Merge missing Overture footprints and enrich traceable building heights.

Standalone CLI: python -m sigma.map_construction.tools.enrich_osm_building_heights --help
The unified Stage I entry is documented in docs/stage1.md.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import zipfile
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from shapely.affinity import affine_transform
from shapely.geometry import LineString, Polygon, shape
from shapely.ops import polygonize, unary_union

from sigma.map_construction.http_range import HTTPRangeReader, extract_remote_zip_member
from sigma.map_construction.tools.building_footprint_enrichment import (
    enrich_overture_footprints,
    enrich_overture_vegetation,
    load_overture_land_cover,
    render_topviews,
)


GHS_RELEASE = "R2024A"
GHS_BASE_URL = (
    "https://jeodpp.jrc.ec.europa.eu/ftp/jrc-opendata/GHSL/"
    "GHS_OBAT_GLOBE_R2024A/GHS_OBAT_CSV_GLOBE_R2024A/V1-0"
)
VALID_HEIGHT_MIN_M = 2.0
VALID_HEIGHT_MAX_M = 350.0
NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)")


def parse_number(value: Any) -> Optional[float]:
    if value is None:
        return None
    match = NUMBER_RE.search(str(value))
    if match is None:
        return None
    number = float(match.group(0))
    return number if math.isfinite(number) else None


def valid_height(value: Any) -> Optional[float]:
    number = parse_number(value)
    if number is None or not VALID_HEIGHT_MIN_M <= number <= VALID_HEIGHT_MAX_M:
        return None
    return number


def element_tags(element: ET.Element) -> Dict[str, str]:
    return {
        tag.attrib["k"]: tag.attrib.get("v", "")
        for tag in element.findall("tag")
        if "k" in tag.attrib
    }


def osm_identifier(element: ET.Element) -> str:
    prefix = {"way": "w", "relation": "r"}.get(element.tag)
    return f"{prefix}{element.attrib['id']}" if prefix else ""


def project_geometry(geometry: Any, center_lon: float, center_lat: float) -> Any:
    x_scale = 111_320.0 * max(0.05, math.cos(math.radians(center_lat)))
    y_scale = 111_320.0
    return affine_transform(
        geometry,
        [x_scale, 0.0, 0.0, y_scale, -x_scale * center_lon, -y_scale * center_lat],
    )


def way_geometry(
    way: ET.Element,
    nodes: Dict[str, Tuple[float, float]],
) -> Optional[Any]:
    coordinates = [
        nodes[ref.attrib["ref"]]
        for ref in way.findall("nd")
        if ref.attrib.get("ref") in nodes
    ]
    if len(coordinates) < 4 or coordinates[0] != coordinates[-1]:
        return None
    polygon = Polygon(coordinates)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    return polygon if not polygon.is_empty else None


def relation_geometry(
    relation: ET.Element,
    ways: Dict[str, ET.Element],
    nodes: Dict[str, Tuple[float, float]],
) -> Optional[Any]:
    outer_lines = []
    inner_lines = []
    for member in relation.findall("member"):
        if member.attrib.get("type") != "way":
            continue
        way = ways.get(member.attrib.get("ref", ""))
        if way is None:
            continue
        coordinates = [
            nodes[ref.attrib["ref"]]
            for ref in way.findall("nd")
            if ref.attrib.get("ref") in nodes
        ]
        if len(coordinates) < 2:
            continue
        target = inner_lines if member.attrib.get("role") == "inner" else outer_lines
        target.append(LineString(coordinates))

    if not outer_lines:
        return None
    outer = unary_union(list(polygonize(unary_union(outer_lines))))
    if outer.is_empty:
        closed = [way_geometry(ways[m.attrib["ref"]], nodes) for m in relation.findall("member")
                  if m.attrib.get("type") == "way"
                  and m.attrib.get("role", "outer") != "inner"
                  and m.attrib.get("ref") in ways]
        outer = unary_union([item for item in closed if item is not None])
    if outer.is_empty:
        return None
    if inner_lines:
        inner = unary_union(list(polygonize(unary_union(inner_lines))))
        if not inner.is_empty:
            outer = outer.difference(inner)
    return outer if not outer.is_empty else None


def load_osm(path: Path) -> Tuple[ET.ElementTree, List[Dict[str, Any]], Tuple[float, ...]]:
    tree = ET.parse(path)
    root = tree.getroot()
    bounds = root.find("bounds")
    if bounds is None:
        raise ValueError(f"OSM has no bounds element: {path}")
    bbox = (
        float(bounds.attrib["minlon"]),
        float(bounds.attrib["minlat"]),
        float(bounds.attrib["maxlon"]),
        float(bounds.attrib["maxlat"]),
    )
    center_lon = (bbox[0] + bbox[2]) / 2.0
    center_lat = (bbox[1] + bbox[3]) / 2.0
    nodes = {
        node.attrib["id"]: (float(node.attrib["lon"]), float(node.attrib["lat"]))
        for node in root.findall("node")
    }
    ways = {way.attrib["id"]: way for way in root.findall("way")}
    buildings: List[Dict[str, Any]] = []
    for element in list(root):
        tags = element_tags(element)
        if (
            "building" not in tags
            or tags["building"].strip().casefold() in {"no", "false", "0"}
            or element.tag not in {"way", "relation"}
        ):
            continue
        geometry = None
        if element.tag == "way":
            geometry = way_geometry(element, nodes)
        elif element.tag == "relation":
            geometry = relation_geometry(element, ways, nodes)
        if geometry is None:
            continue
        projected = project_geometry(geometry, center_lon, center_lat)
        buildings.append(
            {
                "id": osm_identifier(element),
                "element": element,
                "tags": tags,
                "geometry": geometry,
                "projected": projected,
            }
        )
    return tree, buildings, bbox


def _overture_cache_path(
    bbox: Sequence[float],
    release: str,
    cache_root: Path,
    feature_type: str = "building",
) -> Path:
    bbox_text = ",".join(f"{value:.7f}" for value in bbox)
    if feature_type == "building":
        cache_key = hashlib.sha256(
            f"{release}|{bbox_text}".encode()
        ).hexdigest()[:20]
        return cache_root / "overture" / release / f"{cache_key}.geojson"
    cache_key = hashlib.sha256(
        f"{release}|{feature_type}|{bbox_text}".encode()
    ).hexdigest()[:20]
    return (
        cache_root / "overture" / release / feature_type
        / f"{cache_key}.geojson"
    )


def resolve_overture_release(
    cli: Path,
    requested: str,
    *,
    bbox: Optional[Sequence[float]] = None,
    cache_root: Optional[Path] = None,
    feature_types: Sequence[str] = ("building",),
) -> str:
    if requested and requested != "latest":
        return requested
    try:
        result = subprocess.run(
            [str(cli), "releases", "latest"],
            check=True,
            text=True,
            capture_output=True,
        )
    except subprocess.CalledProcessError as exc:
        if bbox is not None and cache_root is not None:
            release_root = cache_root / "overture"
            candidates = sorted(
                (
                    path.name
                    for path in release_root.iterdir()
                    if path.is_dir() and path.name != "latest"
                ),
                reverse=True,
            ) if release_root.is_dir() else []
            for release in candidates:
                cached = [
                    _overture_cache_path(
                        bbox, release, cache_root, feature_type
                    )
                    for feature_type in feature_types
                ]
                if cached and all(
                    path.is_file() and path.stat().st_size > 100
                    for path in cached
                ):
                    print(
                        "OVERTURE_RELEASE_SOURCE=matching_bbox_cache "
                        f"release={release}",
                        file=sys.stderr,
                    )
                    return release
        detail = (exc.stderr or "").strip()
        raise RuntimeError(
            "Overture latest-release lookup failed and no exact-bbox "
            f"concrete release cache is available: {detail}"
        ) from exc
    release = result.stdout.strip().splitlines()[-1]
    if not release:
        raise RuntimeError("Overture client returned an empty latest release")
    return release


def download_overture(
    cli: Path,
    bbox: Sequence[float],
    release: str,
    cache_root: Path,
    feature_type: str = "building",
) -> Path:
    output = _overture_cache_path(
        bbox, release, cache_root, feature_type
    )
    if output.is_file() and output.stat().st_size > 100:
        return output
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".tmp.geojson")
    bbox_text = ",".join(f"{value:.7f}" for value in bbox)
    command = [
        str(cli),
        "download",
        f"--bbox={bbox_text}",
        "-f",
        "geojson",
        "-t",
        feature_type,
        "-r",
        release,
        "-o",
        str(temporary),
    ]
    subprocess.run(command, check=True)
    temporary.replace(output)
    state = temporary.with_suffix(temporary.suffix + ".state")
    if state.exists():
        state.unlink()
    return output

def height_source(properties: Dict[str, Any]) -> Tuple[str, Optional[float]]:
    for source in properties.get("sources") or []:
        if source.get("property") == "/properties/height":
            return str(source.get("dataset") or "Overture"), parse_number(source.get("confidence"))
    return "Overture", None


def load_overture(path: Path, center: Tuple[float, float]) -> List[Dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    features = payload.get("features", [])
    result = []
    for feature in features:
        properties = feature.get("properties") or {}
        height = valid_height(properties.get("height"))
        floors = parse_number(properties.get("num_floors"))
        source_name, source_confidence = height_source(properties)
        if height is None and floors is not None and floors > 0:
            height = min(VALID_HEIGHT_MAX_M, 3.0 * floors)
            source_name = "Overture num_floors"
            source_confidence = None
        geometry = shape(feature["geometry"])
        osm_ids = {
            str(source.get("record_id")).split("@", 1)[0]
            for source in properties.get("sources") or []
            if source.get("dataset") == "OpenStreetMap" and source.get("record_id")
        }
        source_datasets = {
            str(source.get("dataset") or "")
            for source in properties.get("sources") or []
            if source.get("dataset")
            and source.get("property") in {None, "", "/geometry"}
        }
        if not source_datasets:
            source_datasets = {
                str(source.get("dataset") or "")
                for source in properties.get("sources") or []
                if source.get("dataset")
            }
        result.append(
            {
                "id": str(feature.get("id") or ""),
                "height": height,
                "height_source": source_name,
                "source_confidence": source_confidence,
                "num_floors": floors,
                "subtype": properties.get("subtype"),
                "source_datasets": source_datasets,
                "osm_ids": osm_ids,
                "geometry": geometry,
                "projected": project_geometry(geometry, center[0], center[1]),
            }
        )
    return result


def normalized_region_name(value: str) -> str:
    return re.sub(r"[^a-z0-9]", "", value.casefold())


def ghs_archive_url(country: str) -> str:
    country = country.upper()
    return (
        f"{GHS_BASE_URL}/"
        f"GHS_OBAT_CSV_{country}_E2020_R2024A_V1_0.zip"
    )


def find_ghs_member(url: str, country: str, admin1: str) -> str:
    expected = normalized_region_name(admin1)
    with HTTPRangeReader(url) as remote:
        with zipfile.ZipFile(remote) as archive:
            candidates = [
                info.filename
                for info in archive.infolist()
                if info.filename.lower().endswith(".csv")
                and normalized_region_name(
                    Path(info.filename).stem.rsplit("_", 1)[-1]
                ) == expected
            ]
    if len(candidates) != 1:
        raise RuntimeError(
            f"Expected one GHS-OBAT member for {country}/{admin1}, "
            f"found {candidates}"
        )
    return candidates[0]


def load_ghs_rows(
    csv_path: Path,
    bbox: Sequence[float],
    cache_path: Optional[Path] = None,
) -> List[Dict[str, Any]]:
    if cache_path is not None and cache_path.is_file():
        return json.loads(cache_path.read_text(encoding="utf-8"))
    min_lon, min_lat, max_lon, max_lat = bbox
    rows = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            lon = float(row["lon"])
            lat = float(row["lat"])
            height = valid_height(row.get("height"))
            if (
                height is None
                or lon < min_lon
                or lon > max_lon
                or lat < min_lat
                or lat > max_lat
            ):
                continue
            rows.append(
                {
                    "id": row["id"],
                    "lon": lon,
                    "lat": lat,
                    "height": height,
                    "area": float(row["area"]),
                }
            )
    if cache_path is not None:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        cache_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    return rows


def _cached_ghs_csv(
    cache_root: Path,
    country: str,
    admin1: str,
) -> Optional[Path]:
    directory = cache_root / "ghs_obat" / GHS_RELEASE / country
    expected = normalized_region_name(admin1)
    candidates = sorted(
        path
        for path in directory.glob("*.csv")
        if normalized_region_name(path.stem).endswith(expected)
    )
    if len(candidates) > 1:
        raise RuntimeError(
            f"Multiple cached GHS-OBAT files match {country}/{admin1}: "
            f"{candidates}"
        )
    return candidates[0] if candidates else None


def prepare_ghs(
    country: str,
    admin1: str,
    bbox: Sequence[float],
    cache_root: Path,
    explicit_csv: Optional[Path],
) -> Tuple[Path, List[Dict[str, Any]], str]:
    country = country.upper()
    url = ghs_archive_url(country)
    cached_csv = _cached_ghs_csv(cache_root, country, admin1)
    if explicit_csv is not None:
        csv_path = explicit_csv
        member = csv_path.name
    elif cached_csv is not None:
        csv_path = cached_csv
        member = csv_path.name
        print(f"GHS_SOURCE=region_cache csv={csv_path}", file=sys.stderr)
    else:
        member = find_ghs_member(url, country, admin1)
        csv_path = cache_root / "ghs_obat" / GHS_RELEASE / country / Path(member).name
        if not csv_path.is_file():
            extract_remote_zip_member(url, member, csv_path)
    bbox_key = hashlib.sha256(
        "|".join(f"{value:.7f}" for value in bbox).encode()
    ).hexdigest()[:20]
    subset = csv_path.parent / "subsets" / f"{bbox_key}.json"
    return csv_path, load_ghs_rows(csv_path, bbox, subset), member

def match_overture(
    buildings: List[Dict[str, Any]],
    overture: List[Dict[str, Any]],
) -> Dict[str, Dict[str, Any]]:
    matches: Dict[str, Dict[str, Any]] = {}
    used = set()
    by_osm: Dict[str, List[int]] = {}
    for index, item in enumerate(overture):
        for osm_id in item["osm_ids"]:
            by_osm.setdefault(osm_id, []).append(index)

    for building in buildings:
        candidates = [
            index
            for index in by_osm.get(building["id"], [])
            if overture[index]["height"] is not None
        ]
        if len(candidates) == 1:
            index = candidates[0]
            used.add(index)
            matches[building["id"]] = {
                "candidate": overture[index],
                "method": "exact_osm_source_id",
                "distance_m": 0.0,
                "iou": 1.0,
            }

    pairs = []
    for building in buildings:
        if building["id"] in matches or building["projected"] is None:
            continue
        geometry = building["projected"]
        for index, candidate in enumerate(overture):
            if index in used or candidate["height"] is None:
                continue
            other = candidate["projected"]
            if not geometry.envelope.intersects(other.envelope):
                continue
            intersection = geometry.intersection(other).area
            union = geometry.union(other).area
            iou = intersection / union if union > 0 else 0.0
            distance = geometry.centroid.distance(other.centroid)
            area_ratio = min(geometry.area, other.area) / max(geometry.area, other.area)
            if iou < 0.65 and not (distance <= 2.0 and area_ratio >= 0.75):
                continue
            pairs.append((iou + 0.1 * area_ratio - 0.002 * distance, building, index, iou, distance))

    for _, building, index, iou, distance in sorted(pairs, reverse=True, key=lambda row: row[0]):
        if building["id"] in matches or index in used:
            continue
        used.add(index)
        matches[building["id"]] = {
            "candidate": overture[index],
            "method": "strict_geometry",
            "distance_m": distance,
            "iou": iou,
        }
    return matches


def match_ghs(
    buildings: List[Dict[str, Any]],
    rows: List[Dict[str, Any]],
    excluded: Iterable[str],
    center: Tuple[float, float],
) -> Dict[str, Dict[str, Any]]:
    excluded = set(excluded)
    candidates = []
    for index, row in enumerate(rows):
        point_x = (row["lon"] - center[0]) * 111_320.0 * math.cos(math.radians(center[1]))
        point_y = (row["lat"] - center[1]) * 111_320.0
        for building in buildings:
            geometry = building["projected"]
            if building["id"] in excluded or geometry is None or geometry.area <= 0:
                continue
            distance = geometry.centroid.distance(
                type(geometry.centroid)(point_x, point_y)
            )
            area_ratio = min(geometry.area, row["area"]) / max(geometry.area, row["area"])
            if distance <= 8.0 and area_ratio >= 0.65:
                score = distance + 4.0 * abs(math.log(area_ratio))
                candidates.append((score, building, index, distance, area_ratio))

    matches: Dict[str, Dict[str, Any]] = {}
    used = set()
    for _, building, index, distance, area_ratio in sorted(candidates, key=lambda row: row[0]):
        if building["id"] in matches or index in used:
            continue
        used.add(index)
        matches[building["id"]] = {
            "candidate": rows[index],
            "method": "strict_centroid_area",
            "distance_m": distance,
            "area_ratio": area_ratio,
        }
    return matches


def has_measured_overture_height(match: Optional[Dict[str, Any]]) -> bool:
    """Return whether Overture supplies an explicit height attribute."""
    return bool(
        match is not None
        and match["candidate"].get("height") is not None
        and match["candidate"].get("height_source") != "Overture num_floors"
    )


def add_tag(element: ET.Element, key: str, value: str) -> None:
    tags = {tag.attrib.get("k"): tag for tag in element.findall("tag")}
    if key in tags:
        tags[key].set("v", value)
    else:
        ET.SubElement(element, "tag", {"k": key, "v": value})


def write_osm(tree: ET.ElementTree, output: Path) -> None:
    ET.indent(tree.getroot(), space=" ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    with output.open("ab") as handle:
        handle.write(b"\n")


def summarize_comparison(values: List[float]) -> Dict[str, Any]:
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    return {
        "count": len(values),
        "mean_absolute_difference_m": round(sum(values) / len(values), 3),
        "median_absolute_difference_m": round(ordered[len(ordered) // 2], 3),
        "within_3m_fraction": round(sum(value <= 3.0 for value in values) / len(values), 4),
        "within_5m_fraction": round(sum(value <= 5.0 for value in values) / len(values), 4),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-osm", type=Path, required=True)
    parser.add_argument("--output-osm", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--overture-cli", type=Path, required=True)
    parser.add_argument("--overture-release", default="latest")
    parser.add_argument("--overture-geojson", type=Path)
    parser.add_argument("--overture-land-cover-geojson", type=Path)
    parser.add_argument(
        "--height-mode",
        choices=("none", "overture", "overture-ghs"),
        default="overture",
    )
    parser.add_argument(
        "--footprint-mode",
        choices=("none", "overture-conservative"),
        default="overture-conservative",
    )
    parser.add_argument(
        "--vegetation-mode",
        choices=("none", "overture-conservative"),
        default="overture-conservative",
    )
    parser.add_argument("--visualization-dir", type=Path)
    parser.add_argument("--ghs-country")
    parser.add_argument("--ghs-admin1")
    parser.add_argument("--ghs-csv", type=Path)
    args = parser.parse_args()
    if bool(args.ghs_country) != bool(args.ghs_admin1):
        parser.error("--ghs-country and --ghs-admin1 must be provided together")
    if args.height_mode == "overture-ghs" and not args.ghs_country:
        parser.error("--height-mode overture-ghs requires a GHS region")

    started = time.time()
    tree, buildings, bbox = load_osm(args.input_osm)
    osm_source_buildings = list(buildings)
    osm_source_ids = {building["id"] for building in osm_source_buildings}
    center = ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)
    required_feature_types = ["building"]
    if args.vegetation_mode != "none":
        required_feature_types.append("land_cover")
    release = resolve_overture_release(
        args.overture_cli,
        args.overture_release,
        bbox=bbox,
        cache_root=args.cache_root,
        feature_types=required_feature_types,
    )
    overture_path = args.overture_geojson or download_overture(
        args.overture_cli, bbox, release, args.cache_root
    )
    overture = load_overture(overture_path, center)
    land_cover_path = None
    land_cover: List[Dict[str, Any]] = []
    if args.vegetation_mode != "none":
        land_cover_path = (
            args.overture_land_cover_geojson
            or download_overture(
                args.overture_cli,
                bbox,
                release,
                args.cache_root,
                feature_type="land_cover",
            )
        )
        land_cover = load_overture_land_cover(land_cover_path, center)
    visualization_dir = (
        args.visualization_dir
        or args.report.parent / "building_source_topviews"
    )
    footprint_result = enrich_overture_footprints(
        tree,
        buildings,
        overture,
        bbox,
        center,
        visualization_dir,
        args.footprint_mode,
    )
    buildings.extend(footprint_result["added_buildings"])
    vegetation_result = enrich_overture_vegetation(
        tree,
        land_cover,
        buildings,
        footprint_result["context"],
        center,
        visualization_dir,
        args.vegetation_mode,
    )
    overture_matches = (
        match_overture(buildings, overture)
        if args.height_mode in {"overture", "overture-ghs"}
        else {}
    )

    ghs_path = None
    ghs_member = None
    ghs_rows: List[Dict[str, Any]] = []
    if args.height_mode == "overture-ghs":
        ghs_path, ghs_rows, ghs_member = prepare_ghs(
            args.ghs_country,
            args.ghs_admin1,
            bbox,
            args.cache_root,
            args.ghs_csv,
        )

    already_height = {
        item["id"]
        for item in buildings
        if valid_height(item["tags"].get("height")) is not None
    }
    level_heights = {}
    for item in buildings if args.height_mode != "none" else []:
        levels = parse_number(item["tags"].get("building:levels"))
        if (
            item["id"] not in already_height
            and levels is not None
            and 0 < levels <= 100
        ):
            level_heights[item["id"]] = min(VALID_HEIGHT_MAX_M, 3.0 * levels)
    resolved = already_height | set(level_heights) | set(overture_matches)
    ghs_matches = match_ghs(buildings, ghs_rows, resolved, center)

    changes = []
    source_counts: Counter[str] = Counter()
    method_counts: Counter[str] = Counter()
    for building in buildings:
        if building["id"] in already_height:
            continue
        level_height = level_heights.get(building["id"])
        match = overture_matches.get(building["id"])
        if has_measured_overture_height(match):
            candidate = match["candidate"]
            source = candidate["height_source"]
            reference_id = candidate["id"]
            height = float(candidate["height"])
        elif level_height is not None:
            match = {
                "method": "levels_x_3m",
                "distance_m": 0.0,
            }
            height = level_height
            source = "OpenStreetMap building:levels"
            reference_id = building["id"]
        else:
            if match is not None:
                candidate = match["candidate"]
                source = candidate["height_source"]
                reference_id = candidate["id"]
            else:
                match = ghs_matches.get(building["id"])
                if match is None:
                    continue
                candidate = match["candidate"]
                source = "GHS-OBAT R2024A"
                reference_id = candidate["id"]
            height = float(candidate["height"])
        add_tag(building["element"], "height", f"{height:.2f}")
        add_tag(building["element"], "mosaic:height_source", source)
        confidence = (
            "high" if source == "USGS Lidar"
            else "low" if source == "GHS-OBAT R2024A"
            else "medium"
        )
        add_tag(building["element"], "mosaic:height_method", match["method"])
        add_tag(building["element"], "mosaic:height_confidence", confidence)
        add_tag(building["element"], "mosaic:height_reference_id", reference_id)
        source_counts[source] += 1
        method_counts[match["method"]] += 1
        changes.append(
            {
                "osm_id": building["id"],
                "height_m": round(height, 3),
                "source": source,
                "method": match["method"],
                "reference_id": reference_id,
                "distance_m": round(float(match.get("distance_m", 0.0)), 3),
                "iou": round(float(match.get("iou", 0.0)), 4),
                "area_ratio": round(float(match.get("area_ratio", 0.0)), 4),
            }
        )

    overture_ghs_differences = []
    if ghs_rows:
        comparison_matches = match_ghs(buildings, ghs_rows, already_height, center)
        for osm_id, overture_match in overture_matches.items():
            ghs_match = comparison_matches.get(osm_id)
            if ghs_match is not None:
                overture_ghs_differences.append(
                    abs(
                        float(overture_match["candidate"]["height"])
                        - float(ghs_match["candidate"]["height"])
                    )
                )

    external_height_osm_ids = {
        change["osm_id"]
        for change in changes
        if (
            change["osm_id"] in osm_source_ids
            and change["source"] != "OpenStreetMap building:levels"
        )
    }
    images = render_topviews(
        visualization_dir,
        osm_source_buildings,
        overture,
        footprint_result["accepted"],
        footprint_result["context"],
        overture_vegetation=land_cover,
        accepted_vegetation=vegetation_result["accepted"],
        height_enriched_osm_ids=external_height_osm_ids,
    )
    footprint_report = footprint_result["report"]
    footprint_report["images"] = images
    footprint_report["overture_land_cover_layer_available"] = (
        land_cover_path is not None
    )
    footprint_report["externally_height_enriched_existing_osm_count"] = len(
        external_height_osm_ids
    )
    Path(footprint_report["report"]).write_text(
        json.dumps(footprint_report, indent=2), encoding="utf-8"
    )
    vegetation_report = vegetation_result["report"]
    vegetation_report["images"] = images
    Path(vegetation_report["report"]).write_text(
        json.dumps(vegetation_report, indent=2), encoding="utf-8"
    )

    write_osm(tree, args.output_osm)
    total = len(buildings)
    final_height_count = len(already_height) + len(changes)
    report = {
        "success": True,
        "input_osm": str(args.input_osm),
        "output_osm": str(args.output_osm),
        "bbox": list(bbox),
        "building_count": total,
        "height_mode": args.height_mode,
        "footprint_enrichment": footprint_report,
        "vegetation_enrichment": vegetation_report,
        "baseline": {
            "explicit_height_count": len(already_height),
            "building_levels_count": len(level_heights),
            "effective_height_count": len(already_height) + len(level_heights),
        },
        "overture": {
            "release": release,
            "geojson": str(overture_path),
            "land_cover_geojson": (
                str(land_cover_path) if land_cover_path else None
            ),
            "feature_count": len(overture),
            "land_cover_feature_count": len(land_cover),
            "features_with_height": sum(item["height"] is not None for item in overture),
            "matched_building_count": len(overture_matches),
        },
        "ghs_obat": {
            "enabled": bool(args.ghs_country),
            "release": GHS_RELEASE if args.ghs_country else None,
            "country": args.ghs_country,
            "admin1": args.ghs_admin1,
            "archive_member": ghs_member,
            "csv": str(ghs_path) if ghs_path else None,
            "bbox_record_count": len(ghs_rows),
            "matched_building_count": len(ghs_matches),
        },
        "enrichment": {
            "added_height_count": len(changes),
            "derived_from_osm_levels_count": len(level_heights),
            "external_added_height_count": len(changes) - len(level_heights),
            "final_explicit_height_count": final_height_count,
            "final_coverage_fraction": round(final_height_count / total, 4) if total else 1.0,
            "remaining_without_height_count": total - final_height_count,
            "source_counts": dict(source_counts),
            "method_counts": dict(method_counts),
        },
        "cross_source_agreement": {
            "note": "Agreement is a consistency check, not ground-truth accuracy.",
            "overture_vs_ghs_obat": summarize_comparison(overture_ghs_differences),
        },
        "changes": changes,
        "licenses": {
            "Overture_buildings": "ODbL-1.0; preserve attribution to upstream sources",
            "Overture_land_cover": (
                "ODbL-1.0; source records include ESA WorldCover"
            ),
            "ESA_WorldCover_2020": "CC-BY-4.0",
            "GHS_OBAT_R2024A": "ODbL-1.0",
        },
        "elapsed_seconds": round(time.time() - started, 3),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("BUILDING_HEIGHT_ENRICHMENT_SUCCESS=1")
    print(f"BUILDINGS_TOTAL={total}")
    print(
        "BUILDING_FOOTPRINTS_ADDED="
        f"{len(footprint_result['added_buildings'])}"
    )
    print(
        "BUILDING_FOOTPRINT_TOPVIEW="
        f"{footprint_result['report']['images']['merged_topview']}"
    )
    print(
        "OVERTURE_VEGETATION_AREAS_ADDED="
        f"{len(vegetation_result['added'])}"
    )
    print(f"BUILDING_HEIGHTS_ADDED={len(changes)}")
    print(f"BUILDING_HEIGHTS_FINAL={final_height_count}")
    print(f"BUILDING_HEIGHT_COVERAGE={report['enrichment']['final_coverage_fraction']}")
    print(f"BUILDING_HEIGHT_REPORT={args.report}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"BUILDING_HEIGHT_ENRICHMENT_SUCCESS=0 error={exc!r}", file=sys.stderr)
        raise

