#!/usr/bin/env python3
"""Apply explicit, audited OSM compatibility transforms for map generation."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Tuple


def tags(element: ET.Element) -> dict[str, str]:
    return {
        tag.attrib["k"]: tag.attrib.get("v", "")
        for tag in element.findall("tag")
        if "k" in tag.attrib
    }


def set_tag(element: ET.Element, key: str, value: str) -> None:
    for tag in element.findall("tag"):
        if tag.attrib.get("k") == key:
            tag.set("v", value)
            return
    ET.SubElement(element, "tag", {"k": key, "v": value})


def has_valid_height(properties: dict[str, str]) -> bool:
    for key in ("height", "building:levels"):
        raw = properties.get(key, "").strip()
        if not raw:
            continue
        try:
            if math.isfinite(float(raw)) and float(raw) > 0.0:
                return True
        except ValueError:
            continue
    return False


def normalize(
    input_osm: Path,
    output_osm: Path,
    report_path: Path,
    *,
    promote_grass_pitches: bool,
    grass_pitch_way_ids: Tuple[str, ...],
    promote_service_roads: bool,
    missing_building_height_m: Optional[float],
) -> dict[str, object]:
    tree = ET.parse(input_osm)
    root = tree.getroot()
    grass_pitch_ids: list[str] = []
    promoted_service_ids: list[str] = []
    default_height_ids: list[str] = []

    explicit_grass_ids = set(grass_pitch_way_ids)
    explicit_grass_pitch_ids: list[str] = []
    for element in [*root.findall("way"), *root.findall("relation")]:
        properties = tags(element)
        element_id = element.attrib.get("id", "")
        explicit_grass_pitch = (
            element.tag == "way"
            and element_id in explicit_grass_ids
            and properties.get("leisure", "").casefold() == "pitch"
        )
        if (
            promote_grass_pitches
            and properties.get("leisure", "").casefold() == "pitch"
            and (
                properties.get("surface", "").casefold() == "grass"
                or explicit_grass_pitch
            )
            and not properties.get("landuse")
        ):
            if explicit_grass_pitch and not properties.get("surface"):
                set_tag(element, "surface", "grass")
                set_tag(element, "mosaic:surface_source", "configured_way_override")
                explicit_grass_pitch_ids.append(element_id)
            set_tag(element, "landuse", "grass")
            set_tag(element, "mosaic:surface_compatibility", "grass_pitch")
            grass_pitch_ids.append(element_id)

        if (
            promote_service_roads
            and properties.get("highway", "").casefold() == "service"
        ):
            set_tag(element, "highway", "residential")
            set_tag(element, "mosaic:original_highway", "service")
            set_tag(element, "mosaic:xodr_compatibility", "drivable_service")
            promoted_service_ids.append(element_id)

        if (
            missing_building_height_m is not None
            and properties.get("building", "").casefold() not in {"", "no"}
            and not has_valid_height(properties)
        ):
            set_tag(element, "height", f"{missing_building_height_m:.2f}")
            set_tag(element, "mosaic:height_source", "configured_default")
            default_height_ids.append(element_id)

    ET.indent(root, space=" ")
    output_osm.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_osm, encoding="utf-8", xml_declaration=True)
    with output_osm.open("ab") as stream:
        stream.write(b"\n")

    report: dict[str, object] = {
        "success": True,
        "input_osm": str(input_osm),
        "output_osm": str(output_osm),
        "promote_grass_pitches": promote_grass_pitches,
        "promote_service_roads": promote_service_roads,
        "missing_building_height_m": missing_building_height_m,
        "grass_pitch_count": len(grass_pitch_ids),
        "grass_pitch_ids": grass_pitch_ids,
        "explicit_grass_pitch_ids": explicit_grass_pitch_ids,
        "promoted_service_road_count": len(promoted_service_ids),
        "promoted_service_road_ids": promoted_service_ids,
        "default_building_height_count": len(default_height_ids),
        "default_building_height_ids": default_height_ids,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-osm", type=Path, required=True)
    parser.add_argument("--output-osm", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--promote-grass-pitches", action="store_true")
    parser.add_argument("--grass-pitch-way-id", action="append", default=[])
    parser.add_argument("--promote-service-roads", action="store_true")
    parser.add_argument("--missing-building-height-m", type=float)
    args = parser.parse_args()
    if args.missing_building_height_m is not None:
        if not math.isfinite(args.missing_building_height_m):
            raise ValueError("missing building height must be finite")
        if not 1.0 <= args.missing_building_height_m <= 200.0:
            raise ValueError("missing building height must be between 1 and 200 m")
    report = normalize(
        args.input_osm,
        args.output_osm,
        args.report,
        promote_grass_pitches=args.promote_grass_pitches,
        grass_pitch_way_ids=tuple(args.grass_pitch_way_id),
        promote_service_roads=args.promote_service_roads,
        missing_building_height_m=args.missing_building_height_m,
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
