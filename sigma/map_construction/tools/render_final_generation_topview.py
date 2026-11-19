#!/usr/bin/env python3
"""Render the final post-supplement, post-normalization OSM input.

Standalone CLI: python -m sigma.map_construction.tools.render_final_generation_topview --help
The unified Stage I entry is documented in docs/stage1.md.
"""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict

from sigma.map_construction.tools.building_footprint_enrichment import (
    BASKETBALL_COURT,
    GRASS_PITCH,
    OSM_BUILDING,
    TARTAN_TRACK,
    _osm_context,
    render_osm_topview,
)
from sigma.map_construction.tools.enrich_osm_building_heights import load_osm


def _tags(element: ET.Element) -> Dict[str, str]:
    return {
        tag.attrib["k"]: tag.attrib.get("v", "")
        for tag in element.findall("tag")
        if "k" in tag.attrib
    }


def render_final_generation_topview(
    input_osm: Path,
    output: Path,
    report_path: Path,
) -> Dict[str, Any]:
    tree, buildings, bbox = load_osm(input_osm)
    center = ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)
    context = _osm_context(tree, bbox, center)
    root = tree.getroot()
    grass_ids = []
    basketball_ids = []
    tartan_track_ids = []
    for element in list(root):
        tags = _tags(element)
        element_id = str(element.attrib.get("id") or "")
        if (
            tags.get("leisure", "").casefold() == "pitch"
            and (
                tags.get("surface", "").casefold() == "grass"
                or tags.get("landuse", "").casefold() == "grass"
                or tags.get("mosaic:surface_compatibility", "").casefold()
                == "grass_pitch"
            )
        ):
            grass_ids.append(element_id)
        if tags.get("sport", "").casefold() == "basketball":
            basketball_ids.append(element_id)
        if (
            tags.get("leisure", "").casefold() == "track"
            and tags.get("surface", "").casefold()
            in {"tartan", "rubber", "synthetic"}
        ):
            tartan_track_ids.append(element_id)

    output.parent.mkdir(parents=True, exist_ok=True)
    render_osm_topview(output, buildings, context)
    report = {
        "success": True,
        "stage": "final_generation_osm_after_supplement_and_normalization",
        "input_osm": str(input_osm.resolve()),
        "output_image": str(output.resolve()),
        "bbox": list(bbox),
        "building_count": len(buildings),
        "grass_pitch_count": len(context.get("grass_pitches", [])),
        "grass_pitch_ids": grass_ids,
        "basketball_court_count": len(context.get("basketball_courts", [])),
        "basketball_court_ids": basketball_ids,
        "tartan_track_count": len(context.get("tartan_tracks", [])),
        "tartan_track_ids": tartan_track_ids,
        "color_legend": {
            "buildings": list(OSM_BUILDING),
            "grass_pitches": list(GRASS_PITCH),
            "basketball_courts": list(BASKETBALL_COURT),
            "tartan_tracks": list(TARTAN_TRACK),
        },
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-osm", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    report = render_final_generation_topview(
        args.input_osm, args.output, args.report
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
