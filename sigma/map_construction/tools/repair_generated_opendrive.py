#!/usr/bin/env python3
"""Repair and validate the OpenDRIVE emitted by the DTT OSM import.

Standalone CLI: python -m sigma.map_construction.tools.repair_generated_opendrive --help
The unified Stage I entry is documented in docs/stage1.md.
"""

from __future__ import annotations

import argparse
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

from sigma.map_construction.tools.prepare_opendrive import (
    demote_isolated_driving_lanes,
    driving_topology_components,
    remove_dangling_references,
    validate_driving_topology_components,
    write_atomic,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--xodr", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    started = time.time()
    xodr_path = args.xodr.resolve()
    root = ET.parse(str(xodr_path)).getroot()
    removed = remove_dangling_references(root)

    candidate_text = ET.tostring(root, encoding="unicode") + "\n"
    candidate_map = carla.Map(args.map_name, candidate_text)
    components_before = driving_topology_components(candidate_map)
    demoted = demote_isolated_driving_lanes(root, components_before)

    final_text = ET.tostring(root, encoding="unicode") + "\n"
    final_map = carla.Map(args.map_name, final_text)
    components_after = driving_topology_components(final_map)
    topology_count = len(final_map.get_topology())
    waypoint_count = len(final_map.generate_waypoints(2.0))
    if topology_count <= 0 or waypoint_count <= 0:
        raise RuntimeError("CARLA parsed no driving topology from repaired xodr")
    primary_topology_fraction = validate_driving_topology_components(
        components_after,
        "repair",
    )

    write_atomic(xodr_path, final_text)
    report = {
        "success": True,
        "map_name": args.map_name,
        "xodr": str(xodr_path),
        "removed_dangling_references": removed,
        "driving_topology_component_sizes_before": [
            len(component) for component in components_before
        ],
        "demoted_isolated_driving_lanes": demoted,
        "driving_topology_component_sizes_after": [
            len(component) for component in components_after
        ],
        "primary_driving_topology_fraction": primary_topology_fraction,
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
