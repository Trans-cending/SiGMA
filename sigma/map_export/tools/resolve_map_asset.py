#!/usr/bin/env python3
"""Resolve a CARLA map name to one unambiguous Unreal map package."""

import argparse
import json
from pathlib import Path


class MapResolutionError(RuntimeError):
    pass


def _sublevel_packages(directory, package_root, pattern):
    if not directory.is_dir():
        return []
    return [
        "{}/{}".format(package_root, path.stem)
        for path in sorted(directory.glob(pattern))
        if path.is_file()
    ]


def resolve_map_asset(carla_root, map_name, requested_source="auto"):
    name = str(map_name).strip()
    source = str(requested_source).strip().lower()
    if not name or "/" in name or "\\" in name or name in (".", ".."):
        raise MapResolutionError("unsafe map name: {!r}".format(map_name))
    if source not in ("auto", "custom", "official"):
        raise MapResolutionError(
            "map source must be auto, custom, or official; got {!r}".format(
                requested_source
            )
        )

    content = Path(carla_root) / "Unreal" / "CarlaUE4" / "Content"
    custom_dir = content / "CustomMaps" / name
    official_dir = content / "Carla" / "Maps"
    candidates = {
        "custom": {
            "map_source": "custom",
            "map_umap": custom_dir / "{}.umap".format(name),
            "map_package": "/Game/CustomMaps/{0}/{0}".format(name),
            "asset_discovery_root": "/Game/CustomMaps/{}".format(name),
            "expected_sublevels": _sublevel_packages(
                custom_dir,
                "/Game/CustomMaps/{}".format(name),
                "{}_Tile_*.umap".format(name),
            ),
            "auto_discover_tile_sublevels": True,
        },
        "official": {
            "map_source": "official",
            "map_umap": official_dir / "{}.umap".format(name),
            "map_package": "/Game/Carla/Maps/{}".format(name),
            "asset_discovery_root": "/Game/Carla/Maps/Sublevels/{}".format(
                name
            ),
            "expected_sublevels": _sublevel_packages(
                official_dir / "Sublevels" / name,
                "/Game/Carla/Maps/Sublevels/{}".format(name),
                "*.umap",
            ),
            "auto_discover_tile_sublevels": False,
        },
    }

    available = [
        key for key, item in candidates.items() if item["map_umap"].is_file()
    ]
    if source == "auto":
        if len(available) == 0:
            checked = [str(candidates[key]["map_umap"]) for key in candidates]
            raise MapResolutionError(
                "map {!r} was not found; checked: {}".format(
                    name, ", ".join(checked)
                )
            )
        if len(available) > 1:
            raise MapResolutionError(
                "map {!r} exists as both custom and official content; "
                "select --map-source custom or --map-source official".format(name)
            )
        source = available[0]
    elif source not in available:
        raise MapResolutionError(
            "{} map {!r} was not found at {}".format(
                source, name, candidates[source]["map_umap"]
            )
        )

    result = dict(candidates[source])
    result["map_name"] = name
    result["map_umap"] = str(result["map_umap"].resolve())
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--carla-root", required=True)
    parser.add_argument("--map", required=True, dest="map_name")
    parser.add_argument(
        "--map-source", choices=("auto", "custom", "official"), default="auto"
    )
    parser.add_argument("--format", choices=("json", "record"), default="json")
    args = parser.parse_args()

    try:
        result = resolve_map_asset(
            args.carla_root, args.map_name, args.map_source
        )
    except MapResolutionError as exc:
        parser.exit(12, "ERROR: {}\n".format(exc))

    if args.format == "record":
        fields = (
            result["map_source"],
            result["map_package"],
            result["map_umap"],
            result["asset_discovery_root"],
            ",".join(result["expected_sublevels"]),
            "1" if result["auto_discover_tile_sublevels"] else "0",
        )
        if any("|" in value or "\n" in value for value in fields):
            raise MapResolutionError(
                "resolved map fields contain an unsafe delimiter"
            )
        print("|".join(fields))
    else:
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
