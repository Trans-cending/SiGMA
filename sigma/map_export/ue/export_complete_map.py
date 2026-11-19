#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run inside UE4.26 embedded Python and export a complete loaded map.

Uses Unreal's official AssetExportTask + LevelExporterFBX/LevelExporterOBJ.
Python 3.7-compatible for UE4.26.
"""

import collections
import json
import os
import time
import traceback

import unreal

PREFIX = "[MOSAIC_UE_EXPORT]"
STATE = {
    "release_version": "1.0.5",
    "runner": "official_unreal_level_exporter",
    "success": False,
    "checkpoint": True,
    "started_at": time.time(),
    "finished_at": 0.0,
    "finish_reason": "initializing",
    "failure_count": 0,
}


def log(message):
    text = "{} {}".format(PREFIX, message)
    try:
        unreal.log(text)
    except Exception:
        print(text)


def atomic_json_write(path, data):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    temp_path = path + ".tmp"
    with open(temp_path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temp_path, path)


def write_report():
    path = STATE.get("report_file")
    if path:
        atomic_json_write(path, STATE)


def normalize_asset_path(path):
    value = str(path).strip()
    leaf = value.rsplit("/", 1)[-1]
    if "." in leaf:
        value = value.rsplit(".", 1)[0]
    return value


def package_from_object(obj):
    try:
        outer = obj.get_outer()
        if outer is not None:
            return normalize_asset_path(outer.get_path_name())
    except Exception:
        pass
    try:
        return normalize_asset_path(obj.get_path_name())
    except Exception:
        return "<unknown>"


def package_from_actor_path(actor):
    """Resolve an actor's owning map package from its UObject path.

    CARLA's UE4.26 Python wrapper exposes loaded actors through stable
    UObject paths, which provide the owning map package across variations in
    Actor.get_level() and outer-object access. Actor paths use the form:

      /Game/Path/Map.Map:PersistentLevel.ActorName

    Parsing the part before ':PersistentLevel' gives the owning World package
    for persistent-level and streaming-level actors directly from object data.
    """

    try:
        actor_path = str(actor.get_path_name()).strip()
    except Exception:
        actor_path = ""

    marker = ":PersistentLevel"
    if marker in actor_path:
        world_object_path = actor_path.split(marker, 1)[0]
        package = normalize_asset_path(world_object_path)
        if package:
            return package

    # Compatibility fallback for builds where Actor.get_level() is exposed.
    try:
        level = actor.get_level()
    except Exception:
        level = None

    if level is not None:
        for candidate in (level, getattr(level, "get_outer", lambda: None)()):
            if candidate is None:
                continue
            try:
                candidate_path = str(candidate.get_path_name()).strip()
            except Exception:
                continue
            if marker in candidate_path:
                candidate_path = candidate_path.split(marker, 1)[0]
            package = normalize_asset_path(candidate_path)
            if package:
                return package

    return "<unknown>"


def fail(reason, exc=None):
    STATE["success"] = False
    STATE["checkpoint"] = False
    STATE["finish_reason"] = reason
    STATE["failure_count"] = int(STATE.get("failure_count", 0)) + 1
    STATE["finished_at"] = time.time()
    if exc is not None:
        STATE["exception"] = repr(exc)
        STATE["traceback"] = traceback.format_exc()
    write_report()
    log("EXPORT_FAILURE_REASON={}".format(reason))
    raise RuntimeError(reason)


def list_streaming_levels(world):
    """Return streaming levels through APIs exposed by this UE build."""

    getter = getattr(world, "get_streaming_levels", None)
    if callable(getter):
        try:
            levels = list(getter())
            log(
                "STREAMING_LEVEL_ENUMERATION_SOURCE="
                "world.get_streaming_levels count={}".format(
                    len(levels)
                )
            )
            return levels
        except Exception as exc:
            log(
                "STREAMING_LEVEL_METHOD_WARNING={!r}".format(
                    exc
                )
            )

    try:
        levels = list(
            world.get_editor_property("streaming_levels")
        )
        log(
            "STREAMING_LEVEL_ENUMERATION_SOURCE="
            "world.streaming_levels count={}".format(
                len(levels)
            )
        )
        return levels
    except Exception:
        return []


def streaming_package(streaming):
    try:
        return normalize_asset_path(
            streaming.get_world_asset_package_f_name()
        )
    except Exception:
        pass
    try:
        asset = streaming.get_editor_property("world_asset")
        if asset:
            return normalize_asset_path(asset.get_path_name())
    except Exception:
        pass
    return "<unknown>"


def find_streaming_level(world, package_name):
    target = normalize_asset_path(package_name)

    gameplay_statics = getattr(unreal, "GameplayStatics", None)
    getter = (
        getattr(gameplay_statics, "get_streaming_level", None)
        if gameplay_statics is not None
        else None
    )
    if callable(getter):
        for candidate in (
            target,
            target.rsplit("/", 1)[-1],
        ):
            try:
                streaming = getter(world, candidate)
            except Exception:
                streaming = None
            if streaming is not None:
                log(
                    "STREAMING_LEVEL_LOOKUP_SOURCE="
                    "GameplayStatics package={}".format(target)
                )
                return streaming

    for streaming in list_streaming_levels(world):
        if streaming_package(streaming) == target:
            return streaming
    return None


def discover_tile_packages(map_name, main_package, asset_root):
    root = normalize_asset_path(asset_root)
    prefix = "{}_Tile_".format(map_name)
    result = []
    try:
        assets = unreal.EditorAssetLibrary.list_assets(
            root, recursive=True, include_folder=False
        )
    except Exception as exc:
        log("AUTO_DISCOVERY_WARNING={!r}".format(exc))
        return result

    for asset_path in assets:
        package = normalize_asset_path(asset_path)
        leaf = package.rsplit("/", 1)[-1]
        if leaf.startswith(prefix) and package != main_package:
            if package not in result:
                result.append(package)
    return sorted(result)


def find_large_map_manager():
    """Locate the ALargeMapManager actor placed in the persistent level."""
    try:
        actors = list(unreal.EditorLevelLibrary.get_all_level_actors())
    except Exception:
        return None
    for actor in actors:
        try:
            if actor.get_class().get_name() == "LargeMapManager":
                return actor
        except Exception:
            continue
    return None


def tile_grid_id(package):
    """Parse the _X_Y suffix of a <map>_Tile_X_Y package name."""
    leaf = package.rsplit("/", 1)[-1]
    marker = "_Tile_"
    if marker not in leaf:
        return None
    parts = leaf.rsplit(marker, 1)[1].split("_")
    if len(parts) != 2 or not parts[0].isdigit() or not parts[1].isdigit():
        return None
    return int(parts[0]), int(parts[1])


def read_large_map_placement(manager):
    """Read tile placement data from the LargeMapManager actor.

    Returns (source, data, tile_side) where source is one of:
      - "saved_map_tiles": data is {tile_name: (x, y, z)} saved locations
      - "formula": data is (tile_side, tile0_offset) for GetTileLocation math
      - "unavailable": data is None

    CARLA large maps store every tile's actors in tile-local coordinates and
    place the tiles in world space at runtime through
    StreamingLevel->LevelTransform = FTransform(GetTileLocation(id)).
    Editor loads (EditorLevelUtils.add_level_to_world) retain local coordinates,
    so the exporter applies the runtime placement transform.
    """
    tile_side = None
    try:
        tile_side = float(manager.get_editor_property("tile_side"))
    except Exception as exc:
        log("TILE_PLACEMENT_TILE_SIDE_WARNING={!r}".format(exc))

    try:
        raw = manager.get_editor_property("map_tiles")
        tiles = {}
        for key in list(raw.keys()):
            try:
                tile = raw[key]
                name = str(tile.name)
                loc = tile.location
                tiles[name] = (float(loc.x), float(loc.y), float(loc.z))
            except Exception:
                continue
        if tiles:
            return "saved_map_tiles", tiles, tile_side
    except Exception as exc:
        log("TILE_PLACEMENT_MAPTILES_WARNING={!r}".format(exc))

    if tile_side is None:
        return "unavailable", None, None

    try:
        offset = manager.get_editor_property("tile0_offset")
        tile0 = (float(offset.x), float(offset.y), float(offset.z))
    except Exception:
        tile0 = (0.0, 0.0, 0.0)
    return "formula", (tile_side, tile0), tile_side


def expected_tile_locations(source, data, expected_sublevels):
    """Map each expected tile package to its runtime world location."""
    result = {}
    for package in expected_sublevels:
        leaf = package.rsplit("/", 1)[-1]
        if source == "saved_map_tiles":
            for name, loc in data.items():
                if name.rsplit("/", 1)[-1] == leaf:
                    result[package] = loc
                    break
        elif source == "formula":
            grid = tile_grid_id(leaf)
            if grid is None:
                continue
            tile_side, tile0 = data
            x, y = grid
            # Mirror ALargeMapManager::GetTileLocation:
            # FVector(X, -Y, 0) * TileSide + Tile0Offset
            result[package] = (
                x * tile_side + tile0[0],
                -y * tile_side + tile0[1],
                tile0[2],
            )
    return result


def apply_tile_world_offsets(expected_locations, tile_side):
    """Translate tile actors to their runtime world locations.

    Decides once for the whole map whether tiles are stored in tile-local
    coordinates by comparing current actor centroids with the expected
    runtime locations. Maps whose tiles are already stored in absolute
    world coordinates are left untouched.
    """
    if not expected_locations:
        return {"decision": "nothing_to_apply"}

    try:
        actors = list(unreal.EditorLevelLibrary.get_all_level_actors())
    except Exception:
        actors = []

    per_package = collections.defaultdict(list)
    for actor in actors:
        package = package_from_actor_path(actor)
        if package in expected_locations:
            per_package[package].append(actor)

    threshold = 100000.0
    if tile_side and tile_side > 0:
        threshold = tile_side * 0.5

    centroids = {}
    errors = {}
    for package, package_actors in per_package.items():
        if not package_actors:
            continue
        total = [0.0, 0.0, 0.0]
        count = 0
        for actor in package_actors:
            try:
                loc = actor.get_actor_location()
            except Exception:
                continue
            total[0] += float(loc.x)
            total[1] += float(loc.y)
            total[2] += float(loc.z)
            count += 1
        if count == 0:
            continue
        centroid = (total[0] / count, total[1] / count, total[2] / count)
        centroids[package] = centroid
        expected = expected_locations[package]
        errors[package] = (
            (centroid[0] - expected[0]) ** 2
            + (centroid[1] - expected[1]) ** 2
        ) ** 0.5

    if not errors:
        return {"decision": "no_tile_actors_found"}

    max_error = max(errors.values())
    if max_error < threshold:
        log(
            "TILE_PLACEMENT_DECISION=already_positioned "
            "max_centroid_error_cm={} threshold_cm={}".format(
                max_error, threshold
            )
        )
        return {
            "decision": "already_positioned",
            "threshold_cm": threshold,
            "max_centroid_error_cm": max_error,
            "expected_locations": {
                k: list(v) for k, v in expected_locations.items()
            },
        }

    log(
        "TILE_PLACEMENT_DECISION=apply_offsets "
        "max_centroid_error_cm={} threshold_cm={}".format(
            max_error, threshold
        )
    )
    moved_by_package = {}
    failures = 0
    for package, package_actors in per_package.items():
        offset = expected_locations[package]
        moved = 0
        for actor in package_actors:
            try:
                current = actor.get_actor_location()
                actor.set_actor_location(
                    unreal.Vector(
                        current.x + offset[0],
                        current.y + offset[1],
                        current.z + offset[2],
                    ),
                    False,
                    True,
                )
                moved += 1
            except Exception:
                failures += 1
        moved_by_package[package] = moved
        log(
            "TILE_PLACEMENT_APPLIED={} offset=({},{},{}) actors={}".format(
                package, offset[0], offset[1], offset[2], moved
            )
        )

    return {
        "decision": "offsets_applied",
        "threshold_cm": threshold,
        "max_centroid_error_cm": max_error,
        "centroids": {k: list(v) for k, v in centroids.items()},
        "expected_locations": {
            k: list(v) for k, v in expected_locations.items()
        },
        "moved_actor_counts": moved_by_package,
        "move_failure_count": failures,
    }


def ensure_level_loaded(world, package_name):
    package_name = normalize_asset_path(package_name)
    log("SUBLEVEL_REQUEST={}".format(package_name))

    try:
        asset = unreal.EditorAssetLibrary.load_asset(package_name)
        log(
            "SUBLEVEL_ASSET_PRELOAD={} result={}".format(
                package_name, int(asset is not None)
            )
        )
    except Exception as exc:
        log(
            "SUBLEVEL_ASSET_PRELOAD_WARNING={} error={!r}".format(
                package_name, exc
            )
        )

    streaming = find_streaming_level(world, package_name)
    if streaming is None:
        try:
            streaming = unreal.EditorLevelUtils.add_level_to_world(
                world,
                package_name,
                unreal.LevelStreamingAlwaysLoaded,
            )
            log(
                "SUBLEVEL_ADD_TO_WORLD={} result={}".format(
                    package_name, int(streaming is not None)
                )
            )
        except Exception as exc:
            log(
                "SUBLEVEL_ADD_WARNING={} error={!r}".format(
                    package_name, exc
                )
            )
            streaming = None

    if streaming is None:
        streaming = find_streaming_level(world, package_name)
    if streaming is None:
        raise RuntimeError(
            "Unable to find or add streaming level: {}".format(package_name)
        )

    # Set block first, then request load and visibility.
    for prop, value in (
        ("should_block_on_load", True),
        ("disable_distance_streaming", True),
        ("should_be_loaded", True),
        ("should_be_visible", True),
    ):
        try:
            streaming.set_editor_property(prop, value)
        except Exception as exc:
            log(
                "STREAMING_PROPERTY_WARNING={} {} error={!r}".format(
                    package_name, prop, exc
                )
            )

    try:
        loaded_level = streaming.get_loaded_level()
    except Exception:
        loaded_level = None

    if loaded_level is not None:
        try:
            unreal.EditorLevelUtils.set_level_visibility(
                loaded_level, True, True
            )
        except Exception as exc:
            log(
                "LEVEL_VISIBILITY_WARNING={} error={!r}".format(
                    package_name, exc
                )
            )

    try:
        is_loaded = bool(streaming.is_level_loaded())
    except Exception:
        is_loaded = loaded_level is not None
    try:
        is_visible = bool(streaming.is_level_visible())
    except Exception:
        is_visible = loaded_level is not None

    log(
        "SUBLEVEL_STATE={} loaded={} visible={}".format(
            package_name, int(is_loaded), int(is_visible)
        )
    )

    if not is_loaded or loaded_level is None:
        raise RuntimeError(
            "Streaming level did not synchronously load: {}".format(
                package_name
            )
        )

    return (
        {
            "package": package_name,
            "loaded": is_loaded,
            "visible": is_visible,
            "loaded_level_path": loaded_level.get_path_name(),
        },
        loaded_level,
    )


def level_identity(level):
    try:
        return level.get_path_name()
    except Exception:
        return repr(level)


def collect_loaded_levels(world, explicit_levels):
    """Enumerate ULevel objects through actor and streaming-level ownership."""

    levels = []
    seen = set()
    sources = collections.Counter()

    def add(level, source):
        if level is None:
            return
        identity = level_identity(level)
        if identity in seen:
            return
        seen.add(identity)
        levels.append(level)
        sources[source] += 1

    for level in explicit_levels:
        add(level, "explicit_sublevel")

    for method_name in ("get_levels", "get_streaming_levels"):
        getter = getattr(world, method_name, None)
        if not callable(getter):
            continue
        try:
            candidates = list(getter())
        except Exception as exc:
            log(
                "LEVEL_ENUMERATION_METHOD_WARNING={} error={!r}".format(
                    method_name,
                    exc,
                )
            )
            continue

        for candidate in candidates:
            if method_name == "get_streaming_levels":
                try:
                    candidate = candidate.get_loaded_level()
                except Exception:
                    candidate = None
            add(candidate, "world.{}".format(method_name))

    current_level_getter = getattr(
        world,
        "get_current_level",
        None,
    )
    if callable(current_level_getter):
        try:
            add(
                current_level_getter(),
                "world.get_current_level",
            )
        except Exception as exc:
            log(
                "CURRENT_LEVEL_ENUMERATION_WARNING={!r}".format(
                    exc
                )
            )

    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        try:
            add(actor.get_level(), "actor.get_level")
        except Exception:
            pass

    log(
        "LEVEL_ENUMERATION_SOURCE_COUNTS={}".format(
            dict(sources)
        )
    )
    log("LEVEL_ENUMERATION_COUNT={}".format(len(levels)))
    for level in levels:
        log(
            "ENUMERATED_LEVEL={}".format(
                package_from_object(level)
            )
        )

    return levels


def actor_static_mesh_component_count(actor):
    try:
        return len(actor.get_components_by_class(unreal.StaticMeshComponent))
    except Exception:
        return 0


def build_loaded_level_package_evidence(
    persistent_package,
    world_package,
    loaded_levels,
    explicit_sublevel_states,
    actor_level_packages,
):
    """Build package-level evidence for the complete loaded editor world.

    The UWorld package provides authoritative evidence that the persistent
    level is loaded in this CARLA UE4.26 build. Streaming ULevel objects and
    their explicit loaded/visible states provide authoritative evidence for
    Tiles. Actor-owned levels supply an independent consistency signal.
    """

    persistent_package = normalize_asset_path(persistent_package)
    world_package = normalize_asset_path(world_package)

    packages = set()
    sources = collections.defaultdict(list)

    def add(package, source):
        package = normalize_asset_path(package)
        if not package or package == "<unknown>":
            return
        packages.add(package)
        sources[package].append(source)

    if world_package == persistent_package:
        add(persistent_package, "loaded_world_package")

    for level in loaded_levels:
        add(package_from_object(level), "loaded_ulevel_object")

    loaded_sublevel_count = 0
    visible_sublevel_count = 0
    for state in explicit_sublevel_states:
        if not isinstance(state, dict):
            continue
        if state.get("loaded") is True:
            loaded_sublevel_count += 1
            add(state.get("package", ""), "explicit_loaded_sublevel")
        if state.get("visible") is True:
            visible_sublevel_count += 1

    for package in actor_level_packages:
        add(package, "actor_owned_level")

    return {
        "packages": sorted(packages),
        "sources": {
            key: sorted(set(value))
            for key, value in sorted(sources.items())
        },
        "persistent_world_package": world_package,
        "persistent_world_expected_package": persistent_package,
        "persistent_world_counted": persistent_package in packages,
        "explicit_loaded_sublevel_count": loaded_sublevel_count,
        "explicit_visible_sublevel_count": visible_sublevel_count,
        "loaded_ulevel_object_count": len(loaded_levels),
        "actor_level_package_count": len(set(actor_level_packages)),
    }


def audit_world(
    world,
    loaded_levels,
    explicit_sublevel_states,
    persistent_package,
):
    actors = list(unreal.EditorLevelLibrary.get_all_level_actors())
    levels = list(loaded_levels)

    class_counts = collections.Counter()
    per_level = {}
    actor_level_packages = []
    static_mesh_component_count = 0
    actors_with_static_mesh = 0

    for actor in actors:
        try:
            class_name = actor.get_class().get_name()
        except Exception:
            class_name = "<unknown>"
        class_counts[class_name] += 1

        level_name = package_from_actor_path(actor)

        if level_name != "<unknown>":
            actor_level_packages.append(level_name)

        entry = per_level.setdefault(
            level_name,
            {
                "actor_count": 0,
                "static_mesh_component_count": 0,
                "actors_with_static_mesh": 0,
            },
        )
        entry["actor_count"] += 1

        count = actor_static_mesh_component_count(actor)
        static_mesh_component_count += count
        entry["static_mesh_component_count"] += count
        if count > 0:
            actors_with_static_mesh += 1
            entry["actors_with_static_mesh"] += 1

    world_package = normalize_asset_path(world.get_path_name())
    evidence = build_loaded_level_package_evidence(
        persistent_package,
        world_package,
        levels,
        explicit_sublevel_states,
        actor_level_packages,
    )

    return {
        "world_package": world_package,
        "level_count_policy": (
            "unique_package_evidence_from_persistent_world_"
            "explicit_sublevels_ulevel_objects_and_actors"
        ),
        "loaded_level_count": len(evidence["packages"]),
        "loaded_level_packages": evidence["packages"],
        "loaded_level_package_sources": evidence["sources"],
        "persistent_world_counted": evidence["persistent_world_counted"],
        "persistent_world_package": evidence["persistent_world_package"],
        "persistent_world_expected_package": (
            evidence["persistent_world_expected_package"]
        ),
        "explicit_loaded_sublevel_count": (
            evidence["explicit_loaded_sublevel_count"]
        ),
        "explicit_visible_sublevel_count": (
            evidence["explicit_visible_sublevel_count"]
        ),
        "loaded_ulevel_object_count": (
            evidence["loaded_ulevel_object_count"]
        ),
        "actor_level_package_count": (
            evidence["actor_level_package_count"]
        ),
        "actor_count": len(actors),
        "actors_with_static_mesh": actors_with_static_mesh,
        "static_mesh_component_count": static_mesh_component_count,
        "actor_class_counts": dict(class_counts.most_common()),
        "per_level": per_level,
    }


def validate_loaded_world(
    config,
    audit,
    expected_sublevels,
    explicit_sublevel_states,
):
    failures = []
    loaded_packages = set(audit["loaded_level_packages"])
    persistent_package = normalize_asset_path(config["map_package"])

    if audit.get("world_package") != persistent_package:
        failures.append(
            "loaded world package {} != expected persistent package {}".format(
                audit.get("world_package"),
                persistent_package,
            )
        )

    if audit.get("persistent_world_counted") is not True:
        failures.append("persistent world package was not counted")

    expected = [persistent_package] + [
        normalize_asset_path(package)
        for package in expected_sublevels
    ]
    missing = [
        package for package in expected
        if package not in loaded_packages
    ]
    if missing:
        failures.append(
            "missing expected loaded level packages: {}".format(missing)
        )

    states_by_package = {
        normalize_asset_path(state.get("package", "")): state
        for state in explicit_sublevel_states
        if isinstance(state, dict)
    }
    for package in expected_sublevels:
        package = normalize_asset_path(package)
        state = states_by_package.get(package)
        if state is None:
            failures.append(
                "missing explicit sublevel state: {}".format(package)
            )
            continue
        if state.get("loaded") is not True:
            failures.append(
                "sublevel is not loaded: {}".format(package)
            )
        if state.get("visible") is not True:
            failures.append(
                "sublevel is not visible: {}".format(package)
            )

    checks = (
        (
            "loaded_level_count",
            audit["loaded_level_count"],
            int(config.get("minimum_loaded_level_count", 1)),
        ),
        (
            "actor_count",
            audit["actor_count"],
            int(config.get("minimum_actor_count", 1)),
        ),
        (
            "static_mesh_component_count",
            audit["static_mesh_component_count"],
            int(config.get("minimum_static_mesh_component_count", 1)),
        ),
    )
    for label, actual, minimum in checks:
        if actual < minimum:
            failures.append(
                "{} {} < minimum {}".format(label, actual, minimum)
            )
    return failures


def safe_set(obj, prop, value):
    try:
        obj.set_editor_property(prop, value)
        return True
    except Exception as exc:
        log("EXPORT_PROPERTY_WARNING={} error={!r}".format(prop, exc))
        return False


def configure_fbx_options(config):
    settings = config.get("fbx", {})
    options = unreal.FbxExportOption()
    for prop, value in (
        ("ascii", bool(settings.get("ascii", False))),
        ("collision", bool(settings.get("collision", False))),
        ("level_of_detail", bool(settings.get("level_of_detail", False))),
        ("vertex_color", bool(settings.get("vertex_color", True))),
        ("force_front_x_axis", bool(settings.get("force_front_x_axis", False))),
    ):
        safe_set(options, prop, value)
    return options


def component_class_name(component):
    try:
        return component.get_class().get_name()
    except Exception:
        return "<unknown>"


def select_geometry_actors(expected_level_packages):
    """Select every loaded actor with StaticMeshComponent-derived geometry.

    LevelExporterFBX with selected=False exports the persistent World root and
    can omit actors owned by loaded streaming levels. Selecting the geometry
    actors explicitly makes the official exporter process the persistent level
    and every loaded tile in one task.
    """

    unreal.EditorLevelLibrary.clear_actor_selection_set()
    expected = [normalize_asset_path(x) for x in expected_level_packages]
    selected_actor_count = 0
    selected_component_count = 0
    per_level = collections.Counter()
    component_class_counts = collections.Counter()
    actor_class_counts = collections.Counter()
    requested_actor_paths = set()

    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        try:
            components = list(
                actor.get_components_by_class(
                    unreal.StaticMeshComponent
                )
            )
        except Exception:
            components = []
        if not components:
            continue

        unreal.EditorLevelLibrary.set_actor_selection_state(
            actor,
            True,
        )
        selected_actor_count += 1
        selected_component_count += len(components)
        try:
            requested_actor_paths.add(str(actor.get_path_name()))
        except Exception:
            requested_actor_paths.add(repr(actor))

        level_package = package_from_actor_path(actor)
        per_level[level_package] += 1

        try:
            actor_class_counts[
                actor.get_class().get_name()
            ] += 1
        except Exception:
            actor_class_counts["<unknown>"] += 1

        for component in components:
            component_class_counts[
                component_class_name(component)
            ] += 1

    selected_level_packages = sorted(
        package for package, count in per_level.items()
        if count > 0 and package != "<unknown>"
    )
    missing_expected_levels = sorted(
        package for package in expected
        if per_level.get(package, 0) <= 0
    )

    unknown_level_actor_count = int(per_level.get("<unknown>", 0))

    # Verify that the editor selection set contains the actors just selected.
    # LevelExporterFBX(selected=True) consumes this exact selection set.
    selection_query = getattr(
        unreal.EditorLevelLibrary,
        "get_selected_level_actors",
        None,
    )
    selection_state_verification_available = callable(selection_query)
    actual_selected_actor_count = None
    actual_selected_geometry_actor_count = None
    actual_selected_non_geometry_actor_count = None
    actual_selected_component_count = None
    actual_selected_non_geometry_class_counts = collections.Counter()
    missing_selected_geometry_actor_paths = []
    extra_selected_geometry_actor_paths = []
    if selection_state_verification_available:
        try:
            actual_selected_actors = list(selection_query())
            actual_selected_actor_count = len(actual_selected_actors)
            actual_selected_geometry_actor_count = 0
            actual_selected_non_geometry_actor_count = 0
            actual_selected_component_count = 0
            actual_geometry_actor_paths = set()
            for selected_actor in actual_selected_actors:
                try:
                    components = list(
                        selected_actor.get_components_by_class(
                            unreal.StaticMeshComponent
                        )
                    )
                except Exception:
                    components = []
                actual_selected_component_count += len(components)
                if components:
                    actual_selected_geometry_actor_count += 1
                    try:
                        actual_geometry_actor_paths.add(
                            str(selected_actor.get_path_name())
                        )
                    except Exception:
                        actual_geometry_actor_paths.add(repr(selected_actor))
                else:
                    actual_selected_non_geometry_actor_count += 1
                    try:
                        class_name = selected_actor.get_class().get_name()
                    except Exception:
                        class_name = "<unknown>"
                    actual_selected_non_geometry_class_counts[class_name] += 1
            missing_selected_geometry_actor_paths = sorted(
                requested_actor_paths - actual_geometry_actor_paths
            )
            extra_selected_geometry_actor_paths = sorted(
                actual_geometry_actor_paths - requested_actor_paths
            )
        except Exception as exc:
            selection_state_verification_available = False
            log(
                "EDITOR_SELECTION_QUERY_WARNING={!r}".format(
                    exc
                )
            )

    summary = {
        "policy": "all_loaded_geometry_actor_path_v1_0_5",
        "selected_actor_count": selected_actor_count,
        "selected_static_mesh_component_count": (
            selected_component_count
        ),
        "selected_level_count": len(selected_level_packages),
        "selected_level_packages": selected_level_packages,
        "selected_actors_per_level": dict(sorted(per_level.items())),
        "missing_expected_level_geometry": missing_expected_levels,
        "selected_actor_class_counts": dict(
            actor_class_counts.most_common()
        ),
        "selected_component_class_counts": dict(
            component_class_counts.most_common()
        ),
        "unknown_level_actor_count": unknown_level_actor_count,
        "level_resolution_policy": (
            "actor_uobject_path_before_persistent_level"
        ),
        "selection_state_verification_available": (
            selection_state_verification_available
        ),
        "actual_selected_actor_count": actual_selected_actor_count,
        "actual_selected_geometry_actor_count": (
            actual_selected_geometry_actor_count
        ),
        "actual_selected_non_geometry_actor_count": (
            actual_selected_non_geometry_actor_count
        ),
        "actual_selected_static_mesh_component_count": (
            actual_selected_component_count
        ),
        "actual_selected_non_geometry_actor_class_counts": dict(
            actual_selected_non_geometry_class_counts.most_common()
        ),
        "missing_selected_geometry_actor_count": len(
            missing_selected_geometry_actor_paths
        ),
        "extra_selected_geometry_actor_count": len(
            extra_selected_geometry_actor_paths
        ),
        "missing_selected_geometry_actor_paths": (
            missing_selected_geometry_actor_paths[:20]
        ),
        "extra_selected_geometry_actor_paths": (
            extra_selected_geometry_actor_paths[:20]
        ),
    }

    log(
        "SELECTED_GEOMETRY_ACTOR_COUNT={}".format(
            selected_actor_count
        )
    )
    log(
        "SELECTED_STATIC_MESH_COMPONENT_COUNT={}".format(
            selected_component_count
        )
    )
    log(
        "SELECTED_LEVEL_COUNT={}".format(
            len(selected_level_packages)
        )
    )
    log(
        "SELECTED_LEVEL_RESOLUTION_POLICY="
        "actor_uobject_path_before_persistent_level"
    )
    log(
        "SELECTED_UNKNOWN_LEVEL_ACTOR_COUNT={}".format(
            unknown_level_actor_count
        )
    )
    log(
        "EDITOR_SELECTION_STATE_VERIFICATION_AVAILABLE={}".format(
            int(selection_state_verification_available)
        )
    )
    if actual_selected_actor_count is not None:
        log(
            "ACTUAL_SELECTED_ACTOR_COUNT={}".format(
                actual_selected_actor_count
            )
        )
        log(
            "ACTUAL_SELECTED_GEOMETRY_ACTOR_COUNT={}".format(
                actual_selected_geometry_actor_count
            )
        )
        log(
            "ACTUAL_SELECTED_NON_GEOMETRY_ACTOR_COUNT={}".format(
                actual_selected_non_geometry_actor_count
            )
        )
        log(
            "ACTUAL_SELECTED_STATIC_MESH_COMPONENT_COUNT={}".format(
                actual_selected_component_count
            )
        )
        log(
            "ACTUAL_SELECTED_NON_GEOMETRY_ACTOR_CLASS_COUNTS={}".format(
                dict(actual_selected_non_geometry_class_counts.most_common())
            )
        )
        log(
            "SELECTION_GEOMETRY_MISSING_ACTOR_COUNT={}".format(
                len(missing_selected_geometry_actor_paths)
            )
        )
        log(
            "SELECTION_GEOMETRY_EXTRA_ACTOR_COUNT={}".format(
                len(extra_selected_geometry_actor_paths)
            )
        )
    for package in selected_level_packages:
        log(
            "SELECTED_LEVEL={} actors={}".format(
                package,
                per_level[package],
            )
        )
    log(
        "SELECTED_COMPONENT_CLASS_COUNTS={}".format(
            dict(component_class_counts.most_common())
        )
    )

    return summary


def validate_geometry_selection(config, audit, summary):
    failures = []
    if summary["selected_actor_count"] <= 0:
        failures.append("geometry selection selected zero actors")
    if summary["selected_actor_count"] != audit["actors_with_static_mesh"]:
        failures.append(
            "selected actor count {} != audited actors_with_static_mesh {}".format(
                summary["selected_actor_count"],
                audit["actors_with_static_mesh"],
            )
        )
    if (
        summary["selected_static_mesh_component_count"]
        != audit["static_mesh_component_count"]
    ):
        failures.append(
            "selected component count {} != audited component count {}".format(
                summary["selected_static_mesh_component_count"],
                audit["static_mesh_component_count"],
            )
        )
    if summary["missing_expected_level_geometry"]:
        failures.append(
            "expected levels missing selected geometry: {}".format(
                summary["missing_expected_level_geometry"]
            )
        )
    if summary["unknown_level_actor_count"] != 0:
        failures.append(
            "selected geometry actors with unresolved owning level: {}".format(
                summary["unknown_level_actor_count"]
            )
        )
    if summary["selection_state_verification_available"]:
        if (
            summary["actual_selected_geometry_actor_count"]
            != summary["selected_actor_count"]
        ):
            failures.append(
                "actual selected geometry actor count {} != requested {}".format(
                    summary["actual_selected_geometry_actor_count"],
                    summary["selected_actor_count"],
                )
            )
        if summary["missing_selected_geometry_actor_count"]:
            failures.append(
                "requested geometry actors missing from editor selection: {}".format(
                    summary["missing_selected_geometry_actor_paths"]
                )
            )
        if summary["extra_selected_geometry_actor_count"]:
            failures.append(
                "unexpected geometry actors in editor selection: {}".format(
                    summary["extra_selected_geometry_actor_paths"]
                )
            )
        if (
            summary["actual_selected_static_mesh_component_count"]
            != summary["selected_static_mesh_component_count"]
        ):
            failures.append(
                "actual editor selection component count {} != requested {}".format(
                    summary[
                        "actual_selected_static_mesh_component_count"
                    ],
                    summary["selected_static_mesh_component_count"],
                )
            )
    minimum_levels = int(
        config.get("minimum_loaded_level_count", 1)
    )
    if summary["selected_level_count"] < minimum_levels:
        failures.append(
            "selected level count {} < minimum {}".format(
                summary["selected_level_count"],
                minimum_levels,
            )
        )
    return failures


def export_world(world, config, audit, expected_level_packages):
    output_file = os.path.abspath(config["output_file"])
    export_format = str(config["format"]).lower()
    selection_mode = str(
        config.get("selection_mode", "geometry")
    ).lower()
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    task = unreal.AssetExportTask()
    safe_set(task, "object", world)
    safe_set(task, "filename", output_file)
    safe_set(task, "automated", True)
    safe_set(task, "prompt", False)
    safe_set(task, "replace_identical", True)
    safe_set(task, "use_file_archive", False)
    safe_set(task, "write_empty_files", False)

    selection_summary = {
        "policy": "persistent_world_only",
        "selected_actor_count": 0,
        "selected_static_mesh_component_count": 0,
        "selected_level_count": 1,
        "selected_level_packages": [
            normalize_asset_path(config["map_package"])
        ],
        "missing_expected_level_geometry": [],
    }
    if selection_mode == "geometry":
        selection_summary = select_geometry_actors(
            expected_level_packages
        )
        selection_failures = validate_geometry_selection(
            config,
            audit,
            selection_summary,
        )
        if selection_failures:
            raise RuntimeError(
                "all-loaded-geometry selection audit failed: {}".format(
                    selection_failures
                )
            )
        safe_set(task, "selected", True)
        log("ALL_LOADED_GEOMETRY_SELECTION_COMPLETE=1")
    elif selection_mode == "world":
        safe_set(task, "selected", False)
        log("WORLD_MODE_PERSISTENT_LEVEL_ONLY_RISK=1")
    else:
        raise ValueError("selection_mode must be world or geometry")

    if export_format == "fbx":
        exporter = unreal.LevelExporterFBX()
        safe_set(task, "exporter", exporter)
        safe_set(task, "options", configure_fbx_options(config))
    elif export_format == "obj":
        exporter = unreal.LevelExporterOBJ()
        safe_set(task, "exporter", exporter)
    else:
        raise ValueError("format must be fbx or obj")

    log("OFFICIAL_EXPORTER_CLASS={}".format(exporter.get_class().get_name()))
    log("EXPORT_OUTPUT_FILE={}".format(output_file))
    log("EXPORT_SELECTION_MODE={}".format(selection_mode))
    log(
        "EXPORT_SELECTED_ACTOR_COUNT={}".format(
            selection_summary["selected_actor_count"]
        )
    )

    started = time.time()
    result = bool(unreal.Exporter.run_asset_export_task(task))
    elapsed = time.time() - started

    try:
        errors = [str(x) for x in task.get_editor_property("errors")]
    except Exception:
        errors = []

    size = os.path.getsize(output_file) if os.path.isfile(output_file) else 0
    return {
        "export_task_success": result,
        "exporter_class": exporter.get_class().get_name(),
        "format": export_format,
        "selection_mode": selection_mode,
        "selected_actor_count": selection_summary["selected_actor_count"],
        "selection_summary": selection_summary,
        "output_file": output_file,
        "output_exists": os.path.isfile(output_file),
        "output_size_bytes": size,
        "elapsed_seconds": elapsed,
        "task_errors": errors,
    }


def main():
    config_path = os.environ.get("MOSAIC_EXPORT_CONFIG", "").strip()
    if not config_path:
        fail("MOSAIC_EXPORT_CONFIG is not set")

    with open(config_path, "r", encoding="utf-8") as handle:
        config = json.load(handle)

    STATE["config_file"] = os.path.abspath(config_path)
    STATE["report_file"] = os.path.abspath(config["report_file"])
    STATE["run_dir"] = os.path.abspath(config["run_dir"])
    STATE["map_name"] = config["map_name"]
    STATE["map_package"] = normalize_asset_path(config["map_package"])
    STATE["format"] = str(config["format"]).lower()
    STATE["output_file"] = os.path.abspath(config["output_file"])
    STATE["selection_mode"] = config.get("selection_mode", "geometry")
    STATE["finish_reason"] = "loading_map"
    write_report()

    log("EXPORT_RELEASE_VERSION={}".format(STATE["release_version"]))
    log("MAP_NAME={}".format(STATE["map_name"]))
    log("MAP_PACKAGE={}".format(STATE["map_package"]))
    log("EXPORT_FORMAT={}".format(STATE["format"]))
    log("LEVEL_ENUMERATION_POLICY=persistent_world_plus_explicit_sublevels_v1_0_2")
    log("EXPORT_SCOPE_POLICY=all_loaded_geometry_actor_path_v1_0_5")
    log("ACTOR_LEVEL_RESOLUTION_POLICY=actor_uobject_path_v1_0_4")

    loaded = unreal.EditorLevelLibrary.load_level(STATE["map_package"])
    if loaded:
        world = unreal.EditorLevelLibrary.get_editor_world()
    else:
        world = unreal.EditorLoadingAndSavingUtils.load_map(
            STATE["map_package"]
        )
    if world is None:
        fail("persistent map load failed")

    STATE["loaded_world_package"] = normalize_asset_path(
        world.get_path_name()
    )
    log("LOADED_WORLD_PACKAGE={}".format(STATE["loaded_world_package"]))

    expected_sublevels = [
        normalize_asset_path(x)
        for x in config.get("expected_sublevels", [])
    ]
    if config.get("auto_discover_tile_sublevels", True):
        discovered = discover_tile_packages(
            STATE["map_name"],
            STATE["map_package"],
            config.get("asset_discovery_root", STATE["map_package"]),
        )
        STATE["auto_discovered_sublevels"] = discovered
        for package in discovered:
            if package not in expected_sublevels:
                expected_sublevels.append(package)

    expected_sublevels = sorted(set(expected_sublevels))
    STATE["expected_sublevels"] = expected_sublevels
    log("EXPECTED_SUBLEVEL_COUNT={}".format(len(expected_sublevels)))

    STATE["finish_reason"] = "loading_sublevels"
    write_report()
    states = []
    explicitly_loaded_levels = []
    for package in expected_sublevels:
        state, loaded_level = ensure_level_loaded(
            world,
            package,
        )
        states.append(state)
        explicitly_loaded_levels.append(loaded_level)
    STATE["sublevel_states"] = states

    manager = find_large_map_manager()
    if manager is None:
        log("LARGE_MAP_MANAGER_FOUND=0")
        STATE["tile_placement"] = {"decision": "no_large_map_manager"}
    else:
        log("LARGE_MAP_MANAGER_FOUND=1")
        source, data, tile_side = read_large_map_placement(manager)
        STATE["tile_placement_source"] = source
        log("TILE_PLACEMENT_SOURCE={}".format(source))
        if source == "unavailable" and expected_sublevels:
            fail(
                "large map tile placement could not be determined "
                "for a map with tile sublevels"
            )
        locations = expected_tile_locations(
            source,
            data,
            expected_sublevels,
        )
        STATE["tile_placement"] = apply_tile_world_offsets(
            locations,
            tile_side,
        )
        write_report()

    loaded_levels = collect_loaded_levels(
        world,
        explicitly_loaded_levels,
    )

    for level in loaded_levels:
        try:
            unreal.EditorLevelUtils.set_level_visibility(
                level,
                True,
                True,
            )
        except Exception as exc:
            log(
                "LEVEL_VISIBILITY_WARNING={} error={!r}".format(
                    package_from_object(level),
                    exc,
                )
            )

    STATE["finish_reason"] = "auditing_loaded_world"
    audit = audit_world(
        world,
        loaded_levels,
        states,
        STATE["map_package"],
    )
    STATE["world_audit"] = audit
    write_report()

    log("LEVEL_COUNT_POLICY={}".format(audit["level_count_policy"]))
    log(
        "PERSISTENT_WORLD_COUNTED={}".format(
            int(audit["persistent_world_counted"])
        )
    )
    log(
        "LOADED_ULEVEL_OBJECT_COUNT={}".format(
            audit["loaded_ulevel_object_count"]
        )
    )
    log(
        "EXPLICIT_LOADED_SUBLEVEL_COUNT={}".format(
            audit["explicit_loaded_sublevel_count"]
        )
    )
    log("LOADED_LEVEL_COUNT={}".format(audit["loaded_level_count"]))
    log("ACTOR_COUNT={}".format(audit["actor_count"]))
    log(
        "STATIC_MESH_COMPONENT_COUNT={}".format(
            audit["static_mesh_component_count"]
        )
    )
    for package in audit["loaded_level_packages"]:
        log("LOADED_LEVEL={}".format(package))

    failures = validate_loaded_world(
        config,
        audit,
        expected_sublevels,
        states,
    )
    STATE["pre_export_failures"] = failures
    if failures:
        fail("complete-map pre-export audit failed: {}".format(failures))

    STATE["complete_map_audit_passed"] = True
    STATE["finish_reason"] = "exporting"
    write_report()
    log("COMPLETE_MAP_AUDIT_PASSED=1")

    result = export_world(
        world,
        config,
        audit,
        [STATE["map_package"]] + expected_sublevels,
    )
    STATE["export_result"] = result

    export_failures = []
    if result["export_task_success"] is not True:
        export_failures.append("official export task returned false")
    if result["output_exists"] is not True:
        export_failures.append("output file was not created")
    minimum_size = int(config.get("minimum_output_size_bytes", 1))
    if result["output_size_bytes"] < minimum_size:
        export_failures.append(
            "output size {} < minimum {}".format(
                result["output_size_bytes"], minimum_size
            )
        )
    if result["task_errors"]:
        export_failures.append(
            "export task errors: {}".format(result["task_errors"])
        )

    STATE["export_failures"] = export_failures
    if export_failures:
        fail("official level export failed: {}".format(export_failures))

    STATE["success"] = True
    STATE["checkpoint"] = False
    STATE["finish_reason"] = "official_level_export_completed"
    STATE["failure_count"] = 0
    STATE["finished_at"] = time.time()
    write_report()

    log("EXPORT_TASK_SUCCESS=1")
    log("EXPORT_OUTPUT_SIZE_BYTES={}".format(result["output_size_bytes"]))
    log("EXPORT_REPORT={}".format(STATE["report_file"]))
    log("MAP_EXPORT_SUCCESS=1")


try:
    main()
except Exception as exc:
    if STATE.get("checkpoint", True):
        STATE["success"] = False
        STATE["checkpoint"] = False
        STATE["finish_reason"] = "unhandled_exception"
        STATE["failure_count"] = int(STATE.get("failure_count", 0)) + 1
        STATE["finished_at"] = time.time()
        STATE["exception"] = repr(exc)
        STATE["traceback"] = traceback.format_exc()
        try:
            write_report()
        except Exception:
            pass
    log("UNHANDLED_EXCEPTION={!r}".format(exc))
    log(traceback.format_exc())
    raise
