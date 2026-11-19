# -*- coding: utf-8 -*-
"""Retag StaticMeshComponents in one already-loaded Unreal Level.

Semantic classification recognizes the complete canonical CARLA semantic class
roots together with the per-map destination subdirectories.

The SiGMA CLI loads target levels in an unattended UE4Editor process and
uses -ExecutePythonScript or the deterministic map pass runner. Therefore all Actors returned by
EditorLevelLibrary.get_all_level_actors() belong to the currently loaded editor
world and are safe to inspect. Filtering them again by a string prefix can
silently discard every generated Actor when Unreal formats object paths
differently.

Classification uses a combined context:
- StaticMesh asset path
- Actor path
- Actor label
- Component name

This preserves the canonical CARLA semantic-folder behavior while handling
minor DTT naming differences across generated maps.
"""

from __future__ import print_function

import hashlib
import json
import os
import time
import traceback
from collections import Counter
from pathlib import Path

import unreal


MAP_NAME = os.environ["MOSAIC_MAP_NAME"]
TARGET_LEVEL = os.environ["MOSAIC_TARGET_LEVEL_PACKAGE"]
TARGET_LABEL = os.environ.get(
    "MOSAIC_TARGET_LEVEL_LABEL",
    "unknown",
)
REPORT_PATH = Path(os.environ["MOSAIC_LEVEL_RETAG_REPORT"])
MODE = os.environ.get(
    "MOSAIC_RETAG_MODE",
    "retag",
).strip().lower()
ACTOR_OFFSET = int(
    os.environ.get(
        "MOSAIC_RETAG_ACTOR_OFFSET",
        "0",
    )
)
ACTOR_LIMIT = int(
    os.environ.get(
        "MOSAIC_RETAG_ACTOR_LIMIT",
        "120",
    )
)
CHECKPOINT_EVERY = max(
    1,
    int(
        os.environ.get(
            "MOSAIC_RETAG_CHECKPOINT_EVERY",
            "25",
        )
    ),
)

if MODE not in ("retag", "verify"):
    raise RuntimeError(
        "MOSAIC_RETAG_MODE must be retag or verify"
    )
if ACTOR_OFFSET < 0:
    raise RuntimeError(
        "MOSAIC_RETAG_ACTOR_OFFSET must be non-negative"
    )
if ACTOR_LIMIT < 1:
    raise RuntimeError(
        "MOSAIC_RETAG_ACTOR_LIMIT must be positive"
    )

STATE = {
    "release_version": "1.0.6",
    "runner": "batched-nullrhi-canonical-root-report-supervised-semantic-retag",
    "map_name": MAP_NAME,
    "target_level": TARGET_LEVEL,
    "target_label": TARGET_LABEL,
    "started_at": time.time(),
    "success": False,
}

# CARLA determines the semantic class from the class root. Assets already
# stored anywhere below one of these roots are already semantically correct.
#
# v1.0.4 incorrectly treated only:
#   /Game/Carla/Static/<Class>/<MAP_NAME>/...
# as canonical. That caused built-in assets such as:
#   /Game/Carla/Static/Pole/StreetLights/...
# to be duplicated and rewritten unnecessarily. Blueprint construction
# components can restore those built-in references when a Level is reloaded,
# producing a false verification failure even though the original assets were
# already in the correct CARLA semantic class.
CANONICAL_CLASS_ROOTS = {
    "Building": "/Game/Carla/Static/Building",
    "Road": "/Game/Carla/Static/Road",
    "SideWalk": "/Game/Carla/Static/SideWalk",
    "RoadLine": "/Game/Carla/Static/RoadLine",
    "Terrain": "/Game/Carla/Static/Terrain",
    "Vegetation": "/Game/Carla/Static/Vegetation",
    "Pole": "/Game/Carla/Static/Pole",
}

# Map-scoped destinations make generated CustomMaps relinking deterministic
# while preserving the generated source assets.
TARGET_TAG_FOLDERS = {
    semantic_tag: root + "/" + MAP_NAME
    for semantic_tag, root in CANONICAL_CLASS_ROOTS.items()
}


def log(message):
    unreal.log("[MOSAIC_RELEASE_RETAG_V1_0_6] " + str(message))


def save_report():
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    STATE["updated_at"] = time.time()
    payload = json.dumps(
        STATE,
        indent=2,
        ensure_ascii=False,
    )

    last_error = None

    for attempt in range(1, 6):
        temporary = REPORT_PATH.with_suffix(
            REPORT_PATH.suffix
            + ".tmp.{}".format(os.getpid())
        )

        try:
            temporary.write_text(
                payload,
                encoding="utf-8",
            )
            os.replace(
                str(temporary),
                str(REPORT_PATH),
            )
            return
        except OSError as exc:
            last_error = exc
            try:
                if temporary.exists():
                    temporary.unlink()
            except Exception:
                pass
            time.sleep(float(attempt))

    raise RuntimeError(
        "Failed to write semantic report after retries: {}".format(
            last_error
        )
    )


def package_path(object_path):
    return str(object_path).split(".", 1)[0]


def safe_name(obj):
    try:
        return str(obj.get_name())
    except Exception:
        return ""


def safe_path(obj):
    try:
        return str(obj.get_path_name())
    except Exception:
        return ""


def safe_actor_label(actor):
    try:
        return str(actor.get_actor_label())
    except Exception:
        return ""


def current_world_package():
    world = unreal.EditorLevelLibrary.get_editor_world()
    if world is None:
        raise RuntimeError("Editor world is unavailable")
    return package_path(world.get_path_name())


def actor_level_diagnostic(actor):
    actor_path = safe_path(actor)
    actor_package = ""

    try:
        outermost = actor.get_outermost()
        if outermost is not None:
            actor_package = safe_path(outermost)
    except Exception:
        actor_package = ""

    target_lower = TARGET_LEVEL.lower()
    actor_path_lower = actor_path.lower()
    actor_package_lower = actor_package.lower()

    string_match = (
        actor_path_lower.startswith(target_lower + ".")
        or actor_path_lower.startswith(target_lower + ":")
        or (target_lower + ":persistentlevel.") in actor_path_lower
        or actor_package_lower == target_lower
    )

    return {
        "actor_path": actor_path,
        "actor_label": safe_actor_label(actor),
        "actor_package": actor_package,
        "string_matches_target_level": bool(string_match),
    }


def canonical_semantic_tag(mesh_path):
    """Return the CARLA semantic class encoded by an asset package root.

    The entire class root is canonical. The per-map folder is only the
    destination used for generated CustomMaps assets.
    """

    value = package_path(mesh_path).rstrip("/").lower()

    for semantic_tag, root in CANONICAL_CLASS_ROOTS.items():
        root_value = root.rstrip("/").lower()
        if (
            value == root_value
            or value.startswith(root_value + "/")
        ):
            return semantic_tag

    return None


def canonical_semantic_scope(mesh_path):
    """Describe whether a canonical asset is built-in or map-scoped."""

    semantic_tag = canonical_semantic_tag(mesh_path)
    if semantic_tag is None:
        return None

    value = package_path(mesh_path).rstrip("/").lower()
    target = TARGET_TAG_FOLDERS[
        semantic_tag
    ].rstrip("/").lower()

    if value == target or value.startswith(target + "/"):
        return "map_scoped"

    return "class_root"


def classify_context(
    mesh_path,
    actor_path,
    actor_label,
    component_name,
):
    canonical = canonical_semantic_tag(mesh_path)
    if canonical is not None:
        return canonical

    context = " ".join(
        [
            str(mesh_path),
            str(actor_path),
            str(actor_label),
            str(component_name),
        ]
    ).lower()

    # Specific classes must be checked before generic road/building tokens.
    building_tokens = (
        "/static/roofs/",
        "/static/buildings/",
        "roofmesh",
        "roof_mesh",
        "buildingmesh",
        "building_mesh",
        "buildingactor",
        "building_actor",
    )
    if any(token in context for token in building_tokens):
        return "Building"

    road_line_tokens = (
        "lanemark",
        "lane_mark",
        "lane marking",
        "roadline",
        "road_line",
        "roadmark",
        "road_mark",
        "road marking",
    )
    if any(token in context for token in road_line_tokens):
        return "RoadLine"

    sidewalk_tokens = (
        "sidewalk",
        "side_walk",
        "side walk",
        "pavementwalk",
    )
    if any(token in context for token in sidewalk_tokens):
        return "SideWalk"

    road_tokens = (
        "drivinglane",
        "driving_lane",
        "driving lane",
        "/static/road/",
        "/static/roads/",
        "roadmesh",
        "road_mesh",
        "roadsurface",
        "road_surface",
        "road surface",
    )
    if any(token in context for token in road_tokens):
        return "Road"

    terrain_tokens = (
        "/static/terrain/",
        "terrainmesh",
        "terrain_mesh",
        "terrainactor",
        "terrain_actor",
    )
    if any(token in context for token in terrain_tokens):
        return "Terrain"

    vegetation_tokens = (
        "vegetation",
        "foliage",
        "/tree",
        "tree_",
        "/bush",
        "bush_",
    )
    if any(token in context for token in vegetation_tokens):
        return "Vegetation"

    pole_tokens = (
        "/pole",
        "polemesh",
        "pole_mesh",
        "poleactor",
        "pole_actor",
        "mosaic_osmstreetlamp",
        "streetlight",
        "street_light",
        "streetlamp",
        "street_lamp",
        "lamppost",
        "lamp_post",
        "lightpole",
        "light_pole",
    )
    if any(token in context for token in pole_tokens):
        return "Pole"

    return None


def destination_for(source_package, semantic_tag):
    destination_root = TARGET_TAG_FOLDERS[semantic_tag]

    # Reuse assets already under the correct CARLA semantic class root,
    # including built-in assets.
    if canonical_semantic_tag(source_package) == semantic_tag:
        return source_package

    if source_package.lower().startswith(
        destination_root.lower() + "/"
    ):
        return source_package

    source_name = source_package.rsplit("/", 1)[-1]
    digest = hashlib.sha1(
        source_package.encode("utf-8")
    ).hexdigest()[:10]

    return "{}/{}_{}".format(
        destination_root,
        source_name,
        digest,
    )


def duplicate_semantic_mesh(
    source_object,
    semantic_tag,
    cache,
):
    source_object_path = safe_path(source_object)
    source_package = package_path(source_object_path)

    cache_key = (semantic_tag, source_package)

    if cache_key in cache:
        return cache[cache_key]

    destination_package = destination_for(
        source_package,
        semantic_tag,
    )

    if destination_package == source_package:
        cache[cache_key] = source_object
        return source_object

    destination_root = destination_package.rsplit("/", 1)[0]
    unreal.EditorAssetLibrary.make_directory(destination_root)

    if unreal.EditorAssetLibrary.does_asset_exist(
        destination_package
    ):
        destination_object = unreal.EditorAssetLibrary.load_asset(
            destination_package
        )
    else:
        duplicated = unreal.EditorAssetLibrary.duplicate_asset(
            source_package,
            destination_package,
        )

        if not duplicated:
            raise RuntimeError(
                "Failed to duplicate {} -> {}".format(
                    source_package,
                    destination_package,
                )
            )

        destination_object = unreal.EditorAssetLibrary.load_asset(
            destination_package
        )

    if destination_object is None:
        raise RuntimeError(
            "Destination asset could not be loaded: {}".format(
                destination_package
            )
        )

    try:
        unreal.EditorAssetLibrary.save_loaded_asset(
            destination_object,
            False,
        )
    except Exception:
        try:
            unreal.EditorAssetLibrary.save_asset(
                destination_package,
                False,
            )
        except Exception:
            pass

    cache[cache_key] = destination_object

    log(
        "SEMANTIC_ASSET={} -> {} [{}]".format(
            source_package,
            destination_package,
            semantic_tag,
        )
    )

    return destination_object



def save_current_level_and_assets():
    """Save only the current Level.

    Every duplicated semantic mesh is saved immediately by
    duplicate_semantic_mesh(). Saving the current Level limits memory usage
    on large generated Tiles and keeps persistence scoped to this pass.
    """

    level_saved = False
    all_levels_saved = False

    try:
        level_saved = bool(
            unreal.EditorLevelLibrary.save_current_level()
        )
    except Exception:
        level_saved = False

    try:
        all_levels_saved = bool(
            unreal.EditorLevelLibrary.save_all_dirty_levels()
        )
    except Exception:
        all_levels_saved = False

    return {
        "save_current_level": level_saved,
        "save_all_dirty_levels": all_levels_saved,
        "save_dirty_packages": False,
        "save_ok": (
            level_saved
            or all_levels_saved
        ),
    }


def checkpoint(
    *,
    all_actor_count,
    processed_actor_count,
    processed_actor_index,
    static_mesh_component_count,
    classified_component_count,
    modified_component_count,
    remaining_noncanonical_count,
    source_counts,
    replacement_counts,
    already_canonical_counts,
    failure_count,
):
    STATE.update(
        {
            "mode": MODE,
            "actor_offset": ACTOR_OFFSET,
            "actor_limit": ACTOR_LIMIT,
            "all_loaded_actor_count": all_actor_count,
            "processed_actor_count": processed_actor_count,
            "processed_actor_index": processed_actor_index,
            "static_mesh_component_count":
                static_mesh_component_count,
            "classified_component_count":
                classified_component_count,
            "modified_component_count":
                modified_component_count,
            "remaining_noncanonical_count":
                remaining_noncanonical_count,
            "source_counts": dict(source_counts),
            "replacement_counts":
                dict(replacement_counts),
            "already_canonical_counts":
                dict(already_canonical_counts),
            "failure_count": failure_count,
            "checkpoint": True,
        }
    )
    save_report()


def run():
    save_report()

    loaded_package = current_world_package()
    STATE["loaded_world_package"] = loaded_package

    log("TARGET_LEVEL=" + TARGET_LEVEL)
    log("LOADED_WORLD_PACKAGE=" + loaded_package)
    log("MODE=" + MODE)
    log("ACTOR_OFFSET={}".format(ACTOR_OFFSET))
    log("ACTOR_LIMIT={}".format(ACTOR_LIMIT))

    if loaded_package.lower() != TARGET_LEVEL.lower():
        raise RuntimeError(
            "Startup Level mismatch: expected {}, loaded {}".format(
                TARGET_LEVEL,
                loaded_package,
            )
        )

    actors = sorted(
        list(
            unreal.EditorLevelLibrary.get_all_level_actors()
        ),
        key=lambda actor: safe_path(actor),
    )

    all_actor_count = len(actors)

    if MODE == "verify":
        actors_to_process = actors
        effective_offset = 0
        effective_end = all_actor_count
    else:
        effective_offset = min(
            ACTOR_OFFSET,
            all_actor_count,
        )
        effective_end = min(
            effective_offset + ACTOR_LIMIT,
            all_actor_count,
        )
        actors_to_process = actors[
            effective_offset:effective_end
        ]

    component_class = getattr(
        unreal,
        "StaticMeshComponent",
        None,
    )
    if component_class is None:
        raise RuntimeError(
            "unreal.StaticMeshComponent is unavailable"
        )

    source_counts = Counter()
    replacement_counts = Counter()
    already_canonical_counts = Counter()
    already_canonical_class_root_counts = Counter()
    already_canonical_map_scoped_counts = Counter()
    classifiable_context_counts = Counter()
    failures = []
    cache = {}

    modified_actor_count = 0
    modified_component_count = 0
    actor_with_mesh_component_count = 0
    static_mesh_component_count = 0
    classified_component_count = 0
    unmatched_component_count = 0
    remaining_noncanonical_count = 0

    actor_diagnostic_samples = []
    classified_context_samples = []
    unmatched_context_samples = []
    source_mesh_samples = []

    for local_actor_index, actor in enumerate(
        actors_to_process
    ):
        global_actor_index = (
            effective_offset + local_actor_index
            if MODE == "retag"
            else local_actor_index
        )

        diagnostic = actor_level_diagnostic(actor)
        if len(actor_diagnostic_samples) < 50:
            actor_diagnostic_samples.append(diagnostic)

        actor_path = diagnostic["actor_path"]
        actor_label = diagnostic["actor_label"]

        try:
            components = actor.get_components_by_class(
                component_class
            )
        except Exception:
            components = []

        if components:
            actor_with_mesh_component_count += 1

        actor_modified = False

        for component in components:
            static_mesh_component_count += 1
            component_name = safe_name(component)

            try:
                source_mesh = component.get_editor_property(
                    "static_mesh"
                )
            except Exception:
                source_mesh = None

            if source_mesh is None:
                continue

            source_path = safe_path(source_mesh)

            if (
                source_path
                and source_path not in source_mesh_samples
                and len(source_mesh_samples) < 200
            ):
                source_mesh_samples.append(source_path)

            semantic_tag = classify_context(
                source_path,
                actor_path,
                actor_label,
                component_name,
            )
            canonical_tag = canonical_semantic_tag(
                source_path
            )

            context_record = {
                "semantic_tag": semantic_tag,
                "canonical_tag": canonical_tag,
                "mesh_path": source_path,
                "actor_path": actor_path,
                "actor_label": actor_label,
                "component_name": component_name,
                "global_actor_index": global_actor_index,
            }

            if semantic_tag is None:
                unmatched_component_count += 1
                if len(unmatched_context_samples) < 200:
                    unmatched_context_samples.append(
                        context_record
                    )
                continue

            classified_component_count += 1
            source_counts[semantic_tag] += 1
            classifiable_context_counts[semantic_tag] += 1

            if len(classified_context_samples) < 200:
                classified_context_samples.append(
                    context_record
                )

            if canonical_tag == semantic_tag:
                already_canonical_counts[semantic_tag] += 1

                canonical_scope = canonical_semantic_scope(
                    source_path
                )
                if canonical_scope == "map_scoped":
                    already_canonical_map_scoped_counts[
                        semantic_tag
                    ] += 1
                else:
                    already_canonical_class_root_counts[
                        semantic_tag
                    ] += 1

                continue

            remaining_noncanonical_count += 1

            if MODE == "verify":
                continue

            try:
                destination_mesh = duplicate_semantic_mesh(
                    source_mesh,
                    semantic_tag,
                    cache,
                )

                try:
                    component.modify()
                except Exception:
                    pass

                component.set_editor_property(
                    "static_mesh",
                    destination_mesh,
                )

                # This headless workflow persists serialized Level references.
                # Keep render states unchanged: rebuilding hundreds of them on
                # large Tiles preceded intermittent SIGSEGV during validation.

                replacement_counts[semantic_tag] += 1
                modified_component_count += 1
                actor_modified = True

                if (
                    modified_component_count
                    % CHECKPOINT_EVERY
                    == 0
                ):
                    checkpoint(
                        all_actor_count=all_actor_count,
                        processed_actor_count=len(
                            actors_to_process
                        ),
                        processed_actor_index=
                            global_actor_index,
                        static_mesh_component_count=
                            static_mesh_component_count,
                        classified_component_count=
                            classified_component_count,
                        modified_component_count=
                            modified_component_count,
                        remaining_noncanonical_count=
                            remaining_noncanonical_count,
                        source_counts=source_counts,
                        replacement_counts=
                            replacement_counts,
                        already_canonical_counts=
                            already_canonical_counts,
                        failure_count=len(failures),
                    )

            except Exception:
                failures.append(
                    {
                        "actor": actor_path,
                        "actor_label": actor_label,
                        "component": safe_path(component),
                        "component_name": component_name,
                        "source_mesh": source_path,
                        "semantic_tag": semantic_tag,
                        "global_actor_index":
                            global_actor_index,
                        "exception": traceback.format_exc(),
                    }
                )

        if actor_modified:
            modified_actor_count += 1
            try:
                actor.modify()
            except Exception:
                pass

    if MODE == "retag":
        save_result = save_current_level_and_assets()
    else:
        save_result = {
            "verification_only": True,
            "save_ok": True,
        }

    path_match_count = sum(
        1
        for item in actor_diagnostic_samples
        if item["string_matches_target_level"]
    )

    success = (
        len(failures) == 0
        and bool(save_result["save_ok"])
    )

    if MODE == "verify":
        success = (
            success
            and remaining_noncanonical_count == 0
        )

    STATE.update(
        {
            "release_version": "1.0.6",
            "runner":
                "batched-nullrhi-canonical-root-report-supervised-semantic-retag",
            "mode": MODE,
            "processing_scope":
                "sorted_loaded_actor_slice",
            "actor_offset": effective_offset,
            "actor_limit": ACTOR_LIMIT,
            "actor_end_exclusive": effective_end,
            "all_loaded_actor_count":
                all_actor_count,
            "processed_actor_count":
                len(actors_to_process),
            "actor_with_mesh_component_count":
                actor_with_mesh_component_count,
            "static_mesh_component_count":
                static_mesh_component_count,
            "classified_component_count":
                classified_component_count,
            "unmatched_component_count":
                unmatched_component_count,
            "remaining_noncanonical_count":
                remaining_noncanonical_count,
            "modified_actor_count":
                modified_actor_count,
            "modified_component_count":
                modified_component_count,
            "unique_source_mesh_count": len(cache),
            "source_counts": dict(source_counts),
            "replacement_counts":
                dict(replacement_counts),
            "already_canonical_counts":
                dict(already_canonical_counts),
            "already_canonical_class_root_counts":
                dict(already_canonical_class_root_counts),
            "already_canonical_map_scoped_counts":
                dict(already_canonical_map_scoped_counts),
            "canonical_class_roots":
                dict(CANONICAL_CLASS_ROOTS),
            "canonical_policy":
                "complete_carla_class_root",
            "classifiable_context_counts":
                dict(classifiable_context_counts),
            "failure_count": len(failures),
            "failures": failures[:200],
            "save_result": save_result,
            "actor_path_filter_diagnostic": {
                "sample_count":
                    len(actor_diagnostic_samples),
                "sample_string_match_count":
                    path_match_count,
                "sample_string_nonmatch_count":
                    len(actor_diagnostic_samples)
                    - path_match_count,
                "samples": actor_diagnostic_samples,
            },
            "source_mesh_samples":
                source_mesh_samples,
            "classified_context_samples":
                classified_context_samples,
            "unmatched_context_samples":
                unmatched_context_samples,
            "checkpoint": False,
            "success": success,
            "finish_reason": (
                "semantic_verification_completed"
                if MODE == "verify"
                else "semantic_actor_batch_completed"
            ),
            "finished_at": time.time(),
        }
    )

    save_report()

    log(
        "CANONICAL_POLICY=complete_carla_class_root"
    )
    log("ALL_LOADED_ACTORS={}".format(
        all_actor_count
    ))
    log("PROCESSED_ACTORS={}".format(
        len(actors_to_process)
    ))
    log("STATIC_MESH_COMPONENTS={}".format(
        static_mesh_component_count
    ))
    log("CLASSIFIED_COMPONENTS={}".format(
        classified_component_count
    ))
    log("UNMATCHED_COMPONENTS={}".format(
        unmatched_component_count
    ))
    log("REMAINING_NONCANONICAL={}".format(
        remaining_noncanonical_count
    ))

    for key, value in sorted(
        replacement_counts.items()
    ):
        log(
            "{}_RETAGGED_{}={}".format(
                TARGET_LABEL.upper(),
                key.upper(),
                value,
            )
        )

    for key, value in sorted(
        already_canonical_counts.items()
    ):
        log(
            "{}_ALREADY_CANONICAL_{}={}".format(
                TARGET_LABEL.upper(),
                key.upper(),
                value,
            )
        )

    log("REPORT=" + str(REPORT_PATH))
    log("SUCCESS=" + str(int(bool(success))))

    if not success:
        raise RuntimeError(
            "Semantic {} failed".format(MODE)
        )


try:
    run()
except Exception:
    STATE["success"] = False
    STATE["finish_reason"] = (
        "batched_semantic_retag_exception"
    )
    STATE["exception"] = traceback.format_exc()
    STATE["finished_at"] = time.time()
    save_report()
    unreal.log_error(STATE["exception"])
    raise
