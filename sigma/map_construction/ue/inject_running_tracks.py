# -*- coding: utf-8 -*-
"""Inject independent meshes for OSM tartan tracks and sports surfaces."""
from __future__ import print_function

import json
import os
import time
import traceback
from pathlib import Path

import unreal


MAP_NAME = os.environ["MOSAIC_MAP_NAME"]
TARGET_LEVEL = os.environ["MOSAIC_RUNNING_TRACK_TARGET_LEVEL"]
PLAN_PATH = Path(os.environ["MOSAIC_RUNNING_TRACK_PLAN"])
REPORT_PATH = Path(os.environ["MOSAIC_RUNNING_TRACK_REPORT"])
COLOR = tuple(
    max(0, min(255, int(value.strip())))
    for value in os.environ.get("MOSAIC_TARTAN_TRACK_COLOR", "72,14,14").split(",")
)
COURT_COLOR = tuple(
    max(0, min(255, int(value.strip())))
    for value in os.environ.get("MOSAIC_BASKETBALL_COURT_COLOR", "25,60,50").split(",")
)
GRASS_COLOR = tuple(
    max(0, min(255, int(value.strip())))
    for value in os.environ.get("MOSAIC_GRASS_PITCH_COLOR", "45,100,38").split(",")
)
if len(COLOR) != 3:
    raise ValueError("MOSAIC_TARTAN_TRACK_COLOR must contain R,G,B")
if len(COURT_COLOR) != 3:
    raise ValueError("MOSAIC_BASKETBALL_COURT_COLOR must contain R,G,B")
if len(GRASS_COLOR) != 3:
    raise ValueError("MOSAIC_GRASS_PITCH_COLOR must contain R,G,B")

ACTOR_PREFIX = "MOSAIC_TartanTrack_"
COURT_ACTOR_PREFIX = "MOSAIC_BasketballCourt_"
GRASS_ACTOR_PREFIX = "MOSAIC_GrassPitch_"
CUBE_ASSET = "/Engine/BasicShapes/Cube.Cube"
MATERIAL_PACKAGE = "/Game/Carla/Static/Terrain/{}/M_MOSAIC_TartanTrack".format(MAP_NAME)
COURT_MATERIAL_PACKAGE = "/Game/Carla/Static/Terrain/{}/M_MOSAIC_BasketballCourt".format(MAP_NAME)
GRASS_MATERIAL_PACKAGE = "/Game/Carla/Static/Terrain/{}/M_MOSAIC_GrassPitch".format(MAP_NAME)
THICKNESS_M = 0.025
OVERLAP_M = 0.12
Z_OFFSET_CM = 4.0

STATE = {
    "runner": "osm-tartan-track-injection",
    "map_name": MAP_NAME,
    "target_level": TARGET_LEVEL,
    "plan": str(PLAN_PATH),
    "material_package": MATERIAL_PACKAGE,
    "court_material_package": COURT_MATERIAL_PACKAGE,
    "grass_material_package": GRASS_MATERIAL_PACKAGE,
    "color_rgb": list(COLOR),
    "court_color_rgb": list(COURT_COLOR),
    "grass_color_rgb": list(GRASS_COLOR),
    "success": False,
    "started_at": time.time(),
}


def package_path(value):
    return str(value).split(".", 1)[0]


def current_world_package():
    world = unreal.EditorLevelLibrary.get_editor_world()
    if world is None:
        raise RuntimeError("Editor world unavailable")
    return package_path(world.get_path_name())


def safe_label(actor):
    try:
        return str(actor.get_actor_label())
    except Exception:
        return ""


def save_report():
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    STATE["updated_at"] = time.time()
    REPORT_PATH.write_text(json.dumps(STATE, indent=2, ensure_ascii=False), encoding="utf-8")


def save_asset(asset):
    try:
        return bool(unreal.EditorAssetLibrary.save_loaded_asset(asset, False))
    except TypeError:
        return bool(unreal.EditorAssetLibrary.save_loaded_asset(asset, only_if_is_dirty=False))


def srgb_channel_to_linear(channel):
    value = max(0.0, min(1.0, float(channel) / 255.0))
    if value <= 0.04045:
        return value / 12.92
    return ((value + 0.055) / 1.055) ** 2.4


def create_material(package, rgb):
    if unreal.EditorAssetLibrary.does_asset_exist(package):
        material = unreal.EditorAssetLibrary.load_asset(package)
        if material is not None:
            try:
                unreal.MaterialEditingLibrary.delete_all_material_expressions(material)
            except Exception:
                if not unreal.EditorAssetLibrary.delete_asset(package):
                    raise RuntimeError("Failed to reset {}".format(package))
                material = None
    else:
        material = None
    if material is None:
        directory, name = package.rsplit("/", 1)
        unreal.EditorAssetLibrary.make_directory(directory)
        material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
            name, directory, unreal.Material, unreal.MaterialFactoryNew()
        )
    if material is None:
        raise RuntimeError("Failed to create {}".format(package))
    color = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionConstant3Vector, -320, 0
    )
    color.set_editor_property(
        "constant",
        unreal.LinearColor(
            srgb_channel_to_linear(rgb[0]),
            srgb_channel_to_linear(rgb[1]),
            srgb_channel_to_linear(rgb[2]),
            1.0,
        ),
    )
    unreal.MaterialEditingLibrary.connect_material_property(
        color, "", unreal.MaterialProperty.MP_BASE_COLOR
    )
    roughness = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionConstant, -320, 120
    )
    roughness.set_editor_property("r", 1.0)
    unreal.MaterialEditingLibrary.connect_material_property(
        roughness, "", unreal.MaterialProperty.MP_ROUGHNESS
    )
    specular = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionConstant, -320, 240
    )
    specular.set_editor_property("r", 0.0)
    unreal.MaterialEditingLibrary.connect_material_property(
        specular, "", unreal.MaterialProperty.MP_SPECULAR
    )
    metallic = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionConstant, -320, 360
    )
    metallic.set_editor_property("r", 0.0)
    unreal.MaterialEditingLibrary.connect_material_property(
        metallic, "", unreal.MaterialProperty.MP_METALLIC
    )
    unreal.MaterialEditingLibrary.recompile_material(material)
    if not save_asset(material):
        raise RuntimeError("Failed to save {}".format(package))
    return material


def destroy_existing():
    count = 0
    for actor in list(unreal.EditorLevelLibrary.get_all_level_actors()):
        if safe_label(actor).startswith(
            (ACTOR_PREFIX, COURT_ACTOR_PREFIX, GRASS_ACTOR_PREFIX)
        ):
            if not unreal.EditorLevelLibrary.destroy_actor(actor):
                raise RuntimeError("Failed to delete {}".format(safe_label(actor)))
            count += 1
    return count


def disable_collision(component):
    try:
        component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    except Exception:
        pass
    try:
        component.set_editor_property("generate_overlap_events", False)
    except Exception:
        pass


def save_level():
    try:
        if unreal.EditorLevelLibrary.save_current_level():
            return True
    except Exception:
        pass
    return bool(unreal.EditorLevelLibrary.save_all_dirty_levels())


def run():
    loaded = current_world_package()
    if loaded.lower() != TARGET_LEVEL.lower():
        raise RuntimeError("Loaded level mismatch: expected {}, got {}".format(TARGET_LEVEL, loaded))
    plan = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
    segments = list(plan.get("segments", []))
    courts = list(plan.get("basketball_courts", []))
    grass_surfaces = list(plan.get("grass_surfaces", []))
    mesh = unreal.EditorAssetLibrary.load_asset(CUBE_ASSET) or unreal.load_asset(CUBE_ASSET)
    if mesh is None:
        raise RuntimeError("Unable to load {}".format(CUBE_ASSET))
    STATE["removed_existing_actor_count"] = destroy_existing()
    material = create_material(MATERIAL_PACKAGE, COLOR)
    court_material = create_material(COURT_MATERIAL_PACKAGE, COURT_COLOR)
    grass_material = create_material(GRASS_MATERIAL_PACKAGE, GRASS_COLOR)
    grass_labels = []
    for index, surface in enumerate(grass_surfaces):
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor,
            unreal.Vector(
                float(surface["center_x_m"]) * 100.0,
                -float(surface["center_y_m"]) * 100.0,
                float(surface.get("center_z_m", 0.0)) * 100.0 + 2.0 + THICKNESS_M * 50.0,
            ),
            unreal.Rotator(pitch=0.0, yaw=-float(surface["heading_deg"]), roll=0.0),
        )
        if actor is None:
            raise RuntimeError("Failed to spawn grass pitch")
        label = "{}W{}_N{:03d}".format(
            GRASS_ACTOR_PREFIX, surface.get("osm_way_id", "0"), index
        )
        actor.set_actor_label(label)
        components = actor.get_components_by_class(unreal.StaticMeshComponent)
        if not components:
            raise RuntimeError("Grass-pitch actor has no StaticMeshComponent")
        component = components[0]
        try:
            component.set_static_mesh(mesh)
        except Exception:
            component.set_editor_property("static_mesh", mesh)
        component.set_material(0, grass_material)
        disable_collision(component)
        actor.set_actor_scale3d(unreal.Vector(
            max(1.0, float(surface["length_m"])),
            max(1.0, float(surface["width_m"])),
            THICKNESS_M,
        ))
        grass_labels.append(label)
    labels = []
    for index, segment in enumerate(segments):
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor,
            unreal.Vector(
                float(segment["center_x_m"]) * 100.0,
                -float(segment["center_y_m"]) * 100.0,
                float(segment.get("center_z_m", 0.0)) * 100.0 + Z_OFFSET_CM + THICKNESS_M * 50.0,
            ),
            unreal.Rotator(pitch=0.0, yaw=-float(segment["heading_deg"]), roll=0.0),
        )
        if actor is None:
            raise RuntimeError("Failed to spawn tartan track segment")
        label = "{}W{}_N{:05d}".format(ACTOR_PREFIX, segment.get("osm_way_id", "0"), index)
        actor.set_actor_label(label)
        components = actor.get_components_by_class(unreal.StaticMeshComponent)
        if not components:
            raise RuntimeError("Track actor has no StaticMeshComponent")
        component = components[0]
        try:
            component.set_static_mesh(mesh)
        except Exception:
            component.set_editor_property("static_mesh", mesh)
        component.set_material(0, material)
        disable_collision(component)
        actor.set_actor_scale3d(unreal.Vector(
            max(0.25, float(segment["length_m"]) + OVERLAP_M),
            max(1.0, float(segment["width_m"])),
            THICKNESS_M,
        ))
        labels.append(label)
    court_labels = []
    for index, court in enumerate(courts):
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor,
            unreal.Vector(
                float(court["center_x_m"]) * 100.0,
                -float(court["center_y_m"]) * 100.0,
                float(court.get("center_z_m", 0.0)) * 100.0 + Z_OFFSET_CM + THICKNESS_M * 50.0,
            ),
            unreal.Rotator(pitch=0.0, yaw=-float(court["heading_deg"]), roll=0.0),
        )
        if actor is None:
            raise RuntimeError("Failed to spawn basketball court")
        label = "{}W{}_N{:03d}".format(
            COURT_ACTOR_PREFIX, court.get("osm_way_id", "0"), index
        )
        actor.set_actor_label(label)
        components = actor.get_components_by_class(unreal.StaticMeshComponent)
        if not components:
            raise RuntimeError("Basketball-court actor has no StaticMeshComponent")
        component = components[0]
        try:
            component.set_static_mesh(mesh)
        except Exception:
            component.set_editor_property("static_mesh", mesh)
        component.set_material(0, court_material)
        disable_collision(component)
        actor.set_actor_scale3d(unreal.Vector(
            max(1.0, float(court["length_m"])),
            max(1.0, float(court["width_m"])),
            THICKNESS_M,
        ))
        court_labels.append(label)
    if (segments or courts or grass_surfaces) and not save_level():
        raise RuntimeError("Failed to save persistent level after sports-surface injection")
    STATE.update({
        "planned_segment_count": len(segments),
        "generated_segment_count": len(labels),
        "actor_label_samples": labels[:50],
        "planned_basketball_court_count": len(courts),
        "generated_basketball_court_count": len(court_labels),
        "basketball_court_actor_labels": court_labels[:50],
        "planned_grass_surface_count": len(grass_surfaces),
        "generated_grass_surface_count": len(grass_labels),
        "grass_surface_actor_labels": grass_labels[:50],
        "success": (
            len(labels) == len(segments)
            and len(court_labels) == len(courts)
            and len(grass_labels) == len(grass_surfaces)
        ),
        "finished_at": time.time(),
    })
    save_report()


try:
    run()
except Exception:
    STATE["exception"] = traceback.format_exc()
    STATE["finished_at"] = time.time()
    save_report()
    unreal.log_error(STATE["exception"])
    raise
