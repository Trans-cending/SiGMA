# -*- coding: utf-8 -*-
try:
    from .blender_common import *
except ImportError:
    from Blender_xml_storage.blender_common import *

from mosaic_core.rf_materials import (
    load_rf_material_registry,
    material_rgba_table,
    rf_material_from_ue,
)

try:
    from .static_streetlight_filter import (
        is_dtt_streetlight_mesh,
        is_osm_streetlamp_marker,
        position_matches_osm_marker,
    )
except ImportError:
    from Blender_xml_storage.static_streetlight_filter import (
        is_dtt_streetlight_mesh,
        is_osm_streetlamp_marker,
        position_matches_osm_marker,
    )

ITU_MATERIAL_RGBA = material_rgba_table()
ITU_MATERIAL_WHITELIST = set(ITU_MATERIAL_RGBA)
_ITU_NAME_FIX = {
    alias: "itu_" + str(entry["sionna_type"])
    for alias, material_id in
    load_rf_material_registry().get("sionna_aliases", {}).items()
    for entry in [load_rf_material_registry()["materials"][material_id]]
}
STATIC_FBX_MATERIAL_RULES = [
    (str(rule["rf_material_id"]), tuple(str(word) for word in rule.get("keywords", [])))
    for rule in load_rf_material_registry().get("ue_material_rules", [])
]

def is_known_itu_material(name):
    return strip_blender_suffix(name) in ITU_MATERIAL_WHITELIST

def get_or_create_itu_material(name):
    if name not in ITU_MATERIAL_RGBA:
        raise ValueError(f"Unknown ITU material: {name}")

    # Remove wrongly suffixed unused versions if present.
    for mat in list(bpy.data.materials):
        if mat.name != name and strip_blender_suffix(mat.name) == name and mat.users == 0:
            try:
                bpy.data.materials.remove(mat, do_unlink=True)
            except Exception:
                pass

    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name=name)

    rgba = ITU_MATERIAL_RGBA[name]
    mat.diffuse_color = rgba
    mat.use_nodes = True

    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        if "Base Color" in bsdf.inputs:
            bsdf.inputs["Base Color"].default_value = rgba
        if "Alpha" in bsdf.inputs:
            bsdf.inputs["Alpha"].default_value = rgba[3]
        if "Roughness" in bsdf.inputs:
            bsdf.inputs["Roughness"].default_value = 0.35

    if rgba[3] < 1.0:
        mat.blend_method = "BLEND"
        mat.show_transparent_back = True

    return mat

def ensure_core_itu_materials():
    get_or_create_itu_material("itu_metal")
    get_or_create_itu_material("itu_glass")
    get_or_create_itu_material("itu_human")

def ensure_all_itu_materials():
    for name in sorted(ITU_MATERIAL_WHITELIST):
        get_or_create_itu_material(name)

def remove_unused_materials():
    for mat in list(bpy.data.materials):
        if mat.users == 0:
            try:
                bpy.data.materials.remove(mat, do_unlink=True)
            except Exception:
                pass

def canonicalize_imported_materials(objects, actor_class):
    """
    Dynamic models must have exact material names.

    vehicle/truck:
        itu_metal, itu_glass
    human:
        itu_human
    rsu:
        itu_metal
    """
    ensure_core_itu_materials()

    metal = get_or_create_itu_material("itu_metal")
    glass = get_or_create_itu_material("itu_glass")
    human = get_or_create_itu_material("itu_human")

    for obj in objects:
        if obj.type != "MESH":
            continue

        old_mats = list(obj.data.materials)

        if actor_class == "human":
            obj.data.materials.clear()
            obj.data.materials.append(human)
            for poly in obj.data.polygons:
                poly.material_index = 0
            continue

        if actor_class == "rsu":
            obj.data.materials.clear()
            obj.data.materials.append(metal)
            for poly in obj.data.polygons:
                poly.material_index = 0
            continue

        labels = []

        for poly in obj.data.polygons:
            idx = poly.material_index
            label = "metal"

            if 0 <= idx < len(old_mats):
                old_name = strip_blender_suffix(old_mats[idx].name).lower()
                if old_name == "itu_glass" or "glass" in old_name:
                    label = "glass"
                elif glass_like_name(old_name):
                    label = "glass"

            if label == "metal" and glass_like_name(obj.name):
                label = "glass"

            labels.append(label)

        obj.data.materials.clear()
        obj.data.materials.append(metal)
        obj.data.materials.append(glass)

        for poly, label in zip(obj.data.polygons, labels):
            poly.material_index = 1 if label == "glass" else 0

def _fix_itu_material_names(xml_path):
    """Restore 'itu_' prefix stripped by the Mitsuba-Blender exporter.

    The exporter writes:
        <bsdf type="itu-radio-material" id="mat-medium_dry_ground_28">
            <string name="type" value="medium_dry_ground_28"/>
    Sionna RT reads the <string name="type" value="..."> child and needs
    valid ITU type names (e.g. "medium_dry_ground", "metal", "concrete").
    """
    try:
        with open(xml_path, "r", encoding="utf-8") as fh:
            text = fh.read()
    except Exception:
        return

    for stripped, full in _ITU_NAME_FIX.items():
        # Fix <string name="type" value="..."> — the ITU material type that
        # Sionna RT reads inside the itu-radio-material bsdf plugin.
        text = text.replace(
            f'<string name="type" value="{stripped}"',
            f'<string name="type" value="{full}"'
        )
        # Fix bsdf id and all ref id references
        #   id="mat-medium_dry_ground_28" -> id="mat-itu_medium_dry_ground"
        text = text.replace(
            f'id="mat-{stripped}"', f'id="mat-{full}"'
        )
        # Also handle bare material IDs (rare but possible).
        text = text.replace(
            f'id="{stripped}"', f'id="{full}"'
        )

    try:
        with open(xml_path, "w", encoding="utf-8") as fh:
            fh.write(text)
    except Exception:
        pass

def warn_unknown_materials():
    if not XML_WARN_MATERIALS:
        return

    used = set()

    for obj in bpy.data.objects:
        if obj.type != "MESH":
            continue
        for slot in obj.material_slots:
            if slot.material:
                used.add(slot.material.name)

    unknown = sorted(m for m in used if not is_known_itu_material(m))

    if unknown:
        print("  WARNING: unknown/non-ITU materials used in scene:", flush=True)
        for m in unknown[:50]:
            print(f"    - {m}", flush=True)
        if len(unknown) > 50:
            print(f"    ... {len(unknown) - 50} more", flush=True)

def material_to_itu_name(mat_name, actor_class=None):
    """Resolve dynamic mesh material through the shared strict registry."""
    return rf_material_from_ue(mat_name or "", actor_class=actor_class)

def mesh_material_groups(mesh, actor_class=None):
    """
    Return dict itu_material -> list of polygon indices.
    """
    groups = {}

    for poly in mesh.polygons:
        mat_name = None
        try:
            if mesh.materials and poly.material_index < len(mesh.materials):
                mat = mesh.materials[poly.material_index]
                mat_name = mat.name if mat else None
        except Exception:
            mat_name = None

        itu = material_to_itu_name(mat_name, actor_class=actor_class)
        groups.setdefault(itu, []).append(poly.index)

    return groups

def _static_fbx_text(*parts):
    return normalize_text(" ".join(str(p or "") for p in parts))

def _should_drop_static_fbx_object(obj):
    if not XML_STATIC_DROP_SKY:
        return False
    text = _static_fbx_text(
        obj.name,
        getattr(obj.data, "name", ""),
        " ".join(m.name for m in getattr(obj.data, "materials", []) if m),
    )
    return "sky" in text or "bp_sky" in text or text in {"planet"}


def apply_static_dtt_streetlight_mode(objects):
    """Filter DTT fixtures while preserving OSM-authoritative lamp meshes."""
    marker_positions = tuple(
        tuple(float(value) for value in obj.matrix_world.translation)
        for obj in objects
        if is_osm_streetlamp_marker(obj.name)
    )
    candidates = [
        obj
        for obj in objects
        if obj.type == "MESH"
        and is_dtt_streetlight_mesh(
            obj.name,
            getattr(obj.data, "name", ""),
        )
    ]
    protected = []
    filtered = []
    for obj in candidates:
        position = tuple(float(value) for value in obj.matrix_world.translation)
        if position_matches_osm_marker(
            position,
            marker_positions,
            XML_STATIC_OSM_STREETLIGHT_MATCH_TOLERANCE_M,
        ):
            protected.append(obj)
        elif XML_STATIC_DTT_STREETLIGHT_MODE != "full":
            filtered.append(obj)

    for obj in filtered:
        bpy.data.objects.remove(obj, do_unlink=True)

    report = {
        "dtt_streetlight_mode": XML_STATIC_DTT_STREETLIGHT_MODE,
        "dtt_streetlight_mesh_filter_applied": (
            XML_STATIC_DTT_STREETLIGHT_MODE != "full"
        ),
        "dtt_streetlight_candidate_meshes": len(candidates),
        "dtt_streetlight_filtered_meshes": len(filtered),
        "dtt_streetlight_osm_protected_meshes": len(protected),
        "dtt_streetlight_osm_markers": len(marker_positions),
    }
    print(
        "[STATIC_STREETLIGHT_FILTER] "
        + " ".join(f"{key}={value}" for key, value in report.items()),
        flush=True,
    )
    return report


def static_fbx_itu_material(material_name, obj_name="", mesh_name=""):
    base = strip_blender_suffix(material_name or "")
    if is_known_itu_material(base):
        return base
    return rf_material_from_ue(
        material_name or "", object_name=obj_name, mesh_name=mesh_name
    )

def canonicalize_static_fbx_materials(objects):
    ensure_all_itu_materials()
    material_slots = {name: get_or_create_itu_material(name) for name in ITU_MATERIAL_WHITELIST}
    material_counts = {}
    removed = 0

    for obj in list(objects):
        if obj.type != "MESH":
            continue
        if _should_drop_static_fbx_object(obj):
            bpy.data.objects.remove(obj, do_unlink=True)
            removed += 1
            continue

        old_mats = list(obj.data.materials)
        labels = []
        for poly in obj.data.polygons:
            mat_name = ""
            if 0 <= poly.material_index < len(old_mats) and old_mats[poly.material_index]:
                mat_name = old_mats[poly.material_index].name
            label = static_fbx_itu_material(mat_name, obj.name, obj.data.name)
            labels.append(label)
            material_counts[label] = material_counts.get(label, 0) + 1

        ordered = []
        for label in labels:
            if label not in ordered:
                ordered.append(label)
        if not ordered:
            raise ValueError(f"Static mesh {obj.name!r} has no polygons/material assignment")

        slot_index = {label: i for i, label in enumerate(ordered)}
        obj.data.materials.clear()
        for label in ordered:
            obj.data.materials.append(material_slots[label])
        for poly, label in zip(obj.data.polygons, labels):
            poly.material_index = slot_index[label]

    remove_unused_materials()
    print(f"[STATIC_FBX] removed sky/non-RT objects: {removed}", flush=True)
    print("[STATIC_FBX] polygon material counts:", flush=True)
    for name, count in sorted(material_counts.items(), key=lambda kv: (-kv[1], kv[0])):
        print(f"  {name}: {count}", flush=True)


__all__=[k for k in globals() if not k.startswith("__")]
