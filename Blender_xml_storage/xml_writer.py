"""Export frozen Stage III scenes and verify a registered Mitsuba operator.

Run through python -m sigma stage3 -c configs/town03.yaml. Validate
export plugins through their registered operators before exporting.
"""
import xml.etree.ElementTree as ET
import struct

try:
    from .blender_common import *
except ImportError:
    from Blender_xml_storage.blender_common import *

try:
    from .asset_manager import build_baked_clean_mesh_for_blueprint
    from .material_manager import *
    from .mesh_writer import *
except ImportError:
    from Blender_xml_storage.asset_manager import build_baked_clean_mesh_for_blueprint
    from Blender_xml_storage.material_manager import *
    from Blender_xml_storage.mesh_writer import *


try:
    from .static_crop_utils import (
        build_stage1_txrx_crop_spec,
        crop_metadata_path,
        read_crop_metadata,
        write_crop_metadata,
    )
except ImportError:
    from Blender_xml_storage.static_crop_utils import (
        build_stage1_txrx_crop_spec,
        crop_metadata_path,
        read_crop_metadata,
        write_crop_metadata,
    )

_MITSUBA_EXPORTER_READY = False

def _has_mitsuba_operator():
    try:
        op = getattr(bpy.ops.export_scene, "mitsuba", None)
        if op is None:
            return False
        op.get_rna_type()  # bpy.ops attribute lookup alone also succeeds for nonexistent operators.
        return True
    except Exception:
        return False

def ensure_mitsuba_exporter():
    global _MITSUBA_EXPORTER_READY
    if _MITSUBA_EXPORTER_READY and _has_mitsuba_operator():
        return

    print("[MITSUBA] enabling mitsuba-blender ...", flush=True)
    try:
        bpy.ops.preferences.addon_enable(module="mitsuba-blender")
    except Exception:
        pass

    if not _has_mitsuba_operator():
        raise RuntimeError(
            "Mitsuba exporter not found. Set BLENDER_USER_SCRIPTS to the addon directory."
        )

    _MITSUBA_EXPORTER_READY = True
    print("[MITSUBA] exporter ready", flush=True)


def _export_mitsuba_once(xml_path):
    """Export with a fresh converter to isolate each retry from previous failures."""
    import importlib
    from bpy_extras.io_utils import axis_conversion

    exporter = importlib.import_module("mitsuba-blender.io.exporter")
    converter = exporter.SceneConverter()
    converter.export_ctx.axis_mat = axis_conversion(
        to_forward="Y",
        to_up="Z",
    ).to_4x4()
    converter.export_ctx.export_ids = True
    converter.set_path(xml_path)
    window_manager = bpy.context.window_manager
    deps_graph = bpy.context.evaluated_depsgraph_get()
    window_manager.progress_begin(0, len(deps_graph.object_instances))
    try:
        converter.scene_to_dict(deps_graph, window_manager)
        converter.dict_to_xml()
    finally:
        window_manager.progress_end()

def _repair_mesh_for_mitsuba(obj, xml_path):
    """Preserve one invalid BlenderMesh object as explicit face-normal PLY."""
    from mathutils import Matrix

    source_mesh = getattr(obj, "data", None)
    if source_mesh is None:
        raise RuntimeError(f"mesh object has no data: {obj.name}")
    before = (
        len(source_mesh.vertices),
        len(source_mesh.edges),
        len(source_mesh.polygons),
    )
    matrix_determinant = obj.matrix_world.to_3x3().determinant()
    repaired_mesh = build_baked_clean_mesh_for_blueprint(obj, actor_class=None)
    if repaired_mesh is None:
        raise RuntimeError(f"mesh has no valid faces after repair: {obj.name}")

    material_groups = {}
    for polygon in repaired_mesh.polygons:
        material_index = int(polygon.material_index)
        if not (0 <= material_index < len(repaired_mesh.materials)):
            raise RuntimeError(
                f"mesh has an invalid material index after repair: "
                f"object={obj.name} polygon={polygon.index} index={material_index}"
            )
        material = repaired_mesh.materials[material_index]
        if material is None:
            raise RuntimeError(
                f"mesh has an empty material slot after repair: "
                f"object={obj.name} index={material_index}"
            )
        material_groups.setdefault(material_index, []).append(polygon.index)

    object_name = obj.name
    clean_object_name = bpy.path.clean_name(object_name)
    mesh_dir = Path(xml_path).resolve().parent / "meshes"
    shape_records = []
    for material_index, polygon_indices in sorted(material_groups.items()):
        material = repaired_mesh.materials[material_index]
        if len(material_groups) == 1:
            part_name = clean_object_name
        else:
            part_name = (
                f"{clean_object_name}-{bpy.path.clean_name(material.name)}"
                f"-{material_index:03d}"
            )
        ply_path = mesh_dir / f"{part_name}.ply"
        if not write_binary_little_endian_ply(
            repaired_mesh, polygon_indices, ply_path
        ):
            raise RuntimeError(
                f"failed to preserve repaired mesh as PLY: "
                f"object={object_name} material={material.name}"
            )
        shape_records.append({
            "id": f"mesh-{part_name}",
            "filename": os.path.relpath(
                str(ply_path), start=str(Path(xml_path).resolve().parent)
            ),
            "material_ref": f"mat-{material.name}",
            "face_count": len(polygon_indices),
            "source_object": object_name,
        })

    after = (
        len(repaired_mesh.vertices),
        len(repaired_mesh.edges),
        len(repaired_mesh.polygons),
    )
    collections = list(obj.users_collection)
    custom_properties = {
        key: obj[key] for key in obj.keys() if key != "_RNA_UI"
    }
    bpy.data.objects.remove(obj, do_unlink=True)
    replacement = bpy.data.objects.new(object_name, repaired_mesh)
    replacement.matrix_world = Matrix.Identity(4)
    replacement.hide_render = True
    for key, value in custom_properties.items():
        replacement[key] = value
    for collection in collections or [bpy.context.scene.collection]:
        collection.objects.link(replacement)
    repaired_mesh.update_tag()
    replacement.update_tag(refresh={"DATA"})
    return {
        "before": before,
        "after": after,
        "matrix_determinant": matrix_determinant,
        "replacement": replacement,
        "shape_records": shape_records,
    }


def _append_repaired_mesh_shapes(xml_path, shape_records):
    """Append PLY-backed shapes omitted from the BlenderMesh conversion."""
    if not shape_records:
        return
    tree = ET.parse(xml_path)
    root = tree.getroot()
    existing_ids = {node.get("id") for node in root if node.get("id")}
    material_ids = {node.get("id") for node in root.findall("bsdf")}
    for record in shape_records:
        if record["id"] in existing_ids:
            raise RuntimeError(f"duplicate repaired shape id: {record['id']}")
        if record["material_ref"] not in material_ids:
            raise RuntimeError(
                f"repaired shape material is absent from exported scene: "
                f"shape={record['id']} material={record['material_ref']}"
            )
        shape = ET.SubElement(
            root, "shape", {"type": "ply", "id": record["id"], "name": record["id"]}
        )
        ET.SubElement(
            shape, "string", {"name": "filename", "value": record["filename"]}
        )
        ET.SubElement(
            shape, "boolean", {"name": "face_normals", "value": "true"}
        )
        ET.SubElement(
            shape, "ref", {"id": record["material_ref"], "name": "bsdf"}
        )
        existing_ids.add(record["id"] )
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    print(
        f"  [REPAIR_BAD_MESH] appended face-normal PLY shapes={len(shape_records)}",
        flush=True,
    )

def export_mitsuba_xml(xml_path):
    """Export every valid object, preserving invalid-normal meshes as PLY."""
    ensure_mitsuba_exporter()

    if not _has_mitsuba_operator():
        raise RuntimeError("bpy.ops.export_scene.mitsuba is still missing before export.")

    import logging as _logging
    _root_logger = _logging.getLogger()
    _prev_level = _root_logger.level
    _root_logger.setLevel(_logging.ERROR)
    repaired_objects = []
    repaired_shape_records = []

    try:
        try:
            with quiet_blender_output(XML_QUIET_MITSUBA and not XML_VERBOSE):
                _export_mitsuba_once(xml_path)
        except Exception as mitsuba_err:
            m = re.search(r'Blender mesh "([^"]+)"', str(mitsuba_err))
            if not m:
                raise

            def _bad_object_by_name(target):
                target_lower = str(target).strip().lower()
                for candidate in bpy.data.objects:
                    if (
                        candidate.type == "MESH"
                        and candidate.name.strip().lower() == target_lower
                    ):
                        return candidate
                duplicate = re.compile(re.escape(target_lower) + r"\.\d{3}\Z")
                for candidate in bpy.data.objects:
                    if (
                        candidate.type == "MESH"
                        and duplicate.fullmatch(candidate.name.strip().lower())
                    ):
                        return candidate
                return None

            repaired_names = set()
            while len(repaired_names) < XML_MAX_SKIP_BAD_MESHES:
                bad_obj_name = re.sub(r"-itu_\w+$", "", m.group(1))
                bad_obj = _bad_object_by_name(bad_obj_name)
                if bad_obj is None:
                    raise RuntimeError(
                        f"cannot resolve invalid Mitsuba mesh in Blender: {bad_obj_name!r}"
                    ) from mitsuba_err
                object_key = bad_obj.name
                if object_key in repaired_names:
                    raise RuntimeError(
                        f"Mitsuba still sees temporarily hidden repaired mesh: {object_key}"
                    ) from mitsuba_err
                bad_data_name = getattr(bad_obj.data, "name", None)
                report = _repair_mesh_for_mitsuba(bad_obj, xml_path)
                repaired_names.add(object_key)
                repaired_objects.append(report["replacement"])
                repaired_shape_records.extend(report["shape_records"])
                print(
                    f"  [REPAIR_BAD_MESH] obj={object_key} data={bad_data_name} "
                    f"matrix_det={report['matrix_determinant']:.3g} "
                    f"geometry={report['before']}->{report['after']} "
                    f"preserved_shapes={len(report['shape_records'])} "
                    f"({len(repaired_names)}/{XML_MAX_SKIP_BAD_MESHES})",
                    flush=True,
                )
                bpy.context.view_layer.update()
                bpy.context.evaluated_depsgraph_get().update()
                try:
                    with quiet_blender_output(XML_QUIET_MITSUBA and not XML_VERBOSE):
                        _export_mitsuba_once(xml_path)
                    break
                except Exception as next_err:
                    next_match = re.search(
                        r'Blender mesh "([^"]+)"', str(next_err)
                    )
                    if not next_match:
                        raise
                    m = next_match
            else:
                last_offender = re.sub(r"-itu_\w+$", "", m.group(1))
                raise RuntimeError(
                    f"export kept failing after preserving "
                    f"{XML_MAX_SKIP_BAD_MESHES} bad mesh(es); "
                    f"last offender: {last_offender}"
                )

        _append_repaired_mesh_shapes(xml_path, repaired_shape_records)
    finally:
        for repaired_object in repaired_objects:
            repaired_object.hide_render = False
        if repaired_objects:
            bpy.context.view_layer.update()
        _root_logger.setLevel(_prev_level)

    # Mitsuba-Blender exporter strips "itu_" prefix from material names.
    # Sionna RT expects full "itu_*" names. Post-process the XML.
    _fix_itu_material_names(xml_path)
    _ensure_film_rfilter(xml_path)

def _ensure_film_rfilter(xml_path):
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        added = 0
        for film in root.iter("film"):
            if not any(child.tag == "rfilter" for child in film):
                rfilter = ET.Element("rfilter")
                rfilter.set("type", "box")
                film.insert(0, rfilter)
                added += 1
        if added:
            tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    except Exception:
        # Non-fatal post-process; the film warning is cosmetic anyway.
        pass

def xml_escape(s):
    import html
    return html.escape(str(s), quote=True)

def write_manual_dynamic_xml(xml_path, shape_records):
    """
    Write a Mitsuba/Sionna-compatible XML using shared mesh files.
    """
    xml_path = Path(xml_path)
    xml_path.parent.mkdir(parents=True, exist_ok=True)

    mats = sorted({rec["material"] for rec in shape_records} | {"itu_metal", "itu_glass", "itu_human"})

    lines = []
    lines.append('<?xml version="1.0" encoding="utf-8"?>')
    lines.append('<scene version="3.5.0">')

    for mat in mats:
        mid = f"mat-{mat}"
        lines.append(f'    <bsdf type="itu-radio-material" id="{xml_escape(mid)}">')
        lines.append(f'        <string name="type" value="{xml_escape(mat)}"/>')
        lines.append('    </bsdf>')

    for rec in shape_records:
        sid = rec["id"]
        name = rec["name"]
        filename = mesh_filename_for_xml(rec["filename"], xml_path)
        mid = f"mat-{rec['material']}"
        lines.append(f'    <shape type="ply" id="{xml_escape(sid)}" name="{xml_escape(name)}">')
        lines.append(f'        <string name="filename" value="{xml_escape(filename)}"/>')
        lines.append('        <transform name="to_world">')
        lines.append(f'            <matrix value="{rec["matrix"]}"/>')
        lines.append('        </transform>')
        lines.append(f'        <ref id="{xml_escape(mid)}"/>')
        lines.append('    </shape>')

    lines.append('</scene>')

    tmp = xml_path.with_suffix(xml_path.suffix + f".tmp.{os.getpid()}")
    tmp.write_text("\n".join(lines) + "\n", encoding="utf-8")
    os.replace(tmp, xml_path)

def import_static_fbx_scene(fbx_path, blend_output=""):
    fbx = Path(fbx_path).expanduser().resolve()
    if not fbx.is_file():
        raise FileNotFoundError(f"XML_STATIC_FBX_PATH does not exist: {fbx}")

    print(f"[STATIC_FBX] importing: {fbx}", flush=True)
    before = set(bpy.data.objects)
    with quiet_blender_output(XML_QUIET_FBX and not XML_VERBOSE):
        bpy.ops.import_scene.fbx(filepath=str(fbx))
    imported = list(set(bpy.data.objects) - before)
    meshes = [o for o in imported if o.type == "MESH"]
    print(f"[STATIC_FBX] imported objects={len(imported)} meshes={len(meshes)} materials={len(bpy.data.materials)}", flush=True)

    canonicalize_static_fbx_materials(meshes)

    if XML_STATIC_SAVE_BLEND and blend_output:
        blend_path = Path(blend_output).expanduser().resolve()
        blend_path.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))
        print(f"[STATIC_FBX] saved blend: {blend_path}", flush=True)

    return imported

def detect_static_town_key(blend_name):
    if XML_TOWN_KEY:
        return XML_TOWN_KEY
    m = re.search(r"Town\d{2}", blend_name or "", re.IGNORECASE)
    if m:
        return m.group(0)
    stem = Path(blend_name).stem if blend_name else ""
    if stem.endswith("Fixed"):
        stem = stem[:-5]
    if XML_EXPORT_MODE == "static" and stem:
        return stem
    print(f"ERROR: cannot detect TownXX in blend filename '{blend_name}'. Set XML_TOWN_KEY for non-Town maps.")
    sys.exit(1)

def _town_label_for_match(name):
    stem = Path(str(name or "")).name
    if re.fullmatch(r"\d{6}", stem):
        scoped_map = os.environ.get("DATASET_MAP_NAME", "").strip()
        if scoped_map:
            stem = scoped_map
    stem = stem.replace("_Opt", "")
    if stem.lower().startswith("town10"):
        return "Town10"
    m = re.match(r"(?i)^(Town\d{2})", stem)
    if m:
        return m.group(1).title().replace("Town", "Town")
    return stem

def _is_trajectory_dir_name(name):
    return re.fullmatch(r"\d{6}", str(name or "")) is not None

def _xml_shape_filename(shape):
    for child in list(shape):
        if child.tag == "string" and child.get("name") == "filename":
            return child.get("value") or ""
    return ""


def _xml_shape_material_text(shape):
    parts = []
    for child in list(shape):
        if child.tag == "ref":
            parts.append(child.get("id") or "")
        elif child.tag == "bsdf":
            parts.append(child.get("id") or "")
            for grand in list(child):
                if grand.tag == "string":
                    parts.append(grand.get("value") or "")
    return " ".join(parts).lower()


def _xml_shape_identity_text(shape):
    return " ".join([
        shape.get("id") or "",
        shape.get("name") or "",
        _xml_shape_filename(shape),
        _xml_shape_material_text(shape),
    ]).lower()


def _resolve_xml_mesh_path(xml_path, filename):
    if not filename:
        return None
    mesh_path = Path(filename)
    if not mesh_path.is_absolute():
        mesh_path = Path(xml_path).parent / mesh_path
    return mesh_path


def _read_ply_xyz_bbox(ply_path):
    """Return ([min xyz], [max xyz]) for simple ASCII/binary PLY vertex data."""
    try:
        with open(ply_path, "rb") as fh:
            header = []
            while True:
                line = fh.readline()
                if not line:
                    return None
                header.append(line)
                if line.strip() == b"end_header":
                    data_start = fh.tell()
                    break
            text = [line.decode("ascii", "ignore").strip() for line in header]
            fmt = ""
            vertex_count = 0
            vertex_props = []
            in_vertex = False
            for line in text:
                parts = line.split()
                if not parts:
                    continue
                if parts[0] == "format" and len(parts) >= 2:
                    fmt = parts[1]
                elif parts[0] == "element":
                    in_vertex = len(parts) >= 3 and parts[1] == "vertex"
                    if in_vertex:
                        vertex_count = int(parts[2])
                elif in_vertex and parts[0] == "property" and len(parts) >= 3 and parts[1] != "list":
                    vertex_props.append((parts[1], parts[2]))
            if vertex_count <= 0 or len(vertex_props) < 3:
                return None
            names = [name for _, name in vertex_props]
            types = [tp for tp, _ in vertex_props]
            if names[:3] != ["x", "y", "z"] or any(tp not in {"float", "float32"} for tp in types[:3]):
                return None
            if fmt == "ascii":
                mins = [float("inf"), float("inf"), float("inf")]
                maxs = [float("-inf"), float("-inf"), float("-inf")]
                for _ in range(vertex_count):
                    vals = fh.readline().decode("ascii", "ignore").split()
                    if len(vals) < 3:
                        continue
                    xyz = [float(vals[0]), float(vals[1]), float(vals[2])]
                    for i, val in enumerate(xyz):
                        mins[i] = min(mins[i], val)
                        maxs[i] = max(maxs[i], val)
                return mins, maxs
            if fmt not in {"binary_little_endian", "binary_big_endian"}:
                return None
            sizes = {
                "char": 1, "uchar": 1, "int8": 1, "uint8": 1,
                "short": 2, "ushort": 2, "int16": 2, "uint16": 2,
                "int": 4, "uint": 4, "int32": 4, "uint32": 4,
                "float": 4, "float32": 4, "double": 8, "float64": 8,
            }
            if any(tp not in sizes for tp, _ in vertex_props):
                return None
            row_size = sum(sizes[tp] for tp, _ in vertex_props)
            if row_size <= 0:
                return None
            endian = "<" if fmt == "binary_little_endian" else ">"
            fh.seek(data_start)
            mins = [float("inf"), float("inf"), float("inf")]
            maxs = [float("-inf"), float("-inf"), float("-inf")]
            for _ in range(vertex_count):
                row = fh.read(row_size)
                if len(row) < 12:
                    break
                xyz = struct.unpack_from(endian + "fff", row, 0)
                for i, val in enumerate(xyz):
                    mins[i] = min(mins[i], val)
                    maxs[i] = max(maxs[i], val)
            if any(not math.isfinite(v) for v in mins + maxs):
                return None
            return mins, maxs
    except Exception:
        return None


def _is_static_xml_ghost_blocker(shape, xml_path):
    if not XML_STATIC_DROP_GHOST_BLOCKERS:
        return False, ""
    text = _xml_shape_identity_text(shape)
    filename = _xml_shape_filename(shape)
    base = Path(filename).stem.lower()

    # Static UE map exports contain static geometry only. A leaked
    # generic component can retain centimeter-scale vertices and cover the
    # complete RF scene (Town05 exposed a 627 x 239 x 210 m vehicle mesh).
    shape_id = str(shape.get("id") or shape.get("name") or "").lower()
    if (re.fullmatch(r"mesh-skeletalmeshcomponent\d*", shape_id)
            and re.fullmatch(r"skeletalmeshcomponent\d*", base)):
        return True, "leaked-static-skeletal-component"

    # DTT/UE foliage impostors are thin visual sheets and have repeatedly
    # created false RF blockers while being absent from the current FBX view.
    if "treespawnposition" in text:
        return True, "treespawnposition-impostor"
    if any(k in text for k in ("billboard", "impostor", "foliage_proxy", "foliagecard")):
        return True, "foliage-impostor"

    material_text = _xml_shape_material_text(shape)
    generic_split_mesh = re.fullmatch(r"\d+_\d+", base or "") is not None
    if not generic_split_mesh or "wood" not in material_text:
        return False, ""

    bbox = _read_ply_xyz_bbox(_resolve_xml_mesh_path(xml_path, filename))
    if bbox is None:
        return False, ""
    mins, maxs = bbox
    dims = [maxs[i] - mins[i] for i in range(3)]
    horizontal_max = max(dims[0], dims[1])
    height = dims[2]
    if (
        XML_STATIC_GHOST_MIN_HEIGHT_M <= height <= XML_STATIC_GHOST_MAX_HEIGHT_M
        and horizontal_max <= XML_STATIC_GHOST_MAX_HORIZONTAL_M
    ):
        return True, f"small-wood-blocker bbox={','.join(f'{d:.2f}' for d in dims)}"
    return False, ""


def _filter_static_xml_ghost_blockers(xml_path):
    if not XML_STATIC_DROP_GHOST_BLOCKERS:
        return 0
    try:
        tree = ET.parse(xml_path)
    except Exception as exc:
        print(f"[STATIC_FILTER] skip unreadable XML: {xml_path} ({exc})", flush=True)
        return 0
    root = tree.getroot()
    removed = []
    for shape in list(root.findall("shape")):
        drop, reason = _is_static_xml_ghost_blocker(shape, xml_path)
        if not drop:
            continue
        removed.append({
            "id": shape.get("id") or shape.get("name") or "",
            "filename": _xml_shape_filename(shape),
            "reason": reason,
        })
        root.remove(shape)
    if not removed:
        return 0
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    preview = "; ".join(f"{r['id']} ({r['reason']})" for r in removed[:8])
    suffix = "" if len(removed) <= 8 else f"; ... +{len(removed) - 8} more"
    print(f"[STATIC_FILTER] removed ghost blockers={len(removed)}: {preview}{suffix}", flush=True)
    return len(removed)


def remove_static_scene_objects_for_dynamic_export():
    """
    Dynamic-only export mode:
        remove all objects from the opened Town blend before frame processing.
    Every per-frame actor created by this script is tagged with DYNAMIC_TAG, so the
    exported XML contains only CAV / vehicle / truck / pedestrian / RSU geometry.
    """
    removed = 0
    for obj in list(bpy.data.objects):
        try:
            bpy.data.objects.remove(obj, do_unlink=True)
            removed += 1
        except Exception:
            pass
    remove_unused_materials()
    print(f"[DYNAMIC_ONLY] removed static scene objects: {removed}", flush=True)

def export_static_town_once(dataset_root, town_key):
    """Export and, when requested, safely crop one static map XML.

    In channel workflows the crop is mandatory and is derived from Stage1
    world_state. Existing XMLs are only reused directly when cropping is
    disabled. With cropping enabled, an existing full backup is re-cropped so
    changed trajectories receive an updated crop window.
    """
    clean_dynamic_actors()
    streetlight_filter_report = apply_static_dtt_streetlight_mode(
        list(bpy.data.objects)
    )
    # The imported FBX meshes are canonicalized earlier, but the source .blend
    # can also contain pre-existing helper meshes such as a generic Cube with
    # Blender&apos;s default Material. They enter the same static RF scene.
    noncanonical_meshes = [
        obj for obj in bpy.data.objects
        if obj.type == "MESH"
        and len(obj.data.polygons) > 0
        and (
            not any(mat is not None for mat in obj.data.materials)
            or any(
                mat is not None and not is_known_itu_material(mat.name)
                for mat in obj.data.materials
            )
        )
    ]
    if noncanonical_meshes:
        print(
            f"[STATIC_FBX] canonicalizing {len(noncanonical_meshes)} "
            "pre-existing/non-FBX mesh(es)",
            flush=True,
        )
        canonicalize_static_fbx_materials(noncanonical_meshes)
    ensure_core_itu_materials()

    static_root = os.path.join(dataset_root, MITSUBA_STATIC_DIR)
    os.makedirs(static_root, exist_ok=True)
    scoped_map = os.environ.get("DATASET_MAP_NAME", "").strip()
    # In map-scoped runs the physical dataset belongs to the scenario
    # name even when its reusable geometry came from a differently named source
    # map/profile. Keep the source town for FBX/blend selection, but name the
    # static artifact after the scoped dataset map so Stage3 resolves it directly.
    output_key = scoped_map or town_key
    xml_path = os.path.join(static_root, f"{output_key}.xml")
    full_backup = re.sub(r"\.xml$", ".full.xml", xml_path)
    for stale_link in (xml_path, str(crop_metadata_path(xml_path))):
        if os.path.islink(stale_link):
            os.unlink(stale_link)
            print(f"[STATIC] removed stale symlink before export: {stale_link}", flush=True)

    crop_spec = None
    if XML_STATIC_IF_CROP:
        crop_spec = build_stage1_txrx_crop_spec(
            os.path.join(dataset_root, "stage1_multimodal"),
            town_key,
            XML_STATIC_BBOX_EXTEND,
            map_name=scoped_map,
        )
        if crop_spec is None:
            message = (
                "[STATIC_CROP] no TX/RX world_state points found under "
                f"{os.path.join(dataset_root, 'stage1_multimodal')} "
                f"for town={town_key!r} map={scoped_map or Path(dataset_root).name}"
            )
            if XML_STATIC_REQUIRE_CROP:
                raise RuntimeError(message + "; refusing uncropped static XML for Stage3")
            print(message + "; keeping full XML", flush=True)
        else:
            print(
                f"[STATIC_CROP] trajectories={crop_spec['trajectory_count']} "
                f"world_state_yamls={crop_spec['world_state_yaml_count']} "
                f"tx/rx points={crop_spec['tx_rx_points']} "
                f"box_min={[round(v, 1) for v in crop_spec['bbox_min']]} "
                f"box_max={[round(v, 1) for v in crop_spec['bbox_max']]}",
                flush=True,
            )

    existing = os.path.isfile(xml_path) and os.path.getsize(xml_path) > 0
    if XML_SKIP_EXISTING and existing:
        if not XML_STATIC_IF_CROP:
            _filter_static_xml_ghost_blockers(xml_path)
            print(f"[STATIC] skip existing: {xml_path}", flush=True)
            return xml_path
        previous_meta = read_crop_metadata(xml_path)
        same_streetlight_mode = (
            previous_meta.get("dtt_streetlight_mode")
            == XML_STATIC_DTT_STREETLIGHT_MODE
        )
        if (
            crop_spec is not None
            and same_streetlight_mode
            and os.path.isfile(full_backup)
            and os.path.getsize(full_backup) > 0
        ):
            print(f"[STATIC] re-cropping existing full backup: {full_backup}", flush=True)
            _crop_static_xml(
                xml_path,
                crop_spec,
                streetlight_filter_report,
                refresh_full_backup=False,
            )
            print(f"[STATIC] done: {xml_path}", flush=True)
            return xml_path
        if crop_spec is not None and not same_streetlight_mode:
            print(
                "[STATIC_STREETLIGHT_FILTER] existing full XML mode differs; "
                "re-exporting current scene",
                flush=True,
            )
        # Old in-place crops retain only partial geometry. Re-export once
        # so the new crop can widen as well as shrink when trajectory locations
        # change.
        print(
            f"[STATIC_CROP] existing XML lacks reusable full backup; "
            f"re-exporting complete scene before crop: {xml_path}",
            flush=True,
        )

    print(f"[STATIC] exporting {town_key} static scene -> {xml_path}", flush=True)
    export_mitsuba_xml(xml_path)
    _filter_static_xml_ghost_blockers(xml_path)
    if XML_STATIC_IF_CROP and crop_spec is not None:
        _crop_static_xml(
            xml_path,
            crop_spec,
            streetlight_filter_report,
            refresh_full_backup=True,
        )
    print(f"[STATIC] done: {xml_path}", flush=True)
    return xml_path


def _stage1_txrx_crop_box(dataset_root, town_key, extend_m=100.0):
    """Return the crop bbox from canonical compact Stage1 trajectories."""
    spec = build_stage1_txrx_crop_spec(
        os.path.join(dataset_root, "stage1_multimodal"),
        town_key,
        extend_m,
        map_name=os.environ.get("DATASET_MAP_NAME", "").strip(),
    )
    if spec is None:
        return None
    return [spec["bbox_min"], spec["bbox_max"]]


def _crop_static_xml(
    xml_path,
    crop_spec,
    streetlight_filter_report,
    *,
    refresh_full_backup=False,
):
    """Crop the static XML from a preserved complete backup and write metadata."""
    if not crop_spec:
        return False
    import mathutils
    import shutil

    crop_box = [crop_spec["bbox_min"], crop_spec["bbox_max"]]

    def _norm(s):
        # Mitsuba rewrites duplicate-name suffixes from '.' to '_'.
        return str(s).replace("_", ".").strip().lower()

    obj_by_name = {_norm(o.name): o for o in bpy.data.objects if o.type == "MESH"}

    def _world_bbox(obj):
        try:
            deps = bpy.context.evaluated_depsgraph_get()
            corners = [obj.matrix_world @ mathutils.Vector(c)
                       for c in obj.evaluated_get(deps).bound_box]
        except Exception:
            corners = [obj.matrix_world @ mathutils.Vector(c)
                       for c in obj.bound_box]
        xs = [c.x for c in corners]
        ys = [c.y for c in corners]
        zs = [c.z for c in corners]
        return [min(xs), min(ys), min(zs)], [max(xs), max(ys), max(zs)]

    def _intersects(a, b):
        for i in range(3):
            if a[1][i] < b[0][i] or a[0][i] > b[1][i]:
                return False
        return True

    xml_path = str(xml_path)
    full_backup = re.sub(r"\.xml$", ".full.xml", xml_path)
    if refresh_full_backup or not os.path.isfile(full_backup):
        shutil.copy2(xml_path, full_backup)
        print(f"[STATIC_CROP] preserved complete XML: {full_backup}", flush=True)

    tree = ET.parse(full_backup)
    root = tree.getroot()
    input_shapes = len(root.findall("shape"))
    kept = dropped = unknown = ghost_dropped = 0
    for shape in list(root.findall("shape")):
        drop_ghost, _ghost_reason = _is_static_xml_ghost_blocker(shape, xml_path)
        if drop_ghost:
            root.remove(shape)
            dropped += 1
            ghost_dropped += 1
            continue
        shape_id = shape.get("id") or shape.get("name") or ""
        obj = obj_by_name.get(_norm(re.sub(r"^mesh-", "", shape_id)))
        if obj is None:
            unknown += 1
            kept += 1
            continue
        if _intersects(_world_bbox(obj), crop_box):
            kept += 1
        else:
            root.remove(shape)
            dropped += 1
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
    metadata = dict(crop_spec)
    metadata.update(streetlight_filter_report)
    metadata.update({
        "applied": True,
        "input_shape_count": input_shapes,
        "kept_shapes": kept,
        "dropped_shapes": dropped,
        "ghost_blocker_dropped_shapes": ghost_dropped,
        "unknown_kept_shapes": unknown,
        "full_backup": str(Path(full_backup).resolve()),
    })
    sidecar = write_crop_metadata(xml_path, metadata)
    print(
        f"[STATIC_CROP] kept={kept} dropped={dropped} ghost_dropped={ghost_dropped} unknown_kept={unknown} "
        f"metadata={sidecar} -> {xml_path}",
        flush=True,
    )
    return True


__all__=[k for k in globals() if not k.startswith("__")]
