"""Rebuild each road scene from saved Stage II world_state only.

Run: python -m sigma stage3 -c configs/town03.yaml.
"""
# -*- coding: utf-8 -*-
try:
    from .blender_common import *
except ImportError:
    from Blender_xml_storage.blender_common import *

try:
    from .asset_manager import *
    from .material_manager import *
    from .stage1_loader import *
    from .mesh_writer import *
    from .xml_writer import export_mitsuba_xml, write_manual_dynamic_xml
except ImportError:
    from Blender_xml_storage.asset_manager import *
    from Blender_xml_storage.material_manager import *
    from Blender_xml_storage.stage1_loader import *
    from Blender_xml_storage.mesh_writer import *
    from Blender_xml_storage.xml_writer import export_mitsuba_xml, write_manual_dynamic_xml

def create_actor_empty(actor_name, loc, rot, order):
    empty = bpy.data.objects.new(actor_name, None)
    bpy.context.collection.objects.link(empty)

    empty[DYNAMIC_TAG] = True
    empty.location = carla_to_blender_location(loc)
    empty.rotation_euler = carla_to_blender_rotation(rot, order=order)

    return empty

def parent_objects_to_empty(objects, empty):
    for obj in objects:
        obj.parent = empty
        obj.matrix_parent_inverse.identity()
        # matrix_local was captured as model-local transform.
        # Keeping it under the actor empty preserves template geometry.
        obj.matrix_local = obj.matrix_local

def place_fbx_actor(loc, rot, model_path, actor_name, bp_id, order):
    actor_class = classify_bp(bp_id)

    blueprints = load_model_blueprints(model_path, actor_class)

    if not blueprints:
        return None

    empty = create_actor_empty(actor_name, loc, rot, order)
    model_name = Path(model_path).name if model_path is not None else None
    objects = instantiate_model_from_blueprints(blueprints, actor_name, model_name=model_name)
    parent_objects_to_empty(objects, empty)

    return empty

def create_rsu_pole(loc, actor_name="RSU_0"):
    ensure_core_itu_materials()
    mat = get_or_create_itu_material("itu_metal")

    bx, by, bz = carla_to_blender_location(loc)

    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24,
        radius=0.06,
        depth=3.0,
        location=(bx, by, bz + 1.5),
    )

    pole = bpy.context.object
    pole.name = actor_name
    pole[DYNAMIC_TAG] = True
    pole.data.materials.clear()
    pole.data.materials.append(mat)

    bpy.ops.mesh.primitive_cube_add(
        size=0.35,
        location=(bx, by, bz + 3.15),
    )

    box = bpy.context.object
    box.name = actor_name + "_box"
    box[DYNAMIC_TAG] = True
    box.scale = (0.6, 0.25, 0.25)
    box.data.materials.clear()
    box.data.materials.append(mat)

    return pole


def actor_id_in_selected_ids(actor_id, selected_ids):
    """
    Selected ids can appear as int or str in YAML. Compare both robustly.
    """
    if actor_id is None:
        return False

    sid = str(actor_id)
    if sid in selected_ids:
        return True

    iid = safe_int(actor_id, None)
    if iid is not None and str(iid) in selected_ids:
        return True

    return False

def find_selected_vehicle_info(data):
    """
    Find the actual selected vehicle entry from data["vehicles"].

    Why:
        sensors.vehicle_pose is useful for RX/ego localization, but for visual
        CAV mesh export we want the exact same convention used by all other
        vehicle meshes:
            vehicles[actor_id].location
            vehicles[actor_id].angle
            vehicles[actor_id].bp_id

        This avoids the confusing case where the selected actor is skipped in
        the background loop, while the separate CAV mesh was placed using a
        different pose source.
    """
    vehicles_dict = data.get("vehicles", {}) or {}
    selected_ids = collect_selected_vehicle_ids(data)

    # 0) Some datasets store the selected actor in a separate top-level
    # selected_vehicle dict, while vehicles{} contains only background actors.
    # In that case the actor is resolved directly from selected_vehicle.
    selected_vehicle = data.get("selected_vehicle", {}) or {}
    if isinstance(selected_vehicle, dict):
        sel_loc = selected_vehicle.get("location", None)
        sel_type = (
            selected_vehicle.get("bp_id")
            or selected_vehicle.get("type")
            or selected_vehicle.get("blueprint")
            or selected_vehicle.get("vehicle_type")
        )
        sel_id = (
            selected_vehicle.get("id")
            or selected_vehicle.get("actor_id")
            or selected_vehicle.get("vehicle_id")
        )
        sel_yaw = selected_vehicle.get("yaw_deg", None)

        if sel_loc is not None:
            # Build a vehicles[]-compatible entry.
            # vehicle actor angle convention used by xml_export is [roll, yaw, pitch].
            if selected_vehicle.get("angle") is not None:
                sel_angle = selected_vehicle.get("angle")
            elif sel_yaw is not None:
                sel_angle = [0.0, float(sel_yaw), 0.0]
            else:
                # Last fallback: try sensors.vehicle_pose.rotation.
                sensors = data.get("sensors", {}) or {}
                vp = sensors.get("vehicle_pose", {}) or {}
                rot = vp.get("rotation", {}) if isinstance(vp, dict) else {}
                if isinstance(rot, dict):
                    sel_angle = [
                        float(rot.get("roll", 0.0)),
                        float(rot.get("yaw", 0.0)),
                        float(rot.get("pitch", 0.0)),
                    ]
                else:
                    sel_angle = [0.0, 0.0, 0.0]

            return str(sel_id) if sel_id is not None else "selected_vehicle", {
                "bp_id": sel_type or infer_cav_bp(data),
                "location": sel_loc,
                "angle": sel_angle,
            }, "top_level.selected_vehicle"

    # 1) Direct selected/cav/ego id match.
    for sid in sorted(selected_ids):
        vinfo = get_vehicle_by_id(vehicles_dict, sid)
        if isinstance(vinfo, dict):
            return str(sid), vinfo, "selected_vehicle_id"

    # 2) Resolve id-like vehicle_pose fields directly for alternative YAML names.
    sensors = data.get("sensors", {}) or {}
    vp = sensors.get("vehicle_pose", {}) or {}
    for key in ["id", "actor_id", "vehicle_id", "selected_vehicle_id", "cav_id", "ego_id"]:
        if isinstance(vp, dict) and key in vp:
            vinfo = get_vehicle_by_id(vehicles_dict, vp[key])
            if isinstance(vinfo, dict):
                return str(vp[key]), vinfo, f"sensors.vehicle_pose.{key}"

    # 3) Nearest vehicle to sensors.vehicle_pose.location.
    # This is only a fallback, but it is much safer than silently using a
    # generic CAV pose/model when the selected id is absent.
    if isinstance(vp, dict) and vp.get("location") is not None:
        best_id = None
        best_vinfo = None
        best_d2 = None

        for actor_id, vinfo in vehicles_dict.items():
            if not isinstance(vinfo, dict):
                continue
            if vinfo.get("location") is None:
                continue

            bp = str(vinfo.get("bp_id", "")).lower()
            if "walker" in bp or "pedestrian" in bp:
                continue

            try:
                d2 = squared_distance(vp["location"], vinfo["location"])
            except Exception:
                continue

            if best_d2 is None or d2 < best_d2:
                best_id = actor_id
                best_vinfo = vinfo
                best_d2 = d2

        if best_vinfo is not None and best_d2 is not None:
            if best_d2 <= XML_CAV_NEAREST_EPS_M * XML_CAV_NEAREST_EPS_M:
                return str(best_id), best_vinfo, f"nearest_to_sensors.vehicle_pose<=%.2fm" % XML_CAV_NEAREST_EPS_M

    return None, None, "not_found"



def process_frame_shared_dynamic(data, output_root, frame_name, actor_model_cache, model_library):
    """
    Dynamic-only frame export using shared canonical local meshes.
    Reuse shared mesh files across frame exports.
    """
    vehicles_dict = data.get("vehicles", {}) or {}
    sensor_data = data.get("sensors", {}) or {}
    selected_ids = collect_selected_vehicle_ids(data)

    vp = sensor_data.get("vehicle_pose", {}) or {}
    cav_loc = None
    cav_placed = False
    shape_records = []

    # -----------------------------
    # 1. CAV / selected vehicle
    # -----------------------------
    if XML_CAV_FROM_SELECTED_VEHICLE:
        cav_actor_id, cav_vinfo, cav_source = find_selected_vehicle_info(data)
    else:
        cav_actor_id, cav_vinfo, cav_source = None, None, "disabled"

    if isinstance(cav_vinfo, dict):
        cav_bp = cav_vinfo.get("bp_id", infer_cav_bp(data))
        cav_loc = cav_vinfo.get("location", None)
        cav_rot = cav_vinfo.get("angle", None)

        if cav_actor_id is not None:
            selected_ids.add(str(cav_actor_id))

        if cav_loc is not None and cav_rot is not None:
            cav_model = pick_model_for_actor(
                bp_id=cav_bp,
                actor_key="CAV",
                actor_model_cache=actor_model_cache,
                model_library=model_library,
            )

            progress_status(
                f"  CAV selected actor_id={cav_actor_id} source={cav_source} "
                f"bp={cav_bp} model={Path(cav_model).name if cav_model else None} "
                f"loc={cav_loc}"
            )

            cav_placed = append_model_shapes(
                shape_records=shape_records,
                actor_name="CAV",
                model_path=cav_model,
                bp_id=cav_bp,
                loc=cav_loc,
                rot=cav_rot,
                order=cav_vinfo.get("rotation_order", "ryp"),
                output_root=output_root,
            )

    if not cav_placed and vp:
        cav_bp = infer_cav_bp(data)
        cav_loc = vp.get("location", None)

        cav_model = pick_model_for_actor(
            bp_id=cav_bp,
            actor_key="CAV",
            actor_model_cache=actor_model_cache,
            model_library=model_library,
        )

        warning_line(
            f"CAV fallback source=sensors.vehicle_pose SELECTED_VEHICLE_NOT_FOUND "
            f"bp={cav_bp} model={Path(cav_model).name if cav_model else None} "
            f"loc={cav_loc}"
        )

        cav_placed = append_model_shapes(
            shape_records=shape_records,
            actor_name="CAV",
            model_path=cav_model,
            bp_id=cav_bp,
            loc=vp.get("location", {}),
            rot=vp.get("rotation", {}),
            order="pyr",
            output_root=output_root,
        )

    if not cav_placed:
        msg = f"CAV mesh was not placed for frame {frame_name}; selected_ids={sorted(selected_ids)}"
        if XML_FAIL_IF_CAV_MISSING:
            raise RuntimeError(msg)
        warning_line(msg)

    # --- Additional CAVs (CAV_2 .. CAV_N) ---
    # Current dataset stores selected CAV identities/poses in world_state
    # data["selected_cavs"] is populated from canonical world_state/nodes.cav.
    selected_cav_records = data.get("selected_cavs", []) if isinstance(data, dict) else []
    if isinstance(selected_cav_records, (list, tuple)) and len(selected_cav_records) > 1:
        for cav_idx, cav_info in enumerate(selected_cav_records[1:], start=2):
            if not isinstance(cav_info, dict):
                continue
            cav_id_extra = cav_info.get("id", f"CAV_{cav_idx}")
            selected_ids.add(str(cav_id_extra))
            cav_loc_extra = cav_info.get("location", None)
            cav_rot_extra = cav_info.get("angle", cav_info.get("rotation", None))
            if cav_loc_extra is None or cav_rot_extra is None:
                if XML_FAIL_IF_CAV_MISSING:
                    raise RuntimeError(f"Missing selected CAV pose for CAV_{cav_idx} frame={frame_name}")
                warning_line(f"Missing selected CAV pose for CAV_{cav_idx} frame={frame_name}")
                continue
            cav_bp_extra = cav_info.get("bp_id", cav_info.get("type", "vehicle.car"))
            cav_model_extra = pick_model_for_actor(
                bp_id=cav_bp_extra,
                actor_key=f"CAV_{cav_idx}",
                actor_model_cache=actor_model_cache,
                model_library=model_library,
            )
            append_model_shapes(
                shape_records=shape_records,
                actor_name=f"CAV_{cav_idx}",
                model_path=cav_model_extra,
                bp_id=cav_bp_extra,
                loc=cav_loc_extra,
                rot=cav_rot_extra,
                order=cav_info.get("rotation_order", "pyr"),
                output_root=output_root,
            )

    # -----------------------------
    # 2. RSU
    # -----------------------------
    placed_rsu = False
    rp = sensor_data.get("rsu_pose", {}) or {}
    if rp and XML_EXPORT_RSU_PROPS:
        loc = rp.get("location", None)
        if loc is not None:
            append_rsu_shapes(shape_records, "RSU_0", loc, output_root)
            placed_rsu = True

    # -----------------------------
    # 3. Background actors
    # -----------------------------
    rsu_count = 0
    expected_actors = 0
    placed_ok = 0

    for actor_id, vinfo in sorted(vehicles_dict.items(), key=actor_sort_key):
        aid_int = safe_int(actor_id, None)
        aid_str = str(actor_id)

        bp_id = vinfo.get("bp_id", "")
        loc = vinfo.get("location", None)
        ang = vinfo.get("angle", None)

        if loc is None or ang is None:
            continue

        expected_actors += 1

        if actor_id_in_selected_ids(aid_str, selected_ids):
            expected_actors -= 1
            progress_status(f"  SKIP selected actor exported as CAV actor_id={aid_str}")
            continue

        if cav_loc is not None and bp_id and classify_bp(bp_id) != "human":
            if is_same_location(cav_loc, loc, eps=max(0.25, XML_CAV_NEAREST_EPS_M)):
                expected_actors -= 1
                progress_status(f"  SKIP duplicate CAV by location actor_id={aid_str} loc={loc}")
                continue

        if aid_int == 0:
            expected_actors -= 1
            if not placed_rsu and XML_EXPORT_RSU_PROPS:
                append_rsu_shapes(shape_records, f"RSU_{rsu_count}", loc, output_root)
                placed_rsu = True
                rsu_count += 1
            continue

        actor_class = classify_bp(bp_id)
        actor_name = f"Pedestrian_{actor_id}" if actor_class == "human" else f"Vehicle_{actor_id}"

        model_path = pick_model_for_actor(
            bp_id=bp_id,
            actor_key=aid_str,
            actor_model_cache=actor_model_cache,
            model_library=model_library,
        )

        ok = append_model_shapes(
            shape_records=shape_records,
            actor_name=actor_name,
            model_path=model_path,
            bp_id=bp_id,
            loc=loc,
            rot=ang,
            order="ryp",
            output_root=output_root,
        )

        if ok:
            placed_ok += 1

    # -----------------------------
    # -----------------------------

    # -----------------------------
    # 5. Write XML
    # -----------------------------
    if XML_FRAME_SUBDIR:
        frame_dir = os.path.join(output_root, frame_name)
        os.makedirs(frame_dir, exist_ok=True)
        xml_path = os.path.join(frame_dir, "scene.xml")
    else:
        os.makedirs(output_root, exist_ok=True)
        xml_path = os.path.join(output_root, f"{frame_name}.xml")

    write_manual_dynamic_xml(xml_path, shape_records)

    if expected_actors > 0 and placed_ok != expected_actors:
        warning_line(f"actor count mismatch — expected {expected_actors}, placed {placed_ok} in frame {frame_name}")

def process_frame(data, output_root, frame_name, actor_model_cache, model_library):
    if XML_EXPORT_MODE == "dynamic" and XML_DYNAMIC_SHARED_MESH:
        return process_frame_shared_dynamic(
            data=data,
            output_root=output_root,
            frame_name=frame_name,
            actor_model_cache=actor_model_cache,
            model_library=model_library,
        )

    clean_dynamic_actors()

    vehicles_dict = data.get("vehicles", {}) or {}
    sensor_data = data.get("sensors", {}) or {}
    selected_ids = collect_selected_vehicle_ids(data)

    vp = sensor_data.get("vehicle_pose", {}) or {}
    cav_loc = None
    cav_actor_id = None
    cav_placed = False

    # -----------------------------
    # 1. CAV / selected vehicle
    # -----------------------------
    if XML_CAV_FROM_SELECTED_VEHICLE:
        cav_actor_id, cav_vinfo, cav_source = find_selected_vehicle_info(data)
    else:
        cav_actor_id, cav_vinfo, cav_source = None, None, "disabled"

    if isinstance(cav_vinfo, dict):
        cav_bp = cav_vinfo.get("bp_id", infer_cav_bp(data))
        cav_loc = cav_vinfo.get("location", None)
        cav_rot = cav_vinfo.get("angle", None)

        if cav_actor_id is not None:
            selected_ids.add(str(cav_actor_id))

        if cav_loc is not None and cav_rot is not None:
            cav_model = pick_model_for_actor(
                bp_id=cav_bp,
                actor_key="CAV",
                actor_model_cache=actor_model_cache,
                model_library=model_library,
            )

            progress_status(
                f"  CAV selected actor_id={cav_actor_id} source={cav_source} "
                f"bp={cav_bp} model={Path(cav_model).name if cav_model else None} "
                f"loc={cav_loc}"
            )

            result = place_fbx_actor(
                loc=cav_loc,
                rot=cav_rot,
                model_path=cav_model,
                actor_name="CAV",
                bp_id=cav_bp,
                order=cav_vinfo.get("rotation_order", "ryp"),
            )
            cav_placed = result is not None

    # Resolve the fallback pose when the selected-vehicle entry is unusable.
    if not cav_placed and vp:
        cav_bp = infer_cav_bp(data)
        cav_loc = vp.get("location", None)

        cav_model = pick_model_for_actor(
            bp_id=cav_bp,
            actor_key="CAV",
            actor_model_cache=actor_model_cache,
            model_library=model_library,
        )

        warning_line(
            f"CAV fallback source=sensors.vehicle_pose SELECTED_VEHICLE_NOT_FOUND "
            f"bp={cav_bp} model={Path(cav_model).name if cav_model else None} "
            f"loc={cav_loc}"
        )

        result = place_fbx_actor(
            loc=vp.get("location", {}),
            rot=vp.get("rotation", {}),
            model_path=cav_model,
            actor_name="CAV",
            bp_id=cav_bp,
            order="pyr",
        )
        cav_placed = result is not None

    if not cav_placed:
        msg = f"CAV mesh was not placed for frame {frame_name}; selected_ids={sorted(selected_ids)}"
        if XML_FAIL_IF_CAV_MISSING:
            raise RuntimeError(msg)
        warning_line(msg)

    # --- Additional CAVs (CAV_2 .. CAV_N) ---
    selected_cav_records = data.get("selected_cavs", []) if isinstance(data, dict) else []
    if isinstance(selected_cav_records, (list, tuple)) and len(selected_cav_records) > 1:
        for cav_idx, cav_info in enumerate(selected_cav_records[1:], start=2):
            if not isinstance(cav_info, dict):
                continue
            cav_id_extra = cav_info.get("id", f"CAV_{cav_idx}")
            selected_ids.add(str(cav_id_extra))
            cav_loc_extra = cav_info.get("location", None)
            cav_rot_extra = cav_info.get("angle", cav_info.get("rotation", None))
            if cav_loc_extra is None or cav_rot_extra is None:
                if XML_FAIL_IF_CAV_MISSING:
                    raise RuntimeError(f"Missing selected CAV pose for CAV_{cav_idx} frame={frame_name}")
                warning_line(f"Missing selected CAV pose for CAV_{cav_idx} frame={frame_name}")
                continue
            cav_bp_extra = cav_info.get("bp_id", cav_info.get("type", "vehicle.car"))
            cav_model_extra = pick_model_for_actor(
                bp_id=cav_bp_extra,
                actor_key=f"CAV_{cav_idx}",
                actor_model_cache=actor_model_cache,
                model_library=model_library,
            )
            place_fbx_actor(
                loc=cav_loc_extra,
                rot=cav_rot_extra,
                model_path=cav_model_extra,
                actor_name=f"CAV_{cav_idx}",
                bp_id=cav_bp_extra,
                order=cav_info.get("rotation_order", "pyr"),
            )

    # -----------------------------
    # 2. RSU
    # -----------------------------
    placed_rsu = False

    rp = sensor_data.get("rsu_pose", {}) or {}
    if rp:
        loc = rp.get("location", None)
        if loc is not None:
            create_rsu_pole(loc, "RSU_0")
            placed_rsu = True

    # -----------------------------
    # 3. Background actors
    # -----------------------------
    rsu_count = 0
    expected_actors = 0
    placed_ok = 0

    for actor_id, vinfo in sorted(vehicles_dict.items(), key=actor_sort_key):
        aid_int = safe_int(actor_id, None)
        aid_str = str(actor_id)

        bp_id = vinfo.get("bp_id", "")
        loc = vinfo.get("location", None)
        ang = vinfo.get("angle", None)

        if loc is None or ang is None:
            continue

        expected_actors += 1

        # Export the selected actor once as actor_name="CAV".
        if actor_id_in_selected_ids(aid_str, selected_ids):
            expected_actors -= 1
            progress_status(f"  SKIP selected actor exported as CAV actor_id={aid_str}")
            continue

        # Location-resolved CAVs are also exported once; skip their matching
        # background entries.
        if cav_loc is not None and bp_id and classify_bp(bp_id) != "human":
            if is_same_location(cav_loc, loc, eps=max(0.25, XML_CAV_NEAREST_EPS_M)):
                expected_actors -= 1
                progress_status(f"  SKIP duplicate CAV by location actor_id={aid_str} loc={loc}")
                continue

        if aid_int == 0:
            expected_actors -= 1
            if not placed_rsu:
                create_rsu_pole(loc, f"RSU_{rsu_count}")
                placed_rsu = True
                rsu_count += 1
            continue

        actor_class = classify_bp(bp_id)
        actor_name = f"Pedestrian_{actor_id}" if actor_class == "human" else f"Vehicle_{actor_id}"

        model_path = pick_model_for_actor(
            bp_id=bp_id,
            actor_key=aid_str,
            actor_model_cache=actor_model_cache,
            model_library=model_library,
        )

        result = place_fbx_actor(
            loc=loc,
            rot=ang,
            model_path=model_path,
            actor_name=actor_name,
            bp_id=bp_id,
            order="ryp",
        )
        if result is not None:
            placed_ok += 1


    warn_unknown_materials()

    # -----------------------------
    # 4. Export XML
    # -----------------------------
    if XML_FRAME_SUBDIR:
        frame_dir = os.path.join(output_root, frame_name)
        os.makedirs(frame_dir, exist_ok=True)
        xml_path = os.path.join(frame_dir, "scene.xml")
    else:
        os.makedirs(output_root, exist_ok=True)
        xml_path = os.path.join(output_root, f"{frame_name}.xml")

    export_mitsuba_xml(xml_path)

    # Count check: warn if actual placed actors differ from expected.
    if expected_actors > 0 and placed_ok != expected_actors:
        warning_line(f"actor count mismatch — expected {expected_actors}, placed {placed_ok} in frame {frame_name}")


__all__=[k for k in globals() if not k.startswith("__")]
