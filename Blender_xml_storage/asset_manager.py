"""CARLA road-actor mesh indexing and import for Stage III.

Run: python -m sigma stage3 -c configs/town03.yaml.
"""
# -*- coding: utf-8 -*-
try:
    from .blender_common import *
except ImportError:
    from Blender_xml_storage.blender_common import *

try:
    from .material_manager import canonicalize_imported_materials, remove_unused_materials
except ImportError:
    from Blender_xml_storage.material_manager import canonicalize_imported_materials, remove_unused_materials

BP_EXACT_HINTS = [
    ("vehicle.audi.a2", "vehicle", "audi_a2"),
    ("vehicle.audi.etron", "vehicle", "audi_etron"),
    ("vehicle.tesla.model3", "vehicle", "tesla"),
    ("vehicle.mini.cooper_s_2021", "vehicle", "mini2021"),
    ("vehicle.bmw.grandtourer", "vehicle", "bmwgrantourer"),
    ("vehicle.tesla.cybertruck", "truck", "cybertruck"),
    ("vehicle.bmw.grandtourer", "vehicle", "bmw"),
    ("vehicle.chevrolet.impala", "vehicle", "chevrolet"),
    ("vehicle.citroen.c3", "vehicle", "citroen"),
    ("vehicle.dodge.charger", "vehicle", "dodge"),
    ("vehicle.ford.mustang", "vehicle", "mustang"),
    ("vehicle.jeep.wrangler", "vehicle", "jeep"),
    ("vehicle.lincoln.mkz", "vehicle", "lincoln"),
    ("vehicle.mercedes.coupe", "vehicle", "mercedes"),
    ("vehicle.mini.cooper", "vehicle", "mini"),
    ("vehicle.nissan.micra", "vehicle", "micra"),
    ("vehicle.nissan.patrol", "vehicle", "patrol"),
    ("vehicle.seat.leon", "vehicle", "seat"),
    ("vehicle.toyota.prius", "vehicle", "prius"),

    ("vehicle.harley-davidson.low_rider", "vehicle", "harley"),
    ("vehicle.kawasaki.ninja", "vehicle", "kawasaki"),
    ("vehicle.yamaha.yzf", "vehicle", "yamaha"),
    ("vehicle.vespa.zx125", "vehicle", "vespa"),
    ("vehicle.diamondback.century", "vehicle", "diamondback"),
    ("vehicle.gazelle.omafiets", "vehicle", "gazelle"),

    ("vehicle.carlamotors.carlacola", "truck", "carlacola"),
    ("vehicle.carlamotors.firetruck", "truck", "firetruck"),
    ("vehicle.ford.ambulance", "truck", "ambulance"),
    ("vehicle.mercedes.sprinter", "truck", "sprinter"),
    ("vehicle.volkswagen.t2", "truck", "volkswagen_t2"),
    ("vehicle.mitsubishi.fusorosa", "truck", "mitsubishi"),
]

FBX_IMPORT_CACHE = {}
def scan_fbx_library():
    lib = {
        "vehicle": [],
        "truck": [],
        "human": [],
    }

    for category in lib:
        folder = FBX_ROOT / category
        if folder.is_dir():
            lib[category] = sorted(
                list(folder.glob("*.fbx"))
                + list(folder.glob("*.glb"))
                + list(folder.glob("*.gltf"))
            )

    print("===== FBX MODEL LIBRARY =====", flush=True)
    print(f"FBX_ROOT = {FBX_ROOT}", flush=True)
    for category, files in lib.items():
        print(f"  {category}: {len(files)} models", flush=True)
        if XML_VERBOSE:
            for f in files[:20]:
                print(f"    - {f.name}", flush=True)
            if len(files) > 20:
                print("    ...", flush=True)

    return lib

def classify_bp(bp_id):
    bp = str(bp_id).lower()

    if "walker" in bp or "pedestrian" in bp:
        return "human"


    if "static.prop" in bp or "rsu" in bp:
        return "rsu"

    truck_words = [
        "truck",
        "firetruck",
        "ambulance",
        "sprinter",
        "volkswagen.t2",
        "fusorosa",
        "carlamotors",
    ]

    if any(w in bp for w in truck_words):
        return "truck"

    if "vehicle" in bp or "car" in bp or "motorcycle" in bp or "bicycle" in bp:
        return "vehicle"

    return "vehicle"

def score_model_for_bp(bp_id, model_path):
    bp_norm = normalize_text(bp_id)
    bp_compact = compact_text(bp_id)
    stem_norm = normalize_text(model_path.stem)
    stem_compact = compact_text(model_path.stem)

    score = 0

    for bp_prefix, _category, hint in BP_EXACT_HINTS:
        if bp_norm.startswith(normalize_text(bp_prefix)):
            if compact_text(hint) in stem_compact:
                score += 1000

    for token in bp_norm.split("_"):
        if len(token) < 2:
            continue
        if token in stem_norm:
            score += 10
        if token in stem_compact:
            score += 5

    if stem_compact in bp_compact or bp_compact in stem_compact:
        score += 50

    return score

def pick_model_for_actor(bp_id, actor_key, actor_model_cache, model_library):
    if actor_key in actor_model_cache:
        return actor_model_cache[actor_key]

    category = classify_bp(bp_id)

    if category == "rsu":
        actor_model_cache[actor_key] = None
        return None

    search_categories = [category]
    if category == "truck":
        search_categories.append("vehicle")
    elif category == "vehicle":
        search_categories.append("truck")

    candidates = []
    for c in search_categories:
        candidates.extend(model_library.get(c, []))

    # UAV requests use UAV-specific model candidates; an empty candidate list
    # returns None to let the caller handle the missing asset.
    if category == "uav" and not candidates:
        actor_model_cache[actor_key] = None
        return None

    if not candidates:
        for c in ("vehicle", "truck"):
            candidates.extend(model_library.get(c, []))
        if candidates:
            chosen = candidates[stable_int(actor_key) % len(candidates)]
            warning_line(f"no model for bp_id={bp_id}; using {chosen.name}")
            actor_model_cache[actor_key] = chosen
            return chosen
        warning_line(f"no model candidates for bp_id={bp_id} and no fallback available")
        actor_model_cache[actor_key] = None
        return None

    scored = [(score_model_for_bp(bp_id, p), p) for p in candidates]
    scored.sort(key=lambda x: (-x[0], str(x[1])))

    best_score = scored[0][0]
    best = [p for s, p in scored if s == best_score]

    if best_score <= 0:
        chosen = best[stable_int(actor_key) % len(best)]
        if category != "uav":
            warning_line(f"weak model match bp_id={bp_id}; fallback={chosen.name}")
    else:
        chosen = best[0]

    actor_model_cache[actor_key] = chosen

    if XML_VERBOSE:
        print(f"  MODEL actor={actor_key} bp={bp_id} -> {chosen}", flush=True)
    elif str(actor_key) in {"CAV"}:
        progress_status(f"  {actor_key} model: bp={bp_id} -> {chosen.name}")

    return chosen

def import_model_once(model_path):
    if model_path is None:
        return []

    p = Path(model_path).resolve()

    if not p.is_file():
        print(f"  WARNING: model file not found: {p}", flush=True)
        return []

    before = set(bpy.data.objects)

    suffix = p.suffix.lower()

    with quiet_blender_output(XML_QUIET_FBX and not XML_VERBOSE):
        if suffix == ".fbx":
            bpy.ops.import_scene.fbx(filepath=str(p))
        elif suffix in [".glb", ".gltf"]:
            bpy.ops.import_scene.gltf(filepath=str(p))
        else:
            bpy.ops.import_scene.fbx(filepath=str(p))

    after = set(bpy.data.objects)
    new_objects = list(after - before)

    for obj in new_objects:
        obj[DYNAMIC_TAG] = True

    return new_objects

def apply_model_extra_rotation_to_roots(objects):
    if MODEL_EXTRA_ROT == (0.0, 0.0, 0.0):
        return

    obj_set = set(objects)
    roots = [o for o in objects if o.parent not in obj_set]

    for root in roots:
        root.rotation_euler.rotate_axis("X", MODEL_EXTRA_ROT[0])
        root.rotation_euler.rotate_axis("Y", MODEL_EXTRA_ROT[1])
        root.rotation_euler.rotate_axis("Z", MODEL_EXTRA_ROT[2])

def _finite_vec3(v):
    return (
        math.isfinite(float(v[0]))
        and math.isfinite(float(v[1]))
        and math.isfinite(float(v[2]))
    )

def _bbox_diag_from_coords(coords):
    if not coords:
        return 1.0

    xs = [float(v[0]) for v in coords]
    ys = [float(v[1]) for v in coords]
    zs = [float(v[2]) for v in coords]

    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    dz = max(zs) - min(zs)

    return max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-9)

def _valid_triangle_coords(v0, v1, v2, area_eps):
    if not (_finite_vec3(v0) and _finite_vec3(v1) and _finite_vec3(v2)):
        return False

    a = v1 - v0
    b = v2 - v0
    return a.cross(b).length > area_eps

def _triangulate_poly_to_indices(mesh, poly, area_eps):
    """
    Convert one polygon to valid triangle vertex-index triples.
    Degenerate faces are skipped. This prevents Mitsuba-Blender invalid normals.
    """
    raw_indices = [mesh.loops[i].vertex_index for i in poly.loop_indices]

    # Remove repeated vertices while preserving order.
    indices = []
    seen = set()
    for idx in raw_indices:
        if idx in seen:
            continue
        seen.add(idx)
        indices.append(idx)

    if len(indices) < 3:
        return []

    if len(indices) == 3:
        v0 = mesh.vertices[indices[0]].co
        v1 = mesh.vertices[indices[1]].co
        v2 = mesh.vertices[indices[2]].co
        if _valid_triangle_coords(v0, v1, v2, area_eps):
            return [tuple(indices)]
        return []

    coords = [mesh.vertices[i].co.copy() for i in indices]
    tris = []

    try:
        from mathutils.geometry import tessellate_polygon
        local_tris = tessellate_polygon([coords])

        for tri in local_tris:
            if len(tri) != 3:
                continue

            gtri = (
                indices[int(tri[0])],
                indices[int(tri[1])],
                indices[int(tri[2])],
            )

            v0 = mesh.vertices[gtri[0]].co
            v1 = mesh.vertices[gtri[1]].co
            v2 = mesh.vertices[gtri[2]].co

            if _valid_triangle_coords(v0, v1, v2, area_eps):
                tris.append(gtri)

    except Exception:
        # Fallback fan triangulation.
        root = indices[0]
        for i in range(1, len(indices) - 1):
            gtri = (root, indices[i], indices[i + 1])
            v0 = mesh.vertices[gtri[0]].co
            v1 = mesh.vertices[gtri[1]].co
            v2 = mesh.vertices[gtri[2]].co

            if _valid_triangle_coords(v0, v1, v2, area_eps):
                tris.append(gtri)

    return tris

def build_baked_clean_mesh_for_blueprint(obj, actor_class=None):
    """
    Build a clean model-local mesh for FBX blueprint caching.

    Why this is needed:
        The grouped FBX can look visually correct, but imported mesh objects may
        still carry object-level transforms, custom split normals, n-gons, or
        degenerate faces. Mitsuba-Blender later reports these as "invalid normals".

    What this does:
        - bake obj.matrix_world into the vertex coordinates once, while the model
          is still at the origin; this preserves the visual shape as model-local
          geometry;
        - set the future instance matrix_local to identity;
        - triangulate polygons;
        - drop degenerate triangles;
        - preserve material slots and polygon material_index;
        - discard custom normals.

    Actor pose is still applied by the parent empty per frame, so trajectory logic
    is unchanged.
    """
    if obj.type != "MESH" or obj.data is None:
        return None

    src = obj.data

    try:
        src.validate(clean_customdata=True)
    except TypeError:
        try:
            src.validate(verbose=False)
        except Exception:
            pass
    except Exception:
        pass

    model_matrix = obj.matrix_world.copy()

    old_to_new = {}
    verts = []

    for old_i, v in enumerate(src.vertices):
        co = model_matrix @ v.co
        if not _finite_vec3(co):
            continue
        old_to_new[old_i] = len(verts)
        verts.append(co)

    if not verts:
        return None

    # ------------------------------------------------------------
    # Critical alignment fix:
    # Center grouped FBX geometry around its model-local origin before
    # Mitsuba-Blender bakes transforms into PLY vertices. This keeps the
    # exported vehicle aligned with sensors.vehicle_pose / Rx.
    #
    # Re-center only dynamic vehicle-like models. Keep humans untouched because
    # their origin conventions may differ and the visual impact is smaller.
    # ------------------------------------------------------------
    recenter_applied = False
    recenter_offset = None

    if XML_RECENTER_MODEL_LOCAL_ORIGIN and str(actor_class or "").lower() in XML_RECENTER_ACTOR_CLASSES:
        xs = [float(v[0]) for v in verts]
        ys = [float(v[1]) for v in verts]
        zs = [float(v[2]) for v in verts]

        cx = 0.5 * (min(xs) + max(xs))
        cy = 0.5 * (min(ys) + max(ys))
        z0 = min(zs) if XML_RECENTER_Z_TO_BOTTOM else 0.0

        recenter_offset = (cx, cy, z0)

        # Apply even for modest offsets; this normalizes all vehicle/truck
        # local origins consistently. The current problematic case has
        # |center_xy| around 59m.
        for i, v in enumerate(verts):
            verts[i] = v - type(v)((cx, cy, z0))

        recenter_applied = True

    # Area threshold in baked model-local units.
    diag = _bbox_diag_from_coords(verts)
    area_eps = max(1e-16, diag * diag * 1e-12)

    faces = []
    mat_indices = []
    dropped = 0

    for poly in src.polygons:
        tris = _triangulate_poly_to_indices(src, poly, area_eps)

        if not tris:
            dropped += 1
            continue

        for tri in tris:
            if tri[0] not in old_to_new or tri[1] not in old_to_new or tri[2] not in old_to_new:
                dropped += 1
                continue

            new_tri = (
                old_to_new[tri[0]],
                old_to_new[tri[1]],
                old_to_new[tri[2]],
            )

            if len(set(new_tri)) < 3:
                dropped += 1
                continue

            v0 = verts[new_tri[0]]
            v1 = verts[new_tri[1]]
            v2 = verts[new_tri[2]]

            if not _valid_triangle_coords(v0, v1, v2, area_eps):
                dropped += 1
                continue

            faces.append(new_tri)
            mat_indices.append(int(poly.material_index))

    if not faces:
        return None

    # Mitsuba-Blender exports every vertex normal, including normals for
    # vertices excluded from all surviving triangles. Blender represents those
    # isolated vertices with a zero normal, which makes Mitsuba reject an
    # otherwise valid mesh. Compact only unreferenced vertices; face geometry,
    # winding, and material indices remain unchanged.
    used_indices = sorted({index for face in faces for index in face})
    if len(used_indices) != len(verts):
        compact_index = {
            old_index: new_index
            for new_index, old_index in enumerate(used_indices)
        }
        verts = [verts[index] for index in used_indices]
        faces = [tuple(compact_index[index] for index in face) for face in faces]

    clean = bpy.data.meshes.new(obj.data.name + "_blueprint_clean")
    clean.from_pydata(verts, [], faces)

    for mat in src.materials:
        clean.materials.append(mat)

    for poly, mat_idx in zip(clean.polygons, mat_indices):
        if len(clean.materials) > 0:
            poly.material_index = min(max(mat_idx, 0), len(clean.materials) - 1)
        poly.use_smooth = False

    try:
        clean.validate(clean_customdata=True)
    except TypeError:
        try:
            clean.validate(verbose=False)
        except Exception:
            pass
    except Exception:
        pass

    try:
        clean.update(calc_edges=True)
    except TypeError:
        try:
            clean.update()
        except Exception:
            pass
    except Exception:
        pass

    # Discard custom normals. from_pydata creates a clean mesh.
    if XML_VERBOSE and dropped > 0:
        print(
            f"  BLUEPRINT_MESH_CLEAN dropped faces: obj={obj.name}, dropped={dropped}",
            flush=True,
        )

    if recenter_applied:
        progress_status(
            f"  RECENTER_MODEL_LOCAL_ORIGIN class={actor_class} obj={obj.name} "
            f"offset_xyz=({recenter_offset[0]:.6f},{recenter_offset[1]:.6f},{recenter_offset[2]:.6f})"
        )

    return clean

def capture_mesh_blueprints(objects, actor_class=None):
    """
    Store imported mesh geometry as reusable blueprints.

    Important:
        - Bake imported object/world transform into clean model-local mesh data.
        - Store identity matrix_local for future instances.
        - Cache geometry with shared materials for reuse across frames.
        - Mesh data is shared safely across object instances.
    """
    from mathutils import Matrix

    blueprints = []

    for obj in objects:
        if obj.type != "MESH":
            continue

        clean_mesh = build_baked_clean_mesh_for_blueprint(obj, actor_class=actor_class)

        if clean_mesh is None:
            print(f"  WARNING: dropped invalid mesh blueprint object={obj.name}", flush=True)
            continue

        blueprints.append({
            "base_name": obj.name,
            "mesh_data": clean_mesh,
            "matrix_local": Matrix.Identity(4),
        })

    return blueprints

def remove_imported_objects_keep_mesh_data(objects):
    for obj in objects:
        try:
            bpy.data.objects.remove(obj, do_unlink=True)
        except Exception:
            pass

def load_model_blueprints(model_path, actor_class):
    if model_path is None:
        return []

    key = (str(Path(model_path).resolve()), actor_class)

    if key in FBX_IMPORT_CACHE:
        return FBX_IMPORT_CACHE[key]

    if XML_VERBOSE:
        print(f"  IMPORT_ONCE class={actor_class} model={model_path}", flush=True)

    imported = import_model_once(model_path)

    if not imported:
        FBX_IMPORT_CACHE[key] = []
        return []

    canonicalize_imported_materials(imported, actor_class)
    apply_model_extra_rotation_to_roots(imported)

    blueprints = capture_mesh_blueprints(imported, actor_class=actor_class)

    remove_imported_objects_keep_mesh_data(imported)
    remove_unused_materials()

    FBX_IMPORT_CACHE[key] = blueprints

    return blueprints

def instantiate_model_from_blueprints(blueprints, actor_name, model_name=None):
    """
    Fast per-frame model creation:
        create objects sharing cached mesh data.
    Reuse imported geometry and its existing material assignments.
    """
    objects = []

    for i, bp in enumerate(blueprints):
        mesh = bp["mesh_data"]
        obj = bpy.data.objects.new(f"{actor_name}__mesh_{i:02d}", mesh)
        obj[DYNAMIC_TAG] = True
        if model_name is not None:
            obj["fbx_model"] = model_name
        obj.matrix_local = bp["matrix_local"]
        bpy.context.collection.objects.link(obj)
        objects.append(obj)

    return objects


__all__=[k for k in globals() if not k.startswith("__")]
