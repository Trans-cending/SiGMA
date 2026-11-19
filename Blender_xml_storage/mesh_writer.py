"""Shared road-actor meshes in map-scoped Stage III output.

Run: python -m sigma stage3 -c configs/town03.yaml.
"""
# -*- coding: utf-8 -*-
import contextlib
import time

try:
    from .blender_common import *
except ImportError:
    from Blender_xml_storage.blender_common import *

try:
    from .asset_manager import *
    from .material_manager import *
except ImportError:
    from Blender_xml_storage.asset_manager import *
    from Blender_xml_storage.material_manager import *

_SHARED_MESH_EXPORT_CACHE = {}
_SHARED_RSU_MESH_EXPORT_CACHE = {}

def sanitize_xml_id(s):
    s = str(s)
    s = re.sub(r"[^0-9A-Za-z_\\-\\.]+", "_", s)
    if not s:
        s = "id"
    if s[0].isdigit():
        s = "id_" + s
    return s

def is_valid_ply_file(path):
    """
    Validate PLY files before reuse.
    Checks that the PLY header uses actual newline delimiters.
    """
    try:
        p = Path(path)
        if (not p.is_file()) or p.stat().st_size <= 0:
            return False
        with open(p, "rb") as f:
            head = f.read(8192)
        if not head.startswith(b"ply\n"):
            return False
        if b"format binary_little_endian 1.0\n" not in head[:512]:
            return False
        if b"end_header\n" not in head:
            return False
        return True
    except Exception:
        return False

@contextlib.contextmanager
def shared_mesh_write_lock(lock_base):
    """
    Directory-based lock that works across SLURM processes on shared filesystems.

    os.replace() is atomic on local Linux filesystems, but on shared/NFS/Lustre
    filesystems we can still hit EBUSY if many array tasks repeatedly replace
    the same file.  mkdir(lockdir) is the safest simple cross-process mutex.
    """
    import shutil

    lock_dir = Path(str(lock_base) + ".lock")
    lock_dir.parent.mkdir(parents=True, exist_ok=True)
    start = time.time()
    acquired = False

    while True:
        try:
            os.mkdir(lock_dir)
            acquired = True
            try:
                owner = lock_dir / "owner.txt"
                owner.write_text(
                    f"pid={os.getpid()} host={os.uname().nodename} time={time.time()}\n",
                    encoding="utf-8",
                )
            except Exception:
                pass
            break
        except FileExistsError:
            # Remove clearly stale locks left by killed jobs.
            try:
                age = time.time() - lock_dir.stat().st_mtime
                if age > XML_SHARED_MESH_LOCK_STALE_SEC:
                    shutil.rmtree(lock_dir, ignore_errors=True)
                    continue
            except Exception:
                pass

            if time.time() - start > XML_SHARED_MESH_LOCK_TIMEOUT_SEC:
                raise TimeoutError(f"Timed out waiting for shared mesh lock: {lock_dir}")
            time.sleep(0.10)

    try:
        yield
    finally:
        if acquired:
            try:
                shutil.rmtree(lock_dir, ignore_errors=True)
            except Exception:
                pass

def atomic_replace_with_retry(tmp_path, dst_path, max_attempts=80):
    """
    Replace dst_path with retry for shared filesystem transient EBUSY/ESTALE.
    """
    tmp_path = Path(tmp_path)
    dst_path = Path(dst_path)

    last_err = None
    for attempt in range(int(max_attempts)):
        try:
            os.replace(str(tmp_path), str(dst_path))
            return
        except OSError as e:
            last_err = e
            # 16=EBUSY, 116=ESTALE on many Linux systems. Also retry generic
            # transient OSError from network filesystems.
            if getattr(e, "errno", None) not in {16, 116, 11, 5}:
                raise
            time.sleep(min(0.05 * (attempt + 1), 2.0))

    raise last_err

def shared_mesh_signature_for_model(model_path, actor_class):
    p = Path(model_path).resolve()
    try:
        stat = p.stat()
        fbx_sig = f"mtime_ns={stat.st_mtime_ns}|size={stat.st_size}"
    except Exception:
        fbx_sig = "missing_stat"
    return (
        f"path={p}\n"
        f"{fbx_sig}\n"
        f"actor_class={actor_class}\n"
        f"recenter={XML_RECENTER_MODEL_LOCAL_ORIGIN}\n"
        f"z_to_bottom={XML_RECENTER_Z_TO_BOTTOM}\n"
        f"extra_rot={MODEL_EXTRA_ROT_DEG}\n"
        f"writer=shared_mesh_transform_v5_nohash_lock\n"
    )

def shared_mesh_signature_matches(model_dir, signature):
    try:
        sig_path = Path(model_dir) / ".mesh_signature.txt"
        return sig_path.is_file() and sig_path.read_text(encoding="utf-8") == signature
    except Exception:
        return False

def write_shared_mesh_signature(model_dir, signature):
    sig_path = Path(model_dir) / ".mesh_signature.txt"
    tmp = sig_path.with_suffix(sig_path.suffix + f".tmp.{os.getpid()}.{time.time_ns()}")
    tmp.write_text(signature, encoding="utf-8")
    atomic_replace_with_retry(tmp, sig_path)

def write_binary_little_endian_ply(mesh, polygon_indices, ply_path):
    """
    Write a clean triangle PLY for selected mesh polygons.

    Meshes produced by build_baked_clean_mesh_for_blueprint() are triangulated,
    finite, and recentered when requested.
    """
    import struct

    ply_path = Path(ply_path)
    ply_path.parent.mkdir(parents=True, exist_ok=True)

    vertex_map = {}
    vertices = []
    faces = []

    for poly_idx in polygon_indices:
        poly = mesh.polygons[poly_idx]
        if len(poly.vertices) != 3:
            # build_baked_clean_mesh_for_blueprint should have triangulated.
            continue

        face = []
        for old_vi in poly.vertices:
            old_vi = int(old_vi)
            if old_vi not in vertex_map:
                co = mesh.vertices[old_vi].co
                vertex_map[old_vi] = len(vertices)
                vertices.append((float(co.x), float(co.y), float(co.z)))
            face.append(vertex_map[old_vi])

        if len(set(face)) == 3:
            faces.append(tuple(face))

    if not vertices or not faces:
        return False

    tmp_path = ply_path.with_suffix(ply_path.suffix + f".tmp.{os.getpid()}.{time.time_ns()}")

    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(vertices)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        f"element face {len(faces)}\n"
        "property list uchar int vertex_indices\n"
        "end_header\n"
    ).encode("ascii")

    with open(tmp_path, "wb") as f:
        f.write(header)
        for x, y, z in vertices:
            f.write(struct.pack("<fff", x, y, z))
        for a, b, c in faces:
            f.write(struct.pack("<Biii", 3, int(a), int(b), int(c)))

    atomic_replace_with_retry(tmp_path, ply_path)
    return True

def shared_mesh_root_for_output(output_root):
    if XML_SHARED_MESH_ROOT:
        return Path(XML_SHARED_MESH_ROOT).resolve()

    # output_root = DATASET_ROOT/stage2_xml/TownXX_xxxxxx/dynamic
    # Put shared meshes once at DATASET_ROOT/_dynamic_shared_meshes.
    out = Path(output_root).resolve()
    try:
        dataset_root = out.parent.parent
    except Exception:
        dataset_root = out.parent
    return (dataset_root / XML_SHARED_MESH_SUBDIR).resolve()

def shared_model_key(model_path, actor_class):
    """
    Return (directory_name, signature) for shared mesh cache.

    Default directory names use readable frame identifiers:
        vehicle/mercedes/
        vehicle/audi_etron/

    Correctness is protected by:
        - .mesh_signature.txt inside the directory;
        - XML_FORCE_REWRITE_SHARED_PLY / signature mismatch rewrite;
        - cross-process directory lock.

    If you need multiple incompatible mesh-processing configs to run
    concurrently against the same cache root, set XML_SHARED_MESH_USE_HASH=1
    or use different XML_SHARED_MESH_ROOT values.
    """
    p = Path(model_path).resolve()

    # Prefer a readable path relative to FBX_ROOT/category. Current library is
    # flat, so this usually becomes just "mercedes" or "audi_etron".
    try:
        rel = p.relative_to((FBX_ROOT / actor_class).resolve()).with_suffix("")
    except Exception:
        try:
            rel = p.relative_to(FBX_ROOT).with_suffix("")
        except Exception:
            rel = Path(p.stem)

    dir_name = sanitize_xml_id("__".join(rel.parts))
    signature = shared_mesh_signature_for_model(p, actor_class)

    if XML_SHARED_MESH_USE_HASH:
        h = hashlib.sha1(signature.encode("utf-8")).hexdigest()[:12]
        dir_name = f"{dir_name}_{h}"

    return dir_name, signature

def ensure_shared_model_meshes(model_path, actor_class, output_root):
    """
    Ensure one canonical local mesh library exists for the FBX model.

    Returns list of records:
        {"filename": absolute path, "material": "itu_metal", "part": int}
    """
    if model_path is None:
        return []

    model_path = Path(model_path).resolve()
    root = shared_mesh_root_for_output(output_root)
    dir_name, signature = shared_model_key(model_path, actor_class)
    model_dir = root / actor_class / dir_name

    cache_key = (str(model_path), actor_class, str(root), dir_name, signature)

    if cache_key in _SHARED_MESH_EXPORT_CACHE:
        return _SHARED_MESH_EXPORT_CACHE[cache_key]

    blueprints = load_model_blueprints(model_path, actor_class)
    records = []

    # One writer per canonical model directory. This prevents concurrent array
    # jobs from replacing the same PLY at the same time.
    with shared_mesh_write_lock(model_dir / ".write"):
        signature_ok = shared_mesh_signature_matches(model_dir, signature)

        for bi, bp in enumerate(blueprints):
            mesh = bp.get("mesh_data")
            if mesh is None:
                continue

            groups = mesh_material_groups(mesh, actor_class=actor_class)

            for itu_mat, poly_indices in sorted(groups.items()):
                if not poly_indices:
                    continue

                ply_name = f"mesh_{bi:02d}_{sanitize_xml_id(itu_mat)}.ply"
                ply_path = model_dir / ply_name

                need_write = (
                    XML_FORCE_REWRITE_SHARED_PLY
                    or (not signature_ok)
                    or (not is_valid_ply_file(ply_path))
                )

                if need_write:
                    ok = write_binary_little_endian_ply(mesh, poly_indices, ply_path)
                    if ok:
                        progress_status(
                            f"  SHARED_MESH_WRITE class={actor_class} model={model_path.name} "
                            f"part={bi} material={itu_mat} -> {ply_path}"
                        )
                    else:
                        warning_line(
                            f"no triangles for shared mesh class={actor_class} "
                            f"model={model_path.name} part={bi} material={itu_mat}"
                        )
                        continue

                records.append({
                    "filename": str(ply_path),
                    "material": itu_mat,
                    "part": bi,
                })

        if records:
            write_shared_mesh_signature(model_dir, signature)

    _SHARED_MESH_EXPORT_CACHE[cache_key] = records

    if not records:
        warning_line(f"no shared mesh records for model={model_path}")

    return records

def ensure_shared_rsu_meshes(output_root):
    """
    Write canonical RSU pole/head meshes once and return records.

    RSU is shared by all frames and all trajectories, so protect it with the
    same lock and process-local cache as vehicle meshes. This specifically
    fixes EBUSY on rsu_pole.ply under SLURM array concurrency.
    """
    root = shared_mesh_root_for_output(output_root) / "_rsu"
    pole = root / "rsu_pole.ply"
    box = root / "rsu_box.ply"
    cache_key = str(root.resolve())

    records = [
        {"filename": str(pole), "material": "itu_metal", "part": 0},
        {"filename": str(box), "material": "itu_metal", "part": 1},
    ]

    if cache_key in _SHARED_RSU_MESH_EXPORT_CACHE:
        return records

    signature = (
        "rsu_mesh\n"
        "pole_radius=0.06|pole_height=3.0|pole_segments=24|pole_z_center=1.5\n"
        "box_size=(0.35,0.35,0.35)|box_center=(0,0,3.15)\n"
        "writer=shared_mesh_transform_v5_nohash_lock\n"
    )

    with shared_mesh_write_lock(root / ".write"):
        signature_ok = shared_mesh_signature_matches(root, signature)

        if XML_FORCE_REWRITE_SHARED_PLY or (not signature_ok) or (not is_valid_ply_file(pole)):
            write_cylinder_ply(pole, radius=0.06, height=3.0, segments=24, z_center=1.5)
            progress_status(f"  SHARED_MESH_WRITE RSU pole -> {pole}")

        if XML_FORCE_REWRITE_SHARED_PLY or (not signature_ok) or (not is_valid_ply_file(box)):
            write_box_ply(box, size=(0.35, 0.35, 0.35), center=(0.0, 0.0, 3.15))
            progress_status(f"  SHARED_MESH_WRITE RSU box -> {box}")

        write_shared_mesh_signature(root, signature)

    _SHARED_RSU_MESH_EXPORT_CACHE[cache_key] = True
    return records

def write_cylinder_ply(path, radius=0.06, height=3.0, segments=24, z_center=1.5):
    import struct
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    verts = []
    z0 = z_center - 0.5 * height
    z1 = z_center + 0.5 * height

    for z in [z0, z1]:
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            verts.append((radius * math.cos(a), radius * math.sin(a), z))

    bottom_center = len(verts)
    verts.append((0.0, 0.0, z0))
    top_center = len(verts)
    verts.append((0.0, 0.0, z1))

    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        b0, b1 = i, j
        t0, t1 = segments + i, segments + j
        faces.append((b0, b1, t1))
        faces.append((b0, t1, t0))
        faces.append((bottom_center, b1, b0))
        faces.append((top_center, t0, t1))

    _write_simple_ply(path, verts, faces)

def write_box_ply(path, size=(0.35, 0.35, 0.35), center=(0.0, 0.0, 3.15)):
    sx, sy, sz = [float(v) for v in size]
    cx, cy, cz = [float(v) for v in center]
    x0, x1 = cx - sx/2, cx + sx/2
    y0, y1 = cy - sy/2, cy + sy/2
    z0, z1 = cz - sz/2, cz + sz/2

    verts = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]
    faces = [
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1),
        (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3),
        (3, 7, 4), (3, 4, 0),
    ]
    _write_simple_ply(path, verts, faces)

def _write_simple_ply(path, verts, faces):
    import struct
    path = Path(path)
    tmp_path = path.with_suffix(path.suffix + f".tmp.{os.getpid()}")
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(verts)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        f"element face {len(faces)}\n"
        "property list uchar int vertex_indices\n"
        "end_header\n"
    ).encode("ascii")

    with open(tmp_path, "wb") as f:
        f.write(header)
        for x, y, z in verts:
            f.write(struct.pack("<fff", float(x), float(y), float(z)))
        for a, b, c in faces:
            f.write(struct.pack("<Biii", 3, int(a), int(b), int(c)))

    atomic_replace_with_retry(tmp_path, path)

def actor_matrix_world(loc, rot, order):
    """
    Build the same actor transform that create_actor_empty() would use.
    """
    from mathutils import Matrix, Euler

    bx, by, bz = carla_to_blender_location(loc)
    rx, ry, rz = carla_to_blender_rotation(rot, order=order)

    T = Matrix.Translation((bx, by, bz))
    R = Euler((rx, ry, rz), "XYZ").to_matrix().to_4x4()
    return T @ R

def matrix_to_mitsuba_value(M):
    """
    Mitsuba XML matrix value is row-major with translation in the last column.
    """
    return " ".join(f"{float(M[i][j]):.10g}" for i in range(4) for j in range(4))

def mesh_filename_for_xml(filename, xml_path):
    p = Path(filename).resolve()
    if XML_SHARED_MESH_ABS_PATHS:
        return str(p)
    try:
        return os.path.relpath(str(p), start=str(Path(xml_path).parent.resolve()))
    except Exception:
        return str(p)

def append_model_shapes(shape_records, actor_name, model_path, bp_id, loc, rot, order, output_root):
    actor_class = classify_bp(bp_id)
    mesh_records = ensure_shared_model_meshes(model_path, actor_class, output_root)

    if not mesh_records:
        return False

    M = actor_matrix_world(loc, rot, order=order)
    M_value = matrix_to_mitsuba_value(M)

    for rec in mesh_records:
        part = int(rec["part"])
        mat = rec["material"]
        shape_records.append({
            "id": f"mesh-{sanitize_xml_id(actor_name)}__mesh_{part:02d}_{sanitize_xml_id(mat)}",
            "name": f"mesh-{sanitize_xml_id(actor_name)}__mesh_{part:02d}",
            "filename": rec["filename"],
            "material": mat,
            "matrix": M_value,
        })

    return True

def append_rsu_shapes(shape_records, actor_name, loc, output_root):
    bx, by, bz = carla_to_blender_location(loc)

    from mathutils import Matrix
    M = Matrix.Translation((bx, by, bz))
    M_value = matrix_to_mitsuba_value(M)

    for rec in ensure_shared_rsu_meshes(output_root):
        part = int(rec["part"])
        mat = rec["material"]
        shape_records.append({
            "id": f"mesh-{sanitize_xml_id(actor_name)}__mesh_{part:02d}_{sanitize_xml_id(mat)}",
            "name": f"mesh-{sanitize_xml_id(actor_name)}__mesh_{part:02d}",
            "filename": rec["filename"],
            "material": mat,
            "matrix": M_value,
        })

    return True




def append_procedural_uav_shapes(shape_records, record, output_root):
    from mathutils import Matrix

    label = str(record.get("label") or record.get("name") or "uav1")
    actor_name = f"UAV_{int(record.get('index', 1))}"
    loc = record.get("location")
    rot = record.get("rotation", [0.0, 0.0, 0.0])
    order = record.get("rotation_order", "pyr")
    if loc is None:
        return False

    base = actor_matrix_world(loc, rot, order)
    meshes = ensure_shared_uav_meshes(output_root)

    def add(component, local_matrix, suffix):
        rec = meshes[component]
        shape_records.append({
            "id": f"mesh-{sanitize_xml_id(actor_name)}_{sanitize_xml_id(suffix)}",
            "name": f"mesh-{sanitize_xml_id(actor_name)}_{sanitize_xml_id(suffix)}",
            "filename": rec["filename"],
            "material": rec["material"],
            "matrix": matrix_to_mitsuba_value(base @ local_matrix),
        })

    add("body", Matrix.Identity(4), "body")
    add("arm_x", Matrix.Identity(4), "arm_x")
    add("arm_y", Matrix.Identity(4), "arm_y")

    for rotor_name, x, y in _uav_rotor_layout():
        phase = _uav_rotor_phase(record, rotor_name)
        local = (
            Matrix.Translation((float(x), float(y), float(XML_UAV_ROTOR_Z)))
            @ Matrix.Rotation(float(phase), 4, "Z")
        )
        add("rotor", local, rotor_name)

    progress_status(
        f"  UAV procedural actor={actor_name} label={label} "
        f"loc={loc} source_frames={record.get('_source_frames')}"
    )
    return True



__all__=[k for k in globals() if not k.startswith("__")]
