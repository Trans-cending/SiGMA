# -*- coding: utf-8 -*-
from __future__ import annotations
from .sionna_config import *
from mosaic_core.naming import local_name, strip_blender_suffix
from mosaic_core.rf_materials import (
    UnknownRFMaterialError,
    load_rf_material_registry,
    material_entry,
    rf_material_from_sionna,
    sionna_properties_table,
)
from Blender_xml_storage.static_crop_utils import (
    crop_metadata_is_usable,
    crop_metadata_path,
)

SIONNA_VALID_ITU_TYPES = set(sionna_properties_table())
SIONNA_ITU_TYPE_ALIAS = {
    alias: str(material_entry(material_id)["sionna_type"])
    for alias, material_id in
    load_rf_material_registry().get("sionna_aliases", {}).items()
}
FALLBACK_ITU_TYPE = "concrete"
FALLBACK_MATERIAL_ID = "mat-itu_concrete"

SIONNA_MATERIAL_EXTRAPOLATE = os.environ.get("SIONNA_MATERIAL_EXTRAPOLATE", "1") == "1"
SIONNA_RF_MATERIAL_STRICT = os.environ.get("SIONNA_RF_MATERIAL_STRICT", "1") == "1"
_SIONNA_MATERIAL_FREQ_REPORT_SEEN = set()
ITU_MATERIAL_PROPERTIES_BY_GHZ = sionna_properties_table()


_STATIC_ROOT_CACHE: dict[str, ET.Element] = {}
def _freq_hz_to_float(value) -> float:
    try:
        return float(value)
    except Exception:
        pass
    try:
        import drjit as dr
        return float(dr.detach(value)[0])
    except Exception:
        pass
    try:
        return float(value[0])
    except Exception:
        return 0.0

def _format_ghz_ranges(entries) -> str:
    parts = []
    for (lo, hi), _params in entries or []:
        parts.append(f"{lo:g}-{hi:g} GHz")
    return ", ".join(parts) if parts else "unknown"

def _range_contains(freq_ghz: float, entries) -> bool:
    return any(float(lo) <= float(freq_ghz) <= float(hi) for (lo, hi), _params in entries or [])

def _select_itu_range_params(freq_ghz: float, entries):
    entries = list(entries or [])
    for (lo, hi), params in entries:
        if float(lo) <= float(freq_ghz) <= float(hi):
            return (lo, hi), params
    if not entries:
        return None, None
    return min(
        entries,
        key=lambda item: min(abs(float(freq_ghz) - float(item[0][0])),
                             abs(float(freq_ghz) - float(item[0][1]))),
    )

def _material_name(material) -> str:
    for attr in ("name", "id"):
        try:
            value = getattr(material, attr)
            if callable(value):
                value = value()
            if value:
                return str(value)
        except Exception:
            pass
    return str(material)

def _material_itu_type(material) -> str | None:
    for attr in ("itu_type", "_itu_type"):
        try:
            value = getattr(material, attr)
            if callable(value):
                value = value()
            if value:
                return normalize_itu_type_value(str(value), fallback_type="") or None
        except Exception:
            pass
    text = str(material)
    match = re.search(r"ITURadioMaterial\s+type=([A-Za-z0-9_]+)", text)
    if match:
        return normalize_itu_type_value(match.group(1), fallback_type="") or None
    name = _material_name(material)
    raw = extract_itu_type_from_any_name(name)
    if raw in ITU_MATERIAL_PROPERTIES_BY_GHZ:
        return raw
    norm = normalize_itu_type_value(raw, fallback_type="")
    return norm if norm in ITU_MATERIAL_PROPERTIES_BY_GHZ else None

def _make_extrapolating_itu_callback(itu_type: str):
    entries = ITU_MATERIAL_PROPERTIES_BY_GHZ.get(itu_type, [])

    def callback(f_hz):
        f_hz_float = _freq_hz_to_float(f_hz)
        f_ghz_float = f_hz_float / 1e9 if f_hz_float > 0.0 else 0.0
        _range, params = _select_itu_range_params(f_ghz_float, entries)
        if params is None:
            return -1.0, -1.0
        a, b, c, d = params
        f_ghz = f_hz / 1e9
        try:
            import drjit as dr
            return a * dr.power(f_ghz, b), c * dr.power(f_ghz, d)
        except Exception:
            f = float(f_ghz_float)
            return a * (f ** b), c * (f ** d)

    return callback

def _set_material_frequency_callback(material, itu_type: str) -> bool:
    callback = _make_extrapolating_itu_callback(itu_type)
    for attr in ("frequency_update_callback", "_frequency_update_callback"):
        try:
            setattr(material, attr, callback)
            return True
        except Exception:
            pass
    return False

def set_scene_frequency_with_material_report(scene, frequency_hz: float) -> None:
    freq_ghz = float(frequency_hz) / 1e9
    materials = scene.radio_materials.values() if hasattr(scene, "radio_materials") else []
    out_of_range_types = []

    for material in materials:
        itu_type = _material_itu_type(material)
        if not itu_type:
            continue
        entries = ITU_MATERIAL_PROPERTIES_BY_GHZ.get(itu_type)
        if not entries:
            continue
        ranges_text = _format_ghz_ranges(entries)
        in_range = _range_contains(freq_ghz, entries)
        key = (itu_type, round(freq_ghz, 9), in_range)
        if key not in _SIONNA_MATERIAL_FREQ_REPORT_SEEN:
            _SIONNA_MATERIAL_FREQ_REPORT_SEEN.add(key)
            if in_range:
                print(
                    f"[RF MATERIAL] {itu_type}: requested={freq_ghz:.6g} GHz, "
                    f"supported={ranges_text}, status=in-range",
                    flush=True,
                )
            else:
                out_of_range_types.append(itu_type)
                print(
                    f"WARNING: [RF MATERIAL] {itu_type}: requested={freq_ghz:.6g} GHz is outside "
                    f"supported={ranges_text}; extrapolate={int(SIONNA_MATERIAL_EXTRAPOLATE)}",
                    flush=True,
                )
        elif not in_range:
            out_of_range_types.append(itu_type)

        if (not in_range) and SIONNA_MATERIAL_EXTRAPOLATE:
            if not _set_material_frequency_callback(material, itu_type):
                print(
                    f"WARNING: [RF MATERIAL] {itu_type}: could not install extrapolation callback; "
                    "Sionna may reject this frequency.",
                    flush=True,
                )

    try:
        scene.frequency = frequency_hz
    except Exception as exc:
        if out_of_range_types and SIONNA_MATERIAL_EXTRAPOLATE:
            print(
                "WARNING: scene.frequency failed even after material extrapolation setup: "
                f"{exc}",
                flush=True,
            )
        raise

def extract_itu_type_from_any_name(value: str) -> str:
    s = strip_blender_suffix(str(value))
    s = s.replace("-bsdf", "").replace("_bsdf", "")
    if s.startswith("mat-itu_"):
        s = s[len("mat-itu_"):]
    elif s.startswith("itu_"):
        s = s[len("itu_"):]
    elif s.startswith("mat-"):
        s = s[len("mat-"):]
    return s

def normalize_itu_type_value(raw_value: str, fallback_type: str = FALLBACK_ITU_TYPE) -> str:
    try:
        rf_material_id = rf_material_from_sionna(raw_value)
        return str(material_entry(rf_material_id)["sionna_type"])
    except UnknownRFMaterialError:
        if SIONNA_RF_MATERIAL_STRICT:
            raise
        return fallback_type

def canonical_sionna_material_id(raw_id: str | None, fallback_id: str = FALLBACK_MATERIAL_ID) -> str:
    if raw_id is None:
        if SIONNA_RF_MATERIAL_STRICT:
            raise UnknownRFMaterialError("Sionna shape has no RF material id")
        return fallback_id
    raw_id = strip_blender_suffix(str(raw_id))
    normalized = raw_id
    try:
        rf_material_id = rf_material_from_sionna(normalized)
    except UnknownRFMaterialError:
        if SIONNA_RF_MATERIAL_STRICT:
            raise
        return fallback_id
    return f"mat-itu_{material_entry(rf_material_id)['sionna_type']}"

def make_itu_radio_bsdf(mat_id: str) -> ET.Element:
    mat_id = canonical_sionna_material_id(mat_id)
    typ = mat_id[len("mat-itu_"):]
    bsdf = ET.Element("bsdf", {"type": "itu-radio-material", "id": mat_id, "name": mat_id})
    ET.SubElement(bsdf, "string", {"name": "type", "value": typ})
    return bsdf

def ensure_itu_radio_bsdf(root: ET.Element, mat_id: str) -> None:
    mat_id = canonical_sionna_material_id(mat_id)
    typ = mat_id[len("mat-itu_"):]
    for child in list(root):
        if local_name(child.tag) == "bsdf" and child.get("id") == mat_id:
            child.set("type", "itu-radio-material")
            child.set("name", mat_id)
            for c in list(child):
                child.remove(c)
            ET.SubElement(child, "string", {"name": "type", "value": typ})
            return
    root.insert(0, make_itu_radio_bsdf(mat_id))

def rewrite_relative_mesh_paths(root: ET.Element, xml_dir: Path) -> None:
    xml_dir = Path(xml_dir).absolute()
    mesh_dir = xml_dir / "meshes"
    for elem in root.iter():
        if local_name(elem.tag) != "string" or elem.get("name") != "filename":
            continue
        value = elem.get("value")
        if not value:
            continue
        candidate = Path(value)
        if candidate.is_absolute():
            if candidate.is_file():
                continue
            rebound = mesh_dir / candidate.name
            if rebound.is_file():
                elem.set("value", str(rebound))
            continue
        elem.set("value", str((xml_dir / candidate).absolute()))

def scene_has_emitter(root: ET.Element) -> bool:
    return any(local_name(child.tag) == "emitter" for child in list(root))

def add_constant_light(root: ET.Element, radiance: str = "3.0,3.0,3.0") -> None:
    emitter = ET.Element("emitter", {"type": "constant", "id": "constant_env_light"})
    ET.SubElement(emitter, "rgb", {"name": "radiance", "value": radiance})
    root.insert(0, emitter)

def patch_default_bsdf(root: ET.Element) -> None:
    for elem in root.iter():
        if local_name(elem.tag) == "default" and elem.get("name") == "bsdf":
            elem.set("value", FALLBACK_MATERIAL_ID)

def normalize_top_level_bsdfs(root: ET.Element) -> None:
    kept = set()
    to_remove = []
    to_add = {FALLBACK_MATERIAL_ID}

    for child in list(root):
        if local_name(child.tag) != "bsdf":
            continue

        old_id = child.get("id")
        new_id = canonical_sionna_material_id(old_id)

        if old_id == "default-bsdf" or (new_id == FALLBACK_MATERIAL_ID and old_id != FALLBACK_MATERIAL_ID):
            to_remove.append(child)
            to_add.add(FALLBACK_MATERIAL_ID)
            continue

        child.set("id", new_id)
        child.set("name", new_id)
        child.set("type", "itu-radio-material")
        for c in list(child):
            child.remove(c)
        ET.SubElement(child, "string", {"name": "type", "value": new_id[len("mat-itu_"):]})

        if new_id in kept:
            to_remove.append(child)
        else:
            kept.add(new_id)
            to_add.add(new_id)

    for child in to_remove:
        try:
            root.remove(child)
        except ValueError:
            pass

    for mat_id in sorted(to_add):
        ensure_itu_radio_bsdf(root, mat_id)

def patch_shape_material_refs(root: ET.Element) -> None:
    needed = {FALLBACK_MATERIAL_ID}
    shape_count = 0

    for shape in root.iter():
        if local_name(shape.tag) != "shape":
            continue

        shape_count += 1
        if not shape.get("id"):
            shape.set("id", f"shape_auto_{shape_count:08d}")

        for child in list(shape):
            if local_name(child.tag) == "bsdf":
                shape.remove(child)

        refs = [
            child for child in list(shape)
            if local_name(child.tag) == "ref" and (child.get("name") in {None, "bsdf"})
        ]

        if not refs:
            ET.SubElement(shape, "ref", {"name": "bsdf", "id": FALLBACK_MATERIAL_ID})
            needed.add(FALLBACK_MATERIAL_ID)
            continue

        for ref in refs:
            new_id = canonical_sionna_material_id(ref.get("id", ""))
            ref.set("name", "bsdf")
            ref.set("id", new_id)
            needed.add(new_id)

    for mat_id in sorted(needed):
        ensure_itu_radio_bsdf(root, mat_id)

def sanitize_xml_for_sionna(xml_path: Path, out_path: Path) -> Path:
    tree = ET.parse(str(xml_path))
    root = tree.getroot()
    normalize_top_level_bsdfs(root)
    patch_default_bsdf(root)
    patch_shape_material_refs(root)
    if not scene_has_emitter(root):
        add_constant_light(root, radiance="5.0,5.0,5.0")
    _align_film_pixel_format_with_variant(root)
    _strip_rsu_prop_meshes(root)
    _strip_leaked_static_skeletal_meshes(root)
    try:
        ET.indent(root, space="    ")
    except Exception:
        pass
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(out_path), encoding="utf-8", xml_declaration=True)
    return out_path


def _strip_rsu_prop_meshes(root: ET.Element) -> int:
    """Remove RSU/static-RX placeholder props (sign box + pole) from the RF scene.

    The stage-1 generator spawns a physical sign/pole prop to mark the RSU
    position.  The pole cylinder is placed exactly at the TX antenna location
    (same x/y, z from 1.6 m upward), so every forward ray intersects the metal
    pole immediately and the LOS path is blocked for ALL frames.  These props
    are visualization placeholders and are dropped during RF sanitization
    (their shared meshes live under
    ``_dynamic_shared_meshes/_rsu/``).
    """
    removed = 0
    for shape in list(root.findall("shape")):
        shape_id = str(shape.get("id") or shape.get("name") or "")
        filename = ""
        for child in shape:
            if local_name(child.tag) == "string" and child.get("name") == "filename":
                filename = str(child.get("value") or "")
        if ("/_rsu/" in filename or "rsu_pole" in filename or "rsu_box" in filename
                or "static_rx_pole" in filename or "static_rx_box" in filename
                or shape_id.startswith("mesh-RSU_") or shape_id.startswith("mesh-STATIC_RX_")):
            root.remove(shape)
            removed += 1
    if removed:
        print(f"[SANITIZE] removed {removed} RSU/static-RX placeholder prop mesh(es) from RF scene",
              flush=True)
    return removed


def _strip_leaked_static_skeletal_meshes(root: ET.Element) -> int:
    """Remove generic skeletal components leaked by a live UE map export."""
    removed = 0
    for shape in list(root.findall("shape")):
        shape_id = str(shape.get("id") or shape.get("name") or "")
        if shape_id.lower().startswith("dyn_"):
            continue
        filename = ""
        for child in shape:
            if local_name(child.tag) == "string" and child.get("name") == "filename":
                filename = str(child.get("value") or "")
                break
        shape_key = shape_id.lower()
        mesh_key = Path(filename).stem.lower()
        if (re.fullmatch(r"mesh-skeletalmeshcomponent\d*", shape_key)
                and re.fullmatch(r"skeletalmeshcomponent\d*", mesh_key)):
            root.remove(shape)
            removed += 1
    if removed:
        print(
            f"[SANITIZE] removed {removed} leaked static skeletal component mesh(es)",
            flush=True,
        )
    return removed


def _align_film_pixel_format_with_variant(root: ET.Element) -> None:
    """Silence the HDRFilm 'Monochrome mode enabled' WARN at its source.

    Sionna RT 2.0.1 selects a monochrome Mitsuba variant by default
    (cuda_ad_mono_polarized / llvm_ad_mono_polarized).  Under such variants
    the hdrfilm forces its output to luminance and logs a WARN per
    load_scene unless the film already declares pixel_format=luminance.
    Verified by reproduction: adding the explicit pixel_format removes the
    warning while preserving the independent path computation.
    """
    try:
        import mitsuba as _mi
        variant = str(_mi.variant() or "")
    except Exception:
        variant = ""
    if "mono" not in variant:
        return
    for film in root.iter("film"):
        if not any(
            child.tag == "string" and child.get("name") == "pixel_format"
            for child in film
        ):
            el = ET.Element("string")
            el.set("name", "pixel_format")
            el.set("value", "luminance")
            film.insert(0, el)

def is_material_like_xml_id(xml_id: str | None) -> bool:
    if not xml_id:
        return False
    s = str(xml_id)
    return s == "default-bsdf" or s.startswith("mat-itu_") or s.startswith("itu_")

def prefix_dynamic_ids(dynamic_root: ET.Element, prefix: str = "dyn_") -> None:
    id_map = {}
    for elem in dynamic_root.iter():
        old_id = elem.get("id")
        if not old_id or is_material_like_xml_id(old_id) or old_id.startswith(prefix):
            continue
        new_id = prefix + old_id
        id_map[old_id] = new_id
        elem.set("id", new_id)
    for elem in dynamic_root.iter():
        if local_name(elem.tag) == "ref" and elem.get("id") in id_map:
            elem.set("id", id_map[elem.get("id")])

def clone_root_with_abs_mesh(xml_path: Path) -> ET.Element:
    xml_path = Path(xml_path).absolute()
    root = ET.parse(str(xml_path)).getroot()
    rewrite_relative_mesh_paths(root, xml_path.parent)
    return root

def clone_static_root_cached(static_xml: Path) -> ET.Element:
    static_xml = Path(static_xml).absolute()
    key = str(static_xml)
    root = _STATIC_ROOT_CACHE.get(key)
    if root is None:
        root = clone_root_with_abs_mesh(static_xml)
        _STATIC_ROOT_CACHE[key] = root
    return copy.deepcopy(root)

def infer_town_key(traj_dir: Path) -> str:
    path = Path(traj_dir)
    if re.fullmatch(r"\d{6}", path.name) is None:
        raise ValueError(f"Canonical trajectory directory must be six digits: {traj_dir}")
    scoped_map = os.environ.get("DATASET_MAP_NAME", "").strip()
    if not scoped_map and path.parent.name == "stage1_multimodal":
        scoped_map = path.parent.parent.name
    if scoped_map:
        return scoped_map
    raise ValueError(f"Cannot infer map key from canonical trajectory directory: {traj_dir}")

_STATIC_CROP_PREFLIGHT_CACHE = {}


def _validate_static_xml_crop_preflight(xml_path: Path) -> Path:
    """Fail before GPU scene loading if the required static crop is missing/stale."""
    if str(os.environ.get("SIONNA_REQUIRE_CROPPED_STATIC_XML", "0")).strip().lower() not in {"1", "true", "yes", "on"}:
        return xml_path
    xml_path = Path(xml_path).absolute()
    meta_path = crop_metadata_path(xml_path)
    try:
        xml_stat = xml_path.stat()
        meta_stat = meta_path.stat()
    except OSError as exc:
        raise RuntimeError(
            f"Static XML crop metadata is required before ray tracing: xml={xml_path} metadata={meta_path}"
        ) from exc
    cache_key = (str(xml_path), int(xml_stat.st_mtime_ns), int(meta_stat.st_mtime_ns), int(xml_stat.st_size))
    if _STATIC_CROP_PREFLIGHT_CACHE.get(str(xml_path)) == cache_key:
        return xml_path
    usable, reason, meta = crop_metadata_is_usable(xml_path)
    if not usable:
        raise RuntimeError(f"Static XML preflight failed: {reason}: {xml_path}")
    kept = int(meta.get("kept_shapes") or 0)
    input_shapes = int(meta.get("input_shape_count") or 0)
    dropped = int(meta.get("dropped_shapes") or 0)
    if input_shapes >= 2000 and dropped <= 0:
        raise RuntimeError(
            f"Static crop did not reduce a large scene ({input_shapes} shapes); "
            f"refusing GPU ray tracing to avoid likely OOM: {xml_path}"
        )
    max_shapes = int(os.environ.get("SIONNA_STATIC_XML_MAX_SHAPES", "0") or 0)
    if max_shapes > 0 and kept > max_shapes:
        raise RuntimeError(
            f"Cropped static scene still has {kept} shapes > "
            f"SIONNA_STATIC_XML_MAX_SHAPES={max_shapes}; refusing ray tracing before GPU allocation"
        )
    _STATIC_CROP_PREFLIGHT_CACHE[str(xml_path)] = cache_key
    print(
        f"[STATIC_CROP_PREFLIGHT] OK xml={xml_path} kept={kept} "
        f"dropped={dropped} tx_rx_points={int(meta.get('tx_rx_points') or 0)}",
        flush=True,
    )
    return xml_path


def resolve_static_xml(traj_dir: Path) -> Path:
    # Explicit override via env var (absolute path, or directory containing TownXX/static.xml).
    static_override = os.environ.get("STATIC_XML_PATH", "").strip()
    if static_override:
        p = Path(static_override)
        if p.is_file():
            return _validate_static_xml_crop_preflight(p)
        # Try as directory prefix: <STATIC_XML_PATH>/TownXX/static.xml
        town = infer_town_key(traj_dir)
        p = p / town / "static.xml"
        if p.is_file():
            return _validate_static_xml_crop_preflight(p)
        raise FileNotFoundError(f"STATIC_XML_PATH set but not found: {static_override}")

    town = infer_town_key(traj_dir)
    candidates = [
        DATASET_ROOT / "stage2_xml" / "static" / f"{town}.xml",
        DATASET_ROOT / "stage2_static_mitsuba" / town / "static.xml",
        DATASET_ROOT / "static_mitsuba" / town / "static.xml",
        DATASET_ROOT / "_static_mitsuba" / town / "static.xml",
        DATASET_ROOT / "mitsuba_static" / town / "static.xml",
    ]
    for p in candidates:
        if p.is_file():
            return _validate_static_xml_crop_preflight(p)
    raise FileNotFoundError("Static XML not found. Tried:\n" + "\n".join(str(p) for p in candidates))


def dynamic_xml_dir(traj_dir: Path) -> Path:
    value = str(DYNAMIC_SUBDIR)
    if "{trajectory}" in value:
        return DATASET_ROOT / value.format(trajectory=Path(traj_dir).name)
    p = Path(value)
    if p.is_absolute():
        return p / Path(traj_dir).name / "dynamic" if p.name == "stage2_xml" else p
    # New default fallback: DATASET_ROOT/stage2_xml/TownXX_xxxxxx/dynamic
    if value == "dynamic":
        return DATASET_ROOT / "stage2_xml" / Path(traj_dir).name / "dynamic"
    return Path(traj_dir) / value


def _shape_exact_signature(shape: ET.Element):
    """Return a conservative exact-duplicate signature for a mesh shape."""
    if local_name(shape.tag) != "shape":
        return None
    shape_type = str(shape.get("type") or "")
    filename = ""
    material = ""
    transform = []
    for child in list(shape):
        tag = local_name(child.tag)
        if tag == "string" and child.get("name") == "filename":
            filename = os.path.normpath(str(child.get("value") or ""))
        elif tag == "ref" and child.get("name") in {None, "bsdf"}:
            material = canonical_sionna_material_id(child.get("id", ""))
        elif tag == "transform":
            transform.append(ET.tostring(child, encoding="unicode"))
    if not filename:
        return None
    return (shape_type, filename, material, tuple(transform))


def deduplicate_exact_xml_shapes(root: ET.Element) -> int:
    """Remove only byte-equivalent mesh instances from a merged RT scene.

    Coplanar/near-overlap geometry requires mesh-face information; Stage3 path
    geometry deduplication handles the corresponding repeated path chains.
    """
    if not SIONNA_XML_EXACT_SHAPE_DEDUP:
        return 0
    seen = set()
    removed = 0
    for shape in list(root.findall("shape")):
        sig = _shape_exact_signature(shape)
        if sig is None:
            continue
        if sig in seen:
            root.remove(shape)
            removed += 1
        else:
            seen.add(sig)
    if removed:
        print(f"[XML_DEDUP] removed {removed} exact duplicate mesh shape(s)", flush=True)
    return removed

def make_safe_merged_xml(traj_dir: Path, frame_name: str, work_dir: Path) -> Path:
    dynamic_xml = dynamic_xml_dir(traj_dir) / f"{frame_name}.xml"
    if not dynamic_xml.is_file():
        raise FileNotFoundError(f"Missing dynamic XML: {dynamic_xml}")

    static_xml = resolve_static_xml(traj_dir)
    static_root = clone_static_root_cached(static_xml)
    dyn_root = clone_root_with_abs_mesh(dynamic_xml)
    prefix_dynamic_ids(dyn_root, prefix="dyn_")

    merged_root = copy.deepcopy(static_root)
    for child in list(dyn_root):
        merged_root.append(copy.deepcopy(child))

    # Safe source-level guard: remove only truly identical XML mesh instances.
    # Near-coplanar or overlapping faces require mesh-face information and
    # receive geometry-aware path deduplication.
    deduplicate_exact_xml_shapes(merged_root)

    if not scene_has_emitter(merged_root):
        add_constant_light(merged_root, radiance="1.5,1.5,1.5")

    work_dir.mkdir(parents=True, exist_ok=True)
    merged_xml = work_dir / f"{frame_name}_merged.xml"
    safe_xml = work_dir / f"{frame_name}_merged_sionna_safe.xml"
    ET.ElementTree(merged_root).write(str(merged_xml), encoding="utf-8", xml_declaration=True)
    sanitize_xml_for_sionna(merged_xml, safe_xml)
    return safe_xml


__all__=[k for k in globals() if not k.startswith("__")]
