"""Pure rules for keeping CARLA and Sionna streetlight geometry consistent."""
from __future__ import annotations

import math


VALID_DTT_STREETLIGHT_MODES = frozenset({"full", "light_only", "remove"})
OSM_STREETLAMP_PREFIX = "MOSAIC_OSMStreetLamp_"

# These mesh assets belong to tagged DTT procedural streetlights. Building
# window panels such as SM_LightBox2 belong to building geometry.
DTT_STREETLIGHT_MESH_NAMES = frozenset({
    "streetlight_mesh",
    "sm_streetlight3_group3_glass_ext",
    "sm_streetlight3_group3_glass_int",
})


def normalize_dtt_streetlight_mode(value: object) -> str:
    mode = str(value or "light_only").strip().lower().replace("-", "_")
    if mode not in VALID_DTT_STREETLIGHT_MODES:
        allowed = ", ".join(sorted(VALID_DTT_STREETLIGHT_MODES))
        raise ValueError(
            f"DTT streetlight mode must be one of {allowed}; got {value!r}"
        )
    return mode


def is_osm_streetlamp_marker(name: object) -> bool:
    return str(name or "").startswith(OSM_STREETLAMP_PREFIX)


def is_dtt_streetlight_mesh(object_name: object, mesh_name: object) -> bool:
    names = {str(object_name or "").lower(), str(mesh_name or "").lower()}
    return bool(names & DTT_STREETLIGHT_MESH_NAMES)


def position_matches_osm_marker(
    position: tuple[float, float, float],
    marker_positions: tuple[tuple[float, float, float], ...],
    tolerance_m: float = 0.05,
) -> bool:
    tolerance_sq = float(tolerance_m) ** 2
    return any(
        math.fsum(
            (float(value) - float(marker_value)) ** 2
            for value, marker_value in zip(position, marker)
        )
        <= tolerance_sq
        for marker in marker_positions
    )


def static_streetlight_filter_audit_error(
    metadata: dict,
    expected_mode: object = "",
) -> str:
    """Return an explanation when a static-XML filter audit is inconsistent."""
    recorded_mode = metadata.get("dtt_streetlight_mode")
    if not str(recorded_mode or "").strip():
        return "static XML has no complete DTT streetlight mesh-filter audit"
    try:
        mode = normalize_dtt_streetlight_mode(recorded_mode)
    except ValueError as exc:
        return str(exc)
    if expected_mode:
        expected = normalize_dtt_streetlight_mode(expected_mode)
        if mode != expected:
            return (
                "static XML streetlight mode mismatch: "
                f"metadata={mode!r} expected={expected!r}"
            )

    try:
        candidate = int(metadata.get("dtt_streetlight_candidate_meshes", -1))
        filtered = int(metadata.get("dtt_streetlight_filtered_meshes", -1))
        protected = int(metadata.get("dtt_streetlight_osm_protected_meshes", -1))
    except (TypeError, ValueError):
        return "static XML has an invalid DTT streetlight mesh-filter audit"
    applied = metadata.get("dtt_streetlight_mesh_filter_applied")
    if min(candidate, filtered, protected) < 0:
        return "static XML has no complete DTT streetlight mesh-filter audit"
    if mode == "full":
        if applied is not False or filtered != 0:
            return "full static XML unexpectedly reports filtered streetlight meshes"
    elif applied is not True or filtered + protected != candidate:
        return "static XML did not account for every DTT streetlight candidate mesh"
    return ""
