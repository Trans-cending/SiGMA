#!/usr/bin/env python3
"""Regression tests for static DTT streetlight mesh filtering."""
from __future__ import annotations

from Blender_xml_storage.static_streetlight_filter import (
    is_dtt_streetlight_mesh,
    is_osm_streetlamp_marker,
    normalize_dtt_streetlight_mode,
    position_matches_osm_marker,
    static_streetlight_filter_audit_error,
)


def test_identity_and_osm_protection() -> None:
    assert is_dtt_streetlight_mesh("StreetLight_Mesh.082", "StreetLight_Mesh")
    assert is_dtt_streetlight_mesh(
        "anything",
        "SM_Streetlight3_group3_glass_ext",
    )
    assert not is_dtt_streetlight_mesh("SM_Lightbox231", "SM_LightBox2")
    assert is_osm_streetlamp_marker("MOSAIC_OSMStreetLamp_12838567242")
    assert not is_osm_streetlamp_marker("SM_StreetLight0")
    markers = ((725.9575, 80.355, 0.04),)
    assert position_matches_osm_marker(markers[0], markers)
    assert not position_matches_osm_marker((726.1, 80.355, 0.04), markers)


def test_mode_and_filter_audit() -> None:
    assert normalize_dtt_streetlight_mode("light-only") == "light_only"
    try:
        normalize_dtt_streetlight_mode("unknown")
    except ValueError:
        pass
    else:
        raise AssertionError("invalid mode must fail")

    filtered = {
        "dtt_streetlight_mode": "light_only",
        "dtt_streetlight_mesh_filter_applied": True,
        "dtt_streetlight_candidate_meshes": 600,
        "dtt_streetlight_filtered_meshes": 597,
        "dtt_streetlight_osm_protected_meshes": 3,
    }
    assert static_streetlight_filter_audit_error(filtered, "light_only") == ""
    assert "mismatch" in static_streetlight_filter_audit_error(
        filtered,
        "full",
    )

    full = {
        "dtt_streetlight_mode": "full",
        "dtt_streetlight_mesh_filter_applied": False,
        "dtt_streetlight_candidate_meshes": 600,
        "dtt_streetlight_filtered_meshes": 0,
        "dtt_streetlight_osm_protected_meshes": 3,
    }
    assert static_streetlight_filter_audit_error(full, "full") == ""
    broken = dict(filtered, dtt_streetlight_filtered_meshes=596)
    assert "every" in static_streetlight_filter_audit_error(
        broken,
        "light_only",
    )
    assert "complete" in static_streetlight_filter_audit_error({}, "light_only")


if __name__ == "__main__":
    test_identity_and_osm_protection()
    test_mode_and_filter_audit()
    print("MOSAIC_STATIC_STREETLIGHT_FILTER_TESTS=PASS")
