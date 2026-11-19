"""Exercise local OSM–Overture reconciliation on constructed geometry."""
import json
from pathlib import Path

import pytest

pytest.importorskip("shapely")

from sigma.stage1 import run_stage1, validate_stage1_config
from test_stage1_geography import write_osm


def site_sources(tmp_path):
    osm = tmp_path / "site.osm"
    write_osm(osm)
    text = osm.read_text().replace("</osm>", '<way id="50"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way></osm>')
    osm.write_text(text)
    overture = tmp_path / "overture.geojson"
    overture.write_text(json.dumps({"type": "FeatureCollection", "features": [{"type": "Feature", "id": "new-building", "geometry": {"type": "Polygon", "coordinates": [[[-111.9996, 33.0005], [-111.9994, 33.0005], [-111.9994, 33.0007], [-111.9996, 33.0007], [-111.9996, 33.0005]]]}, "properties": {"height": 11, "sources": [{"dataset": "test fixture", "record_id": "fixture-1"}]}}]}))
    return {"map_name": "SigmaFixture", "mode": "prepare", "osm_file": osm.name, "overture_geojson": overture.name, "height_mode": "overture"}


def test_full_local_reconciliation_and_relative_config_paths(tmp_path):
    settings = site_sources(tmp_path)
    result = run_stage1({"site_id": "fixture_site", "stage1": settings, "_config_dir": str(tmp_path)}, tmp_path / "output", seed=17)
    assert result["site_id"] == "fixture_site"
    assert result["geometry_built"] is False
    assert result["status"] == "geographic_inputs_prepared"
    assert result["building_count"] == 3  # two OSM + one conflict-free Overture
    assert result["road_way_count"] == 1
    assert Path(result["prepared_osm"]).is_file()
    assert len(result["provenance"]["overture_sha256"]) == 64
    report = json.loads((tmp_path / "output/geographic_reconciliation.json").read_text())
    assert report["success"] is True
    assert report["footprint_enrichment"]["added_overture_count"] == 1


def test_build_failure_retains_reusable_inputs_and_reports_the_failure(tmp_path):
    settings = site_sources(tmp_path)
    settings["mode"] = "build"
    with pytest.raises(RuntimeError, match="official CARLA 0.9.16"):
        run_stage1({"stage1": settings, "_config_dir": str(tmp_path)}, tmp_path / "output")
    manifest = json.loads((tmp_path / "output/manifest.json").read_text())
    assert manifest["status"] == "failed"
    assert manifest["geometry_built"] is False
    assert Path(manifest["prepared_osm"]).is_file()
    assert (tmp_path / "output/geographic_reconciliation.json").is_file()


def test_reject_ambiguous_osm_sources_and_osm_only_sigma():
    errors = validate_stage1_config({"map_name": "SigmaFixture", "osm_file": "a.osm", "osm_url": "https://example.org/a.osm"})
    assert any("exactly one" in error for error in errors)
    assert any("Overture" in error or "overture" in error for error in errors)


def test_unknown_stage1_options_and_ambiguous_boolean_are_rejected():
    errors = validate_stage1_config({"map_name": "SigmaFixture", "osm_file": "a.osm", "overture_geojson": "a.geojson", "flat_terrain": "false", "enable_prompt": True})
    assert "stage1.flat_terrain must be a YAML boolean" in errors
    assert "unknown stage1 option: enable_prompt" in errors


def test_configured_audit_routes_require_finite_wgs84_coordinates():
    errors = validate_stage1_config({"map_name": "SigmaFixture", "osm_file": "a.osm", "overture_geojson": "a.geojson", "map_audit_routes": {"r1": {"start_latitude": float("nan"), "start_longitude": 2, "end_latitude": 3, "end_longitude": 181}}})
    assert len([error for error in errors if "map_audit_routes" in error]) == 2
