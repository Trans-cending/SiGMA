"""Migrated geography tests; use pytest tests/test_stage1_geography.py."""
#!/usr/bin/env python3
import json
import struct
import zlib
from pathlib import Path

import pytest

pytest.importorskip("shapely")

from sigma.map_construction.tools import building_footprint_enrichment as footprints
from sigma.map_construction.tools import enrich_osm_building_heights as heights


def write_osm(path: Path) -> None:
    path.write_text(
        """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
 <bounds minlat="33.0" minlon="-112.0" maxlat="33.001" maxlon="-111.999"/>
 <node id="1" lat="33.0000" lon="-112.0000"/>
 <node id="2" lat="33.0000" lon="-111.9999"/>
 <node id="3" lat="33.0001" lon="-111.9999"/>
 <node id="4" lat="33.0001" lon="-112.0000"/>
 <node id="5" lat="33.0002" lon="-112.0000"/>
 <node id="6" lat="33.0002" lon="-111.9999"/>
 <node id="7" lat="33.0003" lon="-111.9999"/>
 <node id="8" lat="33.0003" lon="-112.0000"/>
 <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
  <tag k="building" v="yes"/><tag k="height" v="12"/></way>
 <way id="20"><nd ref="5"/><nd ref="6"/><nd ref="7"/><nd ref="8"/><nd ref="5"/>
  <tag k="building" v="yes"/></way>
 <way id="30"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
  <tag k="building" v="no"/></way>
</osm>
""",
        encoding="utf-8",
    )


def overture_feature(osm_id: str, height: float, coordinates):
    return {
        "type": "Feature",
        "id": f"gers-{osm_id}",
        "geometry": {"type": "Polygon", "coordinates": [coordinates]},
        "properties": {
            "height": height,
            "sources": [
                {"dataset": "OpenStreetMap", "record_id": f"{osm_id}@1"},
                {
                    "dataset": "USGS Lidar",
                    "property": "/properties/height",
                },
            ],
        },
    }


def test_existing_height_is_preserved_and_building_no_is_excluded(tmp_path: Path) -> None:
    osm = tmp_path / "input.osm"
    write_osm(osm)
    _, buildings, bbox = heights.load_osm(osm)
    assert [item["id"] for item in buildings] == ["w10", "w20"]

    center = ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)
    geojson = tmp_path / "overture.geojson"
    geojson.write_text(
        json.dumps(
            {
                "type": "FeatureCollection",
                "features": [
                    overture_feature(
                        "w10",
                        30.0,
                        [
                            [-112.0, 33.0],
                            [-111.9999, 33.0],
                            [-111.9999, 33.0001],
                            [-112.0, 33.0001],
                            [-112.0, 33.0],
                        ],
                    ),
                    overture_feature(
                        "w20",
                        9.5,
                        [
                            [-112.0, 33.0002],
                            [-111.9999, 33.0002],
                            [-111.9999, 33.0003],
                            [-112.0, 33.0003],
                            [-112.0, 33.0002],
                        ],
                    ),
                ],
            }
        ),
        encoding="utf-8",
    )
    matches = heights.match_overture(
        buildings, heights.load_overture(geojson, center)
    )
    assert matches["w10"]["candidate"]["height"] == 30.0
    assert matches["w20"]["candidate"]["height"] == 9.5
    assert buildings[0]["tags"]["height"] == "12"


def test_ghs_centroid_area_match_is_strict(tmp_path: Path) -> None:
    osm = tmp_path / "input.osm"
    write_osm(osm)
    _, buildings, bbox = heights.load_osm(osm)
    center = ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)
    target = next(item for item in buildings if item["id"] == "w20")
    centroid = target["geometry"].centroid
    rows = [
        {
            "id": "near",
            "lon": centroid.x,
            "lat": centroid.y,
            "height": 8.0,
            "area": target["projected"].area,
        },
        {
            "id": "wrong-area",
            "lon": centroid.x,
            "lat": centroid.y,
            "height": 40.0,
            "area": target["projected"].area * 4.0,
        },
    ]
    matches = heights.match_ghs(buildings, rows, {"w10"}, center)
    assert matches["w20"]["candidate"]["id"] == "near"


def test_measured_overture_height_outranks_osm_level_estimate() -> None:
    measured = {
        "candidate": {
            "height": 26.0,
            "height_source": "USGS Lidar",
        }
    }
    floor_estimate = {
        "candidate": {
            "height": 9.0,
            "height_source": "Overture num_floors",
        }
    }

    assert heights.has_measured_overture_height(measured)
    assert not heights.has_measured_overture_height(floor_estimate)
    assert not heights.has_measured_overture_height(None)







def test_conservative_footprint_merge_keeps_osm_context(tmp_path: Path) -> None:
    osm = tmp_path / "context.osm"
    osm.write_text(
        """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
 <bounds minlat="33.0" minlon="-112.0" maxlat="33.001" maxlon="-111.999"/>
 <node id="1" lat="33.00010" lon="-111.99990"/>
 <node id="2" lat="33.00010" lon="-111.99980"/>
 <node id="3" lat="33.00020" lon="-111.99980"/>
 <node id="4" lat="33.00020" lon="-111.99990"/>
 <node id="5" lat="33.00050" lon="-112.00000"/>
 <node id="6" lat="33.00050" lon="-111.99900"/>
 <node id="7" lat="33.00082" lon="-111.99985">
  <tag k="natural" v="tree"/>
 </node>
 <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
  <tag k="building" v="yes"/></way>
 <way id="20"><nd ref="5"/><nd ref="6"/>
  <tag k="highway" v="residential"/></way>
</osm>
""",
        encoding="utf-8",
    )
    tree, buildings, bbox = heights.load_osm(osm)
    center = ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)
    geojson = tmp_path / "overture.geojson"
    geojson.write_text(
        json.dumps({
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "id": "new-clean",
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[
                            [-111.99960, 33.00070],
                            [-111.99948, 33.00070],
                            [-111.99948, 33.00080],
                            [-111.99960, 33.00080],
                            [-111.99960, 33.00070],
                        ]],
                    },
                    "properties": {
                        "height": 8.0,
                        "subtype": "residential",
                        "sources": [{"dataset": "Microsoft ML Buildings"}],
                    },
                },
                {
                    "type": "Feature",
                    "id": "road-conflict",
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[
                            [-111.99960, 33.00047],
                            [-111.99948, 33.00047],
                            [-111.99948, 33.00053],
                            [-111.99960, 33.00053],
                            [-111.99960, 33.00047],
                        ]],
                    },
                    "properties": {
                        "height": 7.0,
                        "sources": [{"dataset": "Microsoft ML Buildings"}],
                    },
                },
            ],
        }),
        encoding="utf-8",
    )
    overture = heights.load_overture(geojson, center)
    output_dir = tmp_path / "topviews"
    result = footprints.enrich_overture_footprints(
        tree,
        buildings,
        overture,
        bbox,
        center,
        output_dir,
        "overture-conservative",
    )

    report = result["report"]
    assert report["accepted_overture_candidate_count"] == 1
    assert report["added_overture_count"] == 1
    assert report["osm_tree_point_count"] == 1
    assert report["overture_land_cover_layer_available"] is False
    assert report["rejected_reason_counts"]["road_conflict"] == 1
    assert len(result["added_buildings"]) == 1
    assert result["added_buildings"][0]["tags"]["building"] == "yes"
    assert (
        result["added_buildings"][0]["tags"]["mosaic:overture_subtype"]
        == "residential"
    )
    for filename in (
        "osm_source_topview.png",
        "overture_source_topview.png",
        "merged_topview.png",
    ):
        payload = (output_dir / filename).read_bytes()
        assert payload.startswith(b"\x89PNG\r\n\x1a\n")
        assert len(payload) > 1000

    merged_osm = tmp_path / "merged.osm"
    heights.write_osm(tree, merged_osm)
    _, merged_buildings, _ = heights.load_osm(merged_osm)
    assert len(merged_buildings) == 2


def _png_colors(path: Path):
    payload = path.read_bytes()
    offset = 8
    width = height = None
    compressed = bytearray()
    while offset < len(payload):
        length = struct.unpack(">I", payload[offset:offset + 4])[0]
        kind = payload[offset + 4:offset + 8]
        data = payload[offset + 8:offset + 8 + length]
        offset += 12 + length
        if kind == b"IHDR":
            width, height = struct.unpack(">II", data[:8])
        elif kind == b"IDAT":
            compressed.extend(data)
        elif kind == b"IEND":
            break
    rows = zlib.decompress(bytes(compressed))
    stride = 1 + 3 * width
    assert len(rows) == stride * height
    assert all(rows[row * stride] == 0 for row in range(height))
    return {
        tuple(rows[offset:offset + 3])
        for row in range(height)
        for offset in range(row * stride + 1, (row + 1) * stride, 3)
    }


def test_vegetation_merge_and_height_colors_are_auditable(
    tmp_path: Path,
) -> None:
    osm = tmp_path / "input.osm"
    write_osm(osm)
    tree, buildings, bbox = heights.load_osm(osm)
    center = ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)
    context = footprints._osm_context(tree, bbox, center)
    land_cover_path = tmp_path / "land_cover.geojson"
    land_cover_path.write_text(json.dumps({
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "id": "precise-grass",
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[
                        [-111.99975, 33.00065],
                        [-111.99955, 33.00065],
                        [-111.99955, 33.00082],
                        [-111.99975, 33.00082],
                        [-111.99975, 33.00065],
                    ]],
                },
                "properties": {
                    "subtype": "grass",
                    "cartography": {"max_zoom": 15},
                    "sources": [{"dataset": "ESA WorldCover 2020"}],
                },
            },
            {
                "type": "Feature",
                "id": "coarse-forest",
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[
                        [-111.99945, 33.00065],
                        [-111.99925, 33.00065],
                        [-111.99925, 33.00082],
                        [-111.99945, 33.00082],
                        [-111.99945, 33.00065],
                    ]],
                },
                "properties": {
                    "subtype": "forest",
                    "cartography": {"max_zoom": 7},
                    "sources": [{"dataset": "ESA WorldCover 2020"}],
                },
            },
        ],
    }), encoding="utf-8")
    land_cover = footprints.load_overture_land_cover(
        land_cover_path, center
    )
    output_dir = tmp_path / "topviews"
    result = footprints.enrich_overture_vegetation(
        tree,
        land_cover,
        buildings,
        context,
        center,
        output_dir,
        "overture-conservative",
    )

    assert result["report"]["high_resolution_vegetation_count"] == 1
    assert result["report"]["added_area_count"] == 1
    assert result["report"]["rejected_reason_counts"]["low_resolution"] == 1
    assert result["added"][0]["tags"]["landuse"] == "grass"
    assert (
        result["added"][0]["tags"]["mosaic:vegetation_method"]
        == "overture_land_cover_conservative"
    )

    images = footprints.render_topviews(
        output_dir,
        buildings,
        [],
        [],
        context,
        overture_vegetation=land_cover,
        accepted_vegetation=result["accepted"],
        height_enriched_osm_ids={"w20"},
    )
    osm_colors = _png_colors(Path(images["osm_source_topview"]))
    merged_colors = _png_colors(Path(images["merged_topview"]))
    assert footprints.HEIGHT_ENRICHED_BUILDING not in osm_colors
    assert footprints.HEIGHT_ENRICHED_BUILDING in merged_colors
    assert footprints.OVERTURE_VEGETATION in merged_colors




def test_latest_overture_release_uses_only_matching_bbox_cache(
    tmp_path: Path,
    monkeypatch,
) -> None:
    bbox = (-112.0, 33.0, -111.99, 33.01)
    release = "2026-08-19.0"
    cached = heights._overture_cache_path(bbox, release, tmp_path)
    cached.parent.mkdir(parents=True)
    cached.write_text("x" * 101, encoding="utf-8")
    land_cover_cached = heights._overture_cache_path(
        bbox, release, tmp_path, "land_cover"
    )
    assert land_cover_cached != cached
    assert land_cover_cached.parent.name == "land_cover"

    def fail_registry(*args, **kwargs):
        raise heights.subprocess.CalledProcessError(
            1,
            args[0],
            stderr="registry unavailable",
        )

    monkeypatch.setattr(heights.subprocess, "run", fail_registry)
    assert heights.resolve_overture_release(
        Path("/missing/overturemaps"),
        "latest",
        bbox=bbox,
        cache_root=tmp_path,
    ) == release

    with pytest.raises(RuntimeError, match="no exact-bbox"):
        heights.resolve_overture_release(
            Path("/missing/overturemaps"),
            "latest",
            bbox=(-110.0, 30.0, -109.0, 31.0),
            cache_root=tmp_path,
        )


def test_prepare_ghs_prefers_region_cache(
    tmp_path: Path,
    monkeypatch,
) -> None:
    csv_path = (
        tmp_path
        / "ghs_obat"
        / heights.GHS_RELEASE
        / "USA"
        / "GHS_OBAT_CSV_USA_E2020_R2024A_V1_0_Arizona.csv"
    )
    csv_path.parent.mkdir(parents=True)
    csv_path.write_text(
        "id,lon,lat,height,area\n"
        "cached,-111.995,33.005,10,100\n",
        encoding="utf-8",
    )

    def forbid_network(*args, **kwargs):
        raise AssertionError("network must not be used when region cache exists")

    monkeypatch.setattr(heights, "find_ghs_member", forbid_network)
    resolved, rows, member = heights.prepare_ghs(
        "USA",
        "Arizona",
        (-112.0, 33.0, -111.99, 33.01),
        tmp_path,
        None,
    )
    assert resolved == csv_path
    assert member == csv_path.name
    assert [row["id"] for row in rows] == ["cached"]
