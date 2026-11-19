"""Opt-in OSM normalization tests migrated from the map source."""
import xml.etree.ElementTree as ET

from sigma.map_construction.tools.normalize_osm_generation_features import (
    normalize,
    tags,
)


def test_explicit_generation_transforms_are_opt_in(tmp_path):
    source = tmp_path / "input.osm"
    source.write_text(
        """<?xml version='1.0' encoding='utf-8'?>
<osm version='0.6'>
 <bounds minlat='1' minlon='2' maxlat='3' maxlon='4'/>
 <way id='10'><tag k='leisure' v='pitch'/><tag k='surface' v='grass'/></way>
 <way id='11'><tag k='highway' v='service'/></way>
 <way id='12'><tag k='building' v='yes'/></way>
 <way id='13'><tag k='building' v='yes'/><tag k='height' v='8'/></way>
 <way id='14'><tag k='leisure' v='pitch'/></way>
</osm>
""",
        encoding="utf-8",
    )
    output = tmp_path / "output.osm"
    report = normalize(
        source,
        output,
        tmp_path / "report.json",
        promote_grass_pitches=True,
        grass_pitch_way_ids=("14",),
        promote_service_roads=True,
        missing_building_height_m=3.0,
    )
    ways = {way.attrib["id"]: tags(way) for way in ET.parse(output).getroot().findall("way")}
    assert ways["10"]["landuse"] == "grass"
    assert ways["14"]["landuse"] == "grass"
    assert ways["14"]["surface"] == "grass"
    assert ways["11"]["highway"] == "residential"
    assert ways["11"]["mosaic:original_highway"] == "service"
    assert ways["12"]["height"] == "3.00"
    assert ways["13"]["height"] == "8"
    assert report["grass_pitch_count"] == 2
    assert report["explicit_grass_pitch_ids"] == ["14"]
    assert report["promoted_service_road_count"] == 1
    assert report["default_building_height_count"] == 1
