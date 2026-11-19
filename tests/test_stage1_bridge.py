"""Check the migrated DTT editor patch against original source anchors."""
from pathlib import Path

from sigma.map_construction.install_bridge import install_bridge


def test_dtt_patch_uses_exact_original_anchors_and_never_installs_runtime_lights(tmp_path):
    plugin = tmp_path / 'Unreal/CarlaUE4/Plugins/CarlaTools/Source/CarlaTools'
    downloader = plugin / 'Private/Online/CustomFileDownloader.cpp'
    downloader.parent.mkdir(parents=True)
    downloader.write_text('#include "Misc/FileHelper.h"\n  osm2odr::OSM2ODRSettings Settings;\n  Settings.proj_string += " +lat_0=" + std::to_string(Lat_0) + " +lon_0=" + std::to_string(Lon_0);\n')
    odr = plugin / 'Private/OpenDriveToMap.cpp'
    odr.write_text('#include "Misc/FileHelper.h"\nfloat UOpenDriveToMap::GetHeight(float PosX, float PosY, bool bDrivingLane){\n}\n')
    streetmap = tmp_path / 'Unreal/CarlaUE4/Plugins/Streetmap/Source/StreetMapImporting/Private/StreetMapImportBlueprintLibrary.cpp'
    streetmap.parent.mkdir(parents=True)
    streetmap.write_text('  TArray<UObject*> NewAssets = AssetTools.ImportAssets(AssetPath, DestinationAssetPath, Factory, true );\n')
    install_bridge(tmp_path)
    assert 'MOSAIC_DTT_PROJECTION_ORIGIN_OVERRIDE' in downloader.read_text()
    assert 'MOSAIC_DTT_FLAT_TERRAIN' in odr.read_text()
    assert 'MOSAIC_STREETMAP_COMMANDLET_NO_BROWSER_SYNC' in streetmap.read_text()
    assert (plugin / 'Public/DigitalTwinHeadlessBridge.h').is_file()
    assert not (tmp_path / 'Unreal/CarlaUE4/Plugins/Carla').exists()
    before = (downloader.read_bytes(), odr.read_bytes(), streetmap.read_bytes())
    install_bridge(tmp_path)
    assert before == (downloader.read_bytes(), odr.read_bytes(), streetmap.read_bytes())
