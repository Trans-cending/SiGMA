"""Install only the migrated DTT editor bridge in an explicitly selected checkout.

Run: python -m sigma.map_construction.install_bridge --carla-root /path/to/CARLA
This explicit setup command changes the selected official CARLA source checkout,
creates backups, and requires an editor rebuild. The installation scope is the
DTT editor bridge in that selected checkout.
"""
from __future__ import annotations
import argparse
import shutil
from datetime import datetime
from pathlib import Path

def install_bridge(carla_root: Path) -> None:
    CARLA_ROOT = carla_root.expanduser().resolve()
    PROJECT_ROOT = Path(__file__).resolve().parent
    CARLATOOLS_PLUGIN = CARLA_ROOT / 'Unreal' / 'CarlaUE4' / 'Plugins' / 'CarlaTools' / 'Source' / 'CarlaTools'
    PLUGIN = CARLATOOLS_PLUGIN
    source_copies = [(PROJECT_ROOT / 'bridge/Public/DigitalTwinHeadlessBridge.h', CARLATOOLS_PLUGIN / 'Public/DigitalTwinHeadlessBridge.h'), (PROJECT_ROOT / 'bridge/Private/DigitalTwinHeadlessBridge.cpp', CARLATOOLS_PLUGIN / 'Private/DigitalTwinHeadlessBridge.cpp')]
    downloader_source = PLUGIN / 'Private' / 'Online' / 'CustomFileDownloader.cpp'
    open_drive_to_map_source = PLUGIN / 'Private' / 'OpenDriveToMap.cpp'
    streetmap_import_source = CARLA_ROOT / 'Unreal' / 'CarlaUE4' / 'Plugins' / 'Streetmap' / 'Source' / 'StreetMapImporting' / 'Private' / 'StreetMapImportBlueprintLibrary.cpp'
    projection_include_anchor = '#include "Misc/FileHelper.h"\n'
    projection_include = '#include "HAL/PlatformMisc.h"\n'
    projection_anchor = '  osm2odr::OSM2ODRSettings Settings;\n  Settings.proj_string += " +lat_0=" + std::to_string(Lat_0) + " +lon_0=" + std::to_string(Lon_0);\n'
    projection_override = '  static const char MosaicProjectionOriginBuildMarker[] =\n      "MOSAIC_DTT_PROJECTION_ORIGIN_OVERRIDE";\n  const FString MosaicOriginLatitude = FPlatformMisc::GetEnvironmentVariable(\n      TEXT("MOSAIC_XODR_ORIGIN_LAT"));\n  const FString MosaicOriginLongitude = FPlatformMisc::GetEnvironmentVariable(\n      TEXT("MOSAIC_XODR_ORIGIN_LON"));\n  if (MosaicOriginLatitude.IsEmpty() != MosaicOriginLongitude.IsEmpty())\n  {\n    UE_LOG(LogCarlaToolsMapGenerator, Error,\n        TEXT("%s requires both MOSAIC_XODR_ORIGIN_LAT and MOSAIC_XODR_ORIGIN_LON"),\n        UTF8_TO_TCHAR(MosaicProjectionOriginBuildMarker));\n    return;\n  }\n  if (!MosaicOriginLatitude.IsEmpty())\n  {\n    Lat_0 = FCString::Atof(*MosaicOriginLatitude);\n    Lon_0 = FCString::Atof(*MosaicOriginLongitude);\n    if (!FMath::IsFinite(Lat_0) || !FMath::IsFinite(Lon_0) ||\n        FMath::Abs(Lat_0) > 90.0f || FMath::Abs(Lon_0) > 180.0f)\n    {\n      UE_LOG(LogCarlaToolsMapGenerator, Error,\n          TEXT("%s rejected invalid origin lat=%.9f lon=%.9f"),\n          UTF8_TO_TCHAR(MosaicProjectionOriginBuildMarker), Lat_0, Lon_0);\n      return;\n    }\n    UE_LOG(LogCarlaToolsMapGenerator, Warning,\n        TEXT("%s lat=%.9f lon=%.9f"),\n        UTF8_TO_TCHAR(MosaicProjectionOriginBuildMarker), Lat_0, Lon_0);\n  }\n\n  osm2odr::OSM2ODRSettings Settings;\n  Settings.proj_string += " +lat_0=" + std::to_string(Lat_0) + " +lon_0=" + std::to_string(Lon_0);\n'
    streetmap_import_anchor = '  TArray<UObject*> NewAssets = AssetTools.ImportAssets(AssetPath, DestinationAssetPath, Factory, true );\n'
    streetmap_import_override = '  static const char MosaicStreetMapCommandletBuildMarker[] =\n      "MOSAIC_STREETMAP_COMMANDLET_NO_BROWSER_SYNC";\n  const bool bSyncToBrowser = !IsRunningCommandlet();\n  if (!bSyncToBrowser)\n  {\n    UE_LOG(LogStreetMapImporting, Log,\n        TEXT("%s importing without Content Browser sync"),\n        UTF8_TO_TCHAR(MosaicStreetMapCommandletBuildMarker));\n  }\n  TArray<UObject*> NewAssets = AssetTools.ImportAssets(\n      AssetPath, DestinationAssetPath, Factory, bSyncToBrowser);\n'
    flat_terrain_include_anchor = '#include "Misc/FileHelper.h"\n'
    flat_terrain_include = '#include "HAL/PlatformMisc.h"\n'
    flat_terrain_anchor = 'float UOpenDriveToMap::GetHeight(float PosX, float PosY, bool bDrivingLane){\n'
    flat_terrain_override = 'float UOpenDriveToMap::GetHeight(float PosX, float PosY, bool bDrivingLane){\n  static const char MosaicFlatTerrainBuildMarker[] =\n      "MOSAIC_DTT_FLAT_TERRAIN";\n  static const FString MosaicFlatTerrainValue =\n      FPlatformMisc::GetEnvironmentVariable(\n          UTF8_TO_TCHAR(MosaicFlatTerrainBuildMarker));\n  static const bool bMosaicFlatTerrain =\n      MosaicFlatTerrainValue.Equals(TEXT("1")) ||\n      MosaicFlatTerrainValue.Equals(TEXT("true"), ESearchCase::IgnoreCase) ||\n      MosaicFlatTerrainValue.Equals(TEXT("yes"), ESearchCase::IgnoreCase) ||\n      MosaicFlatTerrainValue.Equals(TEXT("on"), ESearchCase::IgnoreCase);\n  if (bMosaicFlatTerrain)\n  {\n    return 0.0f;\n  }\n'
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    for source, destination in source_copies:
        if not source.is_file():
            raise SystemExit(f'missing source: {source}')
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            if destination.read_bytes() == source.read_bytes():
                print(f'UNCHANGED={destination}')
                continue
            backup = destination.with_name(destination.name + f'.before_mosaic_release_{stamp}.bak')
            shutil.copy2(destination, backup)
            print(f'BACKUP={backup}')
        shutil.copy2(source, destination)
        print(f'INSTALLED={destination}')
    if not downloader_source.is_file():
        raise SystemExit(f'missing CarlaTools source: {downloader_source}')
    downloader_text = downloader_source.read_text(encoding='utf-8')
    if 'MOSAIC_DTT_PROJECTION_ORIGIN_OVERRIDE' in downloader_text:
        print(f'UNCHANGED={downloader_source}')
    else:
        if downloader_text.count(projection_include_anchor) != 1:
            raise SystemExit('unexpected CustomFileDownloader include layout; refusing to patch')
        if downloader_text.count(projection_anchor) != 1:
            raise SystemExit('unexpected ConvertOSMInOpenDrive layout; refusing to patch')
        patched_text = downloader_text.replace(projection_include_anchor, projection_include_anchor + projection_include, 1).replace(projection_anchor, projection_override, 1)
        backup = downloader_source.with_name(downloader_source.name + f'.before_mosaic_projection_{stamp}.bak')
        shutil.copy2(downloader_source, backup)
        downloader_source.write_text(patched_text, encoding='utf-8')
        print(f'BACKUP={backup}')
        print(f'PATCHED={downloader_source}')
    if not open_drive_to_map_source.is_file():
        raise SystemExit(f'missing CarlaTools source: {open_drive_to_map_source}')
    open_drive_text = open_drive_to_map_source.read_text(encoding='utf-8')
    if 'MOSAIC_DTT_FLAT_TERRAIN' in open_drive_text:
        print(f'UNCHANGED={open_drive_to_map_source}')
    else:
        if open_drive_text.count(flat_terrain_include_anchor) != 1:
            raise SystemExit('unexpected OpenDriveToMap include layout; refusing to patch')
        if open_drive_text.count(flat_terrain_anchor) != 1:
            raise SystemExit('unexpected OpenDriveToMap::GetHeight layout; refusing to patch')
        patched_text = open_drive_text.replace(flat_terrain_include_anchor, flat_terrain_include_anchor + flat_terrain_include, 1).replace(flat_terrain_anchor, flat_terrain_override, 1)
        backup = open_drive_to_map_source.with_name(open_drive_to_map_source.name + f'.before_mosaic_flat_terrain_{stamp}.bak')
        shutil.copy2(open_drive_to_map_source, backup)
        open_drive_to_map_source.write_text(patched_text, encoding='utf-8')
        print(f'BACKUP={backup}')
        print(f'PATCHED={open_drive_to_map_source}')
    if not streetmap_import_source.is_file():
        raise SystemExit(f'missing StreetMap source: {streetmap_import_source}')
    streetmap_text = streetmap_import_source.read_text(encoding='utf-8')
    if 'MOSAIC_STREETMAP_COMMANDLET_NO_BROWSER_SYNC' in streetmap_text:
        print(f'UNCHANGED={streetmap_import_source}')
    else:
        if streetmap_text.count(streetmap_import_anchor) != 1:
            raise SystemExit('unexpected StreetMap import layout; refusing to patch')
        patched_text = streetmap_text.replace(streetmap_import_anchor, streetmap_import_override, 1)
        backup = streetmap_import_source.with_name(streetmap_import_source.name + f'.before_mosaic_commandlet_{stamp}.bak')
        shutil.copy2(streetmap_import_source, backup)
        streetmap_import_source.write_text(patched_text, encoding='utf-8')
        print(f'BACKUP={backup}')
        print(f'PATCHED={streetmap_import_source}')
    print('DTT_HEADLESS_BRIDGE_INSTALLED=1')
    print('DTT_PROJECTION_ORIGIN_OVERRIDE_INSTALLED=1')
    print('DTT_FLAT_TERRAIN_SWITCH_INSTALLED=1')
    print('STREETMAP_COMMANDLET_IMPORT_FIX_INSTALLED=1')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--carla-root', required=True, type=Path)
    args = parser.parse_args()
    install_bridge(args.carla_root)
if __name__ == '__main__':
    main()
