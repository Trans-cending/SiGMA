"""Deterministic map resolver tests using asset-registry fixtures."""
#!/usr/bin/env python3
import tempfile
import unittest
from pathlib import Path

from sigma.map_export.tools.resolve_map_asset import MapResolutionError, resolve_map_asset


class ResolveMapAssetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.content = self.root / "Unreal" / "CarlaUE4" / "Content"

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def _touch(path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()

    def test_custom_map_and_tiles(self):
        base = self.content / "CustomMaps" / "MagicMap"
        self._touch(base / "MagicMap.umap")
        self._touch(base / "MagicMap_Tile_1_0.umap")
        self._touch(base / "MagicMap_Tile_0_0.umap")

        result = resolve_map_asset(self.root, "MagicMap")

        self.assertEqual(result["map_source"], "custom")
        self.assertEqual(
            result["map_package"], "/Game/CustomMaps/MagicMap/MagicMap"
        )
        self.assertEqual(
            result["expected_sublevels"],
            [
                "/Game/CustomMaps/MagicMap/MagicMap_Tile_0_0",
                "/Game/CustomMaps/MagicMap/MagicMap_Tile_1_0",
            ],
        )
        self.assertTrue(result["auto_discover_tile_sublevels"])

    def test_official_map_uses_exact_name(self):
        maps = self.content / "Carla" / "Maps"
        self._touch(maps / "Town05.umap")
        self._touch(maps / "Town05_Opt.umap")
        self._touch(maps / "Sublevels" / "Town05_Opt" / "T05_Ground.umap")

        result = resolve_map_asset(self.root, "Town05")

        self.assertEqual(result["map_source"], "official")
        self.assertEqual(result["map_package"], "/Game/Carla/Maps/Town05")
        self.assertEqual(result["expected_sublevels"], [])

    def test_official_optimized_map_loads_its_sublevels(self):
        maps = self.content / "Carla" / "Maps"
        self._touch(maps / "Town05_Opt.umap")
        sublevels = maps / "Sublevels" / "Town05_Opt"
        self._touch(sublevels / "T05_Ground.umap")
        self._touch(sublevels / "T05_Props.umap")

        result = resolve_map_asset(self.root, "Town05_Opt")

        self.assertEqual(result["map_package"], "/Game/Carla/Maps/Town05_Opt")
        self.assertEqual(
            result["expected_sublevels"],
            [
                "/Game/Carla/Maps/Sublevels/Town05_Opt/T05_Ground",
                "/Game/Carla/Maps/Sublevels/Town05_Opt/T05_Props",
            ],
        )
        self.assertFalse(result["auto_discover_tile_sublevels"])

    def test_ambiguous_auto_resolution_fails(self):
        self._touch(self.content / "CustomMaps" / "Shared" / "Shared.umap")
        self._touch(self.content / "Carla" / "Maps" / "Shared.umap")

        with self.assertRaises(MapResolutionError):
            resolve_map_asset(self.root, "Shared")
        self.assertEqual(
            resolve_map_asset(self.root, "Shared", "official")["map_source"],
            "official",
        )

    def test_missing_map_reports_failure(self):
        with self.assertRaises(MapResolutionError):
            resolve_map_asset(self.root, "Missing")


if __name__ == "__main__":
    unittest.main(verbosity=2)
