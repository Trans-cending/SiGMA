# Stage I: site-grounded map construction

`stage1` is a deterministic Python driver for the migrated geographic and Unreal map code. The source modules, original hashes, and migration paths are listed in `stage1_sources.json`.

## What runs

`mode: prepare` reads or downloads OSM XML, sanitizes names that would produce invalid Unreal asset paths, preserves OSM roads/buildings, loads Overture building candidates, projects both sources into a common local reference, and adds only conflict-free missing footprints. Existing OSM footprints and valid heights retain precedence. Screening includes overlap/proximity to existing buildings, road corridors, water, minimum area, and geographic boundaries. Optional Overture/GHS height enrichment and Overture land-cover reconciliation use the migrated implementations. This mode writes reusable geographic inputs. Use `build` for 3D construction and the post-Cook runtime audit for road connectivity.

`mode: build` also performs official CARLA OSM2ODR conversion, repairs invalid references and isolated lane artifacts, checks the dominant driving-network component, retains projection metadata, invokes CARLA's Digital Twin Tool to construct the textured urban scene, checks generated geometry, removes road/building vegetation conflicts, then retags and verifies CARLA semantic asset roots. The migrated DTT automation preserves procedural buildings/facades, roads, vegetation and terrain through the external official CarlaTools plugin and its content. A source build, external Unreal assets, textures, content and matching plugins are required.

A built source map needs cooking for the target official CARLA server. `geometry_built: true` and `cook_required: true` record source-map completion and the next required step. Server loading, traffic operation and sensor rendering are validated after Cook/Import.

## Configuration

Paths in YAML are resolved against the YAML file's parent directory. `overture_cli` may be an executable on `PATH`; a filesystem executable path is resolved against the YAML directory. Use a pinned Overture release for downloaded, repeatable data. Local GeoJSON input defaults to provenance label `local` and is hashed in the output manifest.

```yaml
site_id: ris
stage1:
  map_name: SigmaRIS
  mode: prepare
  osm_file: data/ris.osm
  overture_geojson: data/ris_buildings.geojson
  height_mode: overture
  vegetation_mode: none
```

Use exactly one of `osm_file` or `osm_url`. SiGMA requires an Overture source (`overture_geojson` or `overture_cli`). To use Overture land cover, set `vegetation_mode: overture-conservative` and supply `overture_land_cover_geojson` or a working Overture download CLI. OSM vegetation is preserved with either setting. Optional `height_mode: overture-ghs` additionally needs `ghs_country` and `ghs_admin1`, with `ghs_csv` available for a local source.

Additional build settings are:

```yaml
stage1:
  # Include the geographic settings above.
  mode: build
  carla_root: /path/to/dedicated/CARLA_0.9.16_source
  unreal_editor: /path/to/UnrealEngine_4.26/Engine/Binaries/Linux/UE4Editor
  carla_python: /path/to/carla0916_environment/bin/python
  xodr_projection_margin_m: 25.0
  flat_terrain: true
  timeout_seconds: 14400
```

`max_dtt_tiles` optionally limits the number of 500 m reconstruction tiles. Optional `map_audit_routes` entries use `start_latitude`, `start_longitude`, `end_latitude`, and `end_longitude`; they are checked by the runtime audit after the map is cooked and loaded. `promote_grass_pitches`, `grass_pitch_way_ids`, and `missing_building_height_m` expose existing source normalization rules. Road class promotion is disabled in the public driver.

Run a configured stage with `python -m sigma stage1 -c configs/ris.yaml -o outputs/site --seed 42`. The Python API is `sigma.stage1.run_stage1(config, output_dir, seed)`, where `config` is the complete deployment mapping and `_config_dir` is the YAML parent supplied by the unified CLI. `validate_stage1_config(stage1_mapping)` checks configuration structure independently of CARLA or Unreal. The supplied seed is recorded. Exposing seed control in the external DTT generator and verifying bitwise repeatability remain external build requirements.

## Prepare the official CARLA build

Obtain the official CARLA 0.9.16 source, its matching UE 4.26 branch, and the official content separately. The project needs CarlaTools, PythonScriptPlugin, EditorScriptingUtilities, and the StreetMap import plugin used by DTT. External checkout setup and bridge installation are explicit preparation steps.

The migrated bridge source and guarded installer are provided for a dedicated build:

```bash
python -m sigma.map_construction.install_bridge --carla-root /path/to/dedicated/CARLA_0.9.16_source
```

This explicit setup command copies the DTT editor bridge into **only the selected checkout** and applies the original projection-origin/flat-terrain/StreetMap commandlet fixes, and creates timestamped backups of modified files. Its patch anchors are checked against the source layout. Recompile that checkout's CarlaTools editor plugin using the official source-build procedure. Match the compiler, UE build and server build used for cooking. Installer tests used isolated source fixtures. The existing compiled bridge is present; reproducing native compilation on the deployment checkout remains pending.

The following project extensions support editor automation alongside the standard CARLA Python API:

| Extension | Why it remains | Official CARLA adaptation |
|---|---|---|
| `unreal.DigitalTwinHeadlessBridge` | Automates DTT's internal Generate/Save implementation from the CLI | Compile the supplied source into CarlaTools for CLI-driven generation |
| Projection-origin override | Aligns DTT geometry with the prepared OpenDRIVE origin | Install the guarded CustomFileDownloader patch and rebuild |
| Flat-terrain switch | Preserves the source terrain setting | Install the guarded OpenDriveToMap patch and rebuild |
| StreetMap commandlet fix | Avoids Content Browser synchronization during noninteractive imports | Install the guarded StreetMap patch and rebuild |

The retained `MOSAIC_*` environment keys and `mosaic:*` geographic attribution tags are internal data/Unreal compilation protocols. Keeping these keys preserves generated asset references.

The build invokes an unattended editor process. DTT internally uses a widget implementation, so its generation pass still needs working graphics/offscreen rendering. Geometry and semantic passes use NullRHI and the original report-based subprocess supervisor. Success requires a completed saved report, a clean process log and the source supervisor's process-lifecycle checks.

Rebuild geographic inputs with the target official source, plugins and content. Retag and verify the new source map, then Cook/Import for that exact server. Use the official CARLA map/package procedure for the selected installation.

After cooking and loading the map in the target server, the retained audit command checks road reachability and geographic/mesh alignment:

```bash
python -m sigma.map_construction.tools.audit_carla_map_alignment \
  --config outputs/site/stage1/map_build_config.json \
  --output-dir outputs/site/stage1/runtime_map_audit \
  --host 127.0.0.1 --port 2000
```

Use the same matching official CARLA Python API as the server. A clean source wheel may expose the official release commit `294096e...` as its client version. This form requires package metadata `carla==0.9.16` and a purely hexadecimal clean commit string. Stage I checks the installed client; Stage II connects and checks matching client/server versions before changing the world. The verified clean environment is listed in [remaining_work.md](remaining_work.md).

## Outputs and failure handling

```text
stage1/
  manifest.json
  input/source.osm
  input/sanitized.osm
  input/reconciled.osm
  input/generation.osm
  sanitize.json
  geographic_reconciliation.json
  normalization.json
  source_topviews/                 # candidate/rejection provenance and overlays
  map_build_config.json            # build mode
  input/<map>.prepared.xodr        # build mode
  opendrive_prepare.json
  opendrive_repair.json
  map_audit/opendrive_static_audit.json
  driving_lane_samples.json
  generate_save.json
  geometry_checks/
  semantic_retag.json
  semantic_verify.json
  map.xodr
```

Unreal source assets live under the explicitly selected CARLA checkout's `Content/CustomMaps/<map_name>` during a build. This external build output belongs to the selected deployment checkout. Each build requires a fresh map directory. Failures save `status: failed`, the error type/message, and all completed local intermediates. Geographic preparation and numerical processing remain independently reusable.

The manifest contains site/map identity, seed, source hashes, bounding box, counts, prepared input path, and geometry status. The manifest also records pending checks, including route reachability and rendered semantic validation. The build additionally records the OpenDRIVE path, georeference, source map directory, level packages, and cooking requirement. OSM/Overture source attribution remains in reconciled XML and source-screening reports. Height-enrichment metadata records the upstream source. RF material assignment occurs in the preserved Stage III exporter; semantic folders written here supply the required classification.

## Validation performed

The migrated conservative building/vegetation/height tests, normalization test, and new local-pipeline integration tests pass in the available Python 3.10 geography environment: **14 tests passed**. Tests exercise OSM precedence, OSM building-tag filtering, geometric conflict screening, source-height rules, vegetation provenance, matching-bbox source cache selection, local OSM–Overture integration, relative paths, failure retention and configuration contracts.

The integration input is a small constructed fixture derived from the source
test. Real geographic downloads, official OSM2ODR, native build reproduction,
DTT generation, Cook/Import, runtime reachability and rendered semantics remain
live acceptance checks. An isolated source fixture verifies DTT patch anchors
and repeatable installation. Deployment resource and GPU evidence are listed in
[remaining_work.md](remaining_work.md).
