# Unreal map export for Stage III

The original `mosaic_ue_map_exporter_v1` has been reduced to its actual geometry export chain: map resolution, full streamed-world loading and placement, geometry selection and audit, standard Unreal `AssetExportTask` with `LevelExporterFBX`/`LevelExporterOBJ`, a report-based process supervisor, optional Blender curve-to-mesh conversion, and final report validation. The retained module provides an explicit configuration-driven export pipeline.

`run_export(config, output_dir)` in `sigma.map_export` accepts the complete deployment mapping, reads `stage3.map_export`, and returns a mapping containing the absolute `static_scene` path. The input configuration is preserved. The complete level-export implementation remains in `sigma/map_export/ue/export_complete_map.py`; its geometry placement and completeness rules are inherited from the audited source.

For a unified Stage III deployment, set:

```yaml
stage3:
  map_export:
    enabled: true
    map_source: custom
    format: fbx
    selection_mode: geometry
    rhi: vulkan
    minimum_output_size_bytes: 1048576
    meshify_curves: true
```

`map_name`, `carla_root`, and the editor location are inherited from `stage1` unless explicitly configured. The editor command binary defaults to `UE4Editor-Cmd` beside `stage1.unreal_editor`; it can be given with `unreal_editor_cmd`. Blender defaults to `stage3.blender.executable` or `blender` on `PATH`, and can be given with `blender_bin`. Paths resolve relative to the deployment YAML file.

Stage III supports the source FBX/Blender import path; keep `format: fbx` for automatic integration. Source OBJ export is available through the separate export CLI; Stage III static-scene integration accepts FBX and Blender inputs.

The standalone export CLI also accepts a small YAML document with a root `map_export` section. That standalone document follows the export CLI schema. `sigma validate` uses the complete site-deployment schema:

```bash
python -m sigma.map_export.cli \
  -c configs/town03_export.yaml \
  -o assets/local/town03
```

This creates `export_config.json`, `export_report.json`, supervisor/editor logs, the FBX or OBJ, an optional `blender_meshify_report.json`, `validation.log`, and a durable `manifest.json`. An existing export file is preserved and causes a clear error; select another output directory to repeat the operation. If Blender conversion fails after Unreal completes, the Unreal export and reports remain available.

## Actual configuration fields

| Field | Source capability |
|---|---|
| `enabled` | Explicitly authorizes this export operation; default false |
| `map_name`, `carla_root`, `unreal_editor_cmd` | Select the exact map and matching official CARLA/UE source environment |
| `map_source` | `auto`, `custom`, or `official`; ambiguous names require an explicit choice |
| `format` | `fbx` or `obj` using standard Unreal serializers |
| `selection_mode` | `geometry` selects all supported loaded geometric actors; `world` exports the persistent world |
| `rhi` | Source-supported `vulkan`, `opengl4`, or `nullrhi` |
| `timeout_seconds` | Export supervisor hard timeout; default 21600 |
| `minimum_loaded_level_count` | Defaults to one persistent level plus discovered sublevels |
| `minimum_actor_count`, `minimum_static_mesh_component_count` | Minimum completeness checks, each default 1 |
| `minimum_output_size_bytes` | Minimum output check; source default 1048576 |
| `meshify_curves` | Convert imported curve-like objects to mesh before downstream geometry use; default true for FBX |
| `blender_bin` | Executable used for the source Blender mesh conversion |
| `fbx` | Boolean settings: `ascii`, `collision`, `level_of_detail`, `vertex_color`, `force_front_x_axis` |

The source exporter needs an **uncooked source project** and source map levels in the target official CARLA 0.9.16/UE 4.26 asset tree. Provide the editor, matching plugins and source map levels separately from the packaged server. Rebuild/reimport custom maps for the official target project before export. Cook/Import and server loading are separate deployment steps.

Vulkan/OpenGL follows the original exporter's Xvfb path and requires `xvfb-run`, compatible graphics drivers, and a matching editor build. `nullrhi` is a source option for nonrendering exports; each deployment must pass geometry and streamed-level audits with its chosen assets. Blender mesh conversion needs a Blender version with the FBX importer/exporter used by the source. Real-map Unreal/Blender export remains pending acceptance.

The source report supervisor checks a fresh completed report, exact output identity, geometry-audit success, absence of fatal markers, and a nonempty export. The original source has compatibility handling for UE 4.26's missing actor-level accessors and streaming-level reflection; these adapters support geometry export through official Unreal APIs.

## Coordinates, materials, and actor meshes

The exporter preserves the original Unreal scene geometry and placement. The migrated Blender stage applies the original CARLA-to-radio transform and RF material canonicalization. Apply the Y-axis conversion once, in the Blender stage. Reuse the same rebuilt static source map, georeference, traffic snapshot identifiers, and actor library that generated the sensor records.

The dynamic-scene assembler uses an external `fbx_itu_grouped` actor-mesh library, indexed by `Blender_xml_storage.asset_manager` and configured through `stage3.blender.actor_assets`. The existing library contains vehicle, truck and human meshes; geometry/blueprint matching against the official server and redistribution rights require validation. A general actor-library exporter requires additional implementation. The map exporter produces the static scene.

## Tests and provenance

Five original map-resolution tests pass, covering exact optimized-map names, streamed sublevels, custom/official ambiguity, and missing maps. Three additional unit tests cover strict options, preserved failure reports/configuration, and the source supervisor's completed-geometry/output-identity requirements. These **8 unit tests** exercise configuration, reports and file identity. Real editor execution and physical FBX validation remain pending. The test-only FBX placeholder is used solely to exercise report/file-identity logic.

`map_export_sources.json` records the original and migrated hashes. The source exporter retains copyright notices and source attribution; its standalone redistribution grant requires rights-holder confirmation. Official Unreal/CARLA code and content remain separately obtained third-party dependencies under their own terms.
