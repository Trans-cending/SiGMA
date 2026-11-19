# Migration audit

The user-specified `isac_sim2real_agent` project is the migration baseline.
The WCNC SiGMA paper defines the data-generation scope. The source checkout
was audited read only; `source_snapshot.json` records 781 source, configuration
and document checksums for the unchanged-source comparison.

| Original component | Independent target | Treatment |
| --- | --- | --- |
| `mosaic_digital_twin_release_v101` | `sigma/stage1.py`, `sigma/map_construction` | Retain geographic/DTT/OpenDRIVE/semantic source; provide explicit Python driver |
| `mosaic_ue_map_exporter_v1` | `sigma/map_export` | Retain static geometry, streaming and export audits; provide explicit process supervisor |
| platform `CARLA_sensor_storage` | `sigma/sensing` | Extract road, FoV, projection, frame and storage algorithms; implement official synchronous runtime |
| platform `mosaic_core` | `mosaic_core` | Retain numerical, material and wire-schema helpers; package local RF registry |
| platform Blender modules | `Blender_xml_storage` | Retain material, mesh and XML conversion and actor transforms |
| platform Sionna modules | `Sionna_channel_storage` | Retain paths, CSI and RIS; add physical explicit-array adapter, Radio Map export and frame identity |
| platform Radar modules | `Radar_channel_storage`, two pure `CARLA_sensor_storage/radar` files | Retain isolated RCS backend, waveform profiles and IQ storage |
| platform `CSV_data_export` | `CSV_data_export` | Retain deterministic dataset indexing with explicit dataset root |
| deterministic experimental parameters and RIS mathematics | `experiments` | Retain parameters and add numerical metrics, label and group-export adapters |

The audit excluded `mosaic_agent`, control/state/prompt/provider/UI modules,
Qwen/LLM dependencies, MoE/few-shot/adaptation workloads, viewers, UAV/AirSim
runtime, debug monoliths, historical job scripts and patches, environments,
caches, datasets, weights, experiment outputs and logs. The retained native
extension is the CLI-to-DTT editor bridge with map-alignment patches described
in [stage1.md](stage1.md).

The runtime packages are local to this project. `mosaic_core`, `MOSAIC_*` native
keys, `mosaic:*` geographic tags and stage-first storage names remain internal
protocol identifiers. Interpreters, server roots and assets are configured as
explicit external dependencies.

File-level hashes and migration classifications are in `stage1_sources.json`,
`stage2_sources.json`, `map_export_sources.json` and `provenance_stage3.json`.
Packaging is defined in `pyproject.toml`; runtime assets are obtained separately.
The read-only source-integrity comparison is recorded in `source_integrity.json`.

Current external requirements include compatible actor assets, official
Cook/Import tooling and map/export redistribution grants. General actor-library
export and automatic Cook/Import require additional implementation. Experimental phase codebooks are explicit optional inputs for label export. Authorization details appear in
[THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md).
