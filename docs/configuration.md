# Deployment configurations

The repository supplies two deployments: official CARLA Town03 and the
geographic RIS site reconstructed as `SigmaRIS`. YAML paths resolve relative
to the configuration file. The source files, SHA256 values and field mapping
are recorded in [config_sources.json](config_sources.json).

| File | Entry point | Role |
| --- | --- | --- |
| `configs/town03.yaml` | `python -m sigma` | Town03 Stage II sensing and Stage III channels/Radar |
| `configs/ris.yaml` | `python -m sigma` | RIS Stage I construction, Stage II camera/state capture and Stage III channels |
| `configs/town03_export.yaml` | `python -m sigma.map_export.cli` | Official Town03 source-map export |
| `configs/ris_export.yaml` | `python -m sigma.map_export.cli` | Rebuilt SigmaRIS source-map export |
| `configs/rf_materials.yaml` | Shared material registry | Dielectric and Radar scattering properties |

## Source parameter mapping

Town03 parameters come from
`mosaic_platform_v101/CARLA_sensor_storage/config_traj_backup/config_trajectory_Town03.yaml`.
Its simulation uses seed 42, 10 Hz and 4 seconds. The independent preset records
one communicating vehicle and 29 background vehicles. One capture corresponds
to one `trajectory_id`; each additional capture uses a fresh integer ID.

The source defines four RSU candidates and selects one per run. The preset
uses candidate 0, `[0,0,15]` m, yaw 180 degrees. The other candidates are
`[116,-107,15]`/yaw 0, `[-126,19,16]`/yaw 90 and `[192,22,12]`/yaw 0.
RSU height is absolute in native Town03 coordinates.

`route.selection: seeded_spawn_points` uses the source traffic-spawn approach:
seeded selection from the official map's Driving spawn points. The existing
forward-road planner validates candidate routes, with a bounded
`max_route_candidates: 64`. `route.json` records the selected spawn indices,
seed, endpoints and actual road waypoints. `route.selection: explicit` uses
`start`, `destination` and ordered `via` points, as in the RIS deployment.

Town03 sensors preserve the active source modalities and mount locations:

- CAV: front RGB, Depth and semantics, 2D/3D LiDAR, GNSS and IMU.
- RSU: four-direction RGB, Depth and semantics, 2D/3D LiDAR, GNSS and IMU.
- Each node also has a virtual Sionna RCS Radar calibration entry.

Camera images use 1280×720 pixels and 90-degree FoV. `every_n_samples: 1`
translates the source frame-based cadence to the synchronous acquisition
schema. Rotation fields use explicit `pitch`, `yaw` and `roll`. Named side
cameras follow the canonical source `multimodal_schema.py` yaw directions.
LiDAR rotation follows the 10 Hz world step.

The two Radar entries use the `sionna_rcs_radar_item` from
`mosaic_platform_v101/examples/ris_sionna_rcs_overlay.yaml`, combined with each
Town03 node's original mount and range. The source profile specifies a 77 GHz
carrier, four receive ports, 256 ADC samples and 128 chirps. Town03 Stage III
Radar processing is enabled. Communication uses the source `h001` defaults:
28 GHz, 1 GHz bandwidth, 256 subcarriers, 8×8 transmit and 4×4 receive arrays,
encoded as `arrays: [[4,4,8,8]]`.

## RIS site construction

Geographic build parameters come from
`mosaic_digital_twin_release_v101/runs/Mosaic_D2D_Scenario_wdtlights/pipeline_config.json`.
The site center is `33.4182083333, -111.9261027778`, with a 500 m radius and
this source bounding box in west/south/east/north order:

```text
[-111.93148399477475, 33.413716777425044,
 -111.92072156082524, 33.42269988917496]
```

Prepare input files for this region and configure the external build paths:

```text
assets/local/
  CARLA_0.9.16_source/
  UnrealEngine_4.26/
  actors/
  ris/
    site.osm
    buildings.geojson
    land_cover.geojson
  town03/
    Town03.fbx
```

`site.osm` is the raw OSM input. `buildings.geojson` and `land_cover.geojson`
contain Overture building and land-cover features for the same region; retain
their release and attribution alongside the inputs. Height enrichment uses
`overture-ghs`, with GHS country `USA` and region `Arizona`. The source height
tool acquires GHS data; `stage1.ghs_csv` can select a local table. Vegetation
supplementation uses `overture-conservative`.

Stage I builds `SigmaRIS`. The sensing and export configurations use the same
name. The generated OpenDRIVE georeference provides the shared reference for
geographic routes, RSU/RIS placement and later scene conversion.

```bash
python -m sigma stage1 -c configs/ris.yaml -o outputs/ris
```

Cook and import the map with the selected official CARLA 0.9.16 source and
content. [stage1.md](stage1.md) describes editor/bridge preparation and
[carla.md](carla.md) describes target-version import and runtime checks.

## RIS deployment

Panel, RSU, camera and route values come from
`mosaic_agent/experiments/ris_camera_phase_optimization.json`. The deterministic
parameters are represented directly in `configs/ris.yaml`.

| Setting | Value |
| --- | --- |
| RSU latitude / longitude | `33.41920874636455, -111.92717732926977` |
| RSU height / yaw | 2 m above the nearest Driving-road elevation / 0 degrees |
| RIS latitude / longitude | `33.417827, -111.926134` |
| RIS height / yaw | 2 m above the nearest Driving-road elevation / 180 degrees |
| Panel | 32×32, half-wavelength spacing, 0.02 m thickness |
| Channel | 60 GHz, 100 MHz bandwidth, one subcarrier, 16 RSU ports, one UE port |
| Phase configuration | `effective`, `zero`, amplitude 0.8, full components and paths |
| Camera | RIS front RGB, 960×540, 90-degree FoV, pitch -10 degrees |
| Simulation | Seed 1477, 10 Hz, 150-frame bound, one CAV and 24 background vehicles |
| Acquisition | RIS-camera FoV entry and exit; camera plus synchronized world state |

The capture window and vehicle count follow the saved RIS sensor metadata.
Seed 1477 identifies the first saved source trajectory. Speed is fixed at
30 km/h, an explicit deterministic value within the source 25–35 km/h range.
The default southbound route is:

```text
start:       33.418616405888145, -111.92623489482054
destination: 33.416952154879134, -111.92624334349155
```

The alternative northbound route is recorded in `stage1.map_audit_routes`:

```text
start:       33.41700857068986, -111.92638697089876
destination: 33.418799753624995, -111.92639541956976
```

To collect northbound traffic, set these coordinates in `stage2.route`, assign
a fresh `trajectory_id`, then run Stage II. Both routes use the site's
south-positive tmerc convention. Town03 uses native CARLA geolocation.

## Static scenes and environments

Export each matching source map to the static input selected by its deployment:

```bash
python -m sigma.map_export.cli -c configs/town03_export.yaml -o assets/local/town03
python -m sigma.map_export.cli -c configs/ris_export.yaml -o assets/local/ris
```

The resulting paths are `assets/local/town03/Town03.fbx` and
`assets/local/ris/SigmaRIS.fbx`. `stage3.blender.actor_assets` selects
`assets/local/actors`. Both map and actor geometry must match the server assets
used for sensing.

Backend interpreter paths select `.venv-communication/bin/python` and
`.venv-radar/bin/python`. The main `.venv` supplies official CARLA 0.9.16 for
map conversion and sensing. Configure Blender and its matching Mitsuba exporter
as described in the bilingual README.

The constructed geographic smoke configuration lives beside its inputs at
`tests/fixtures/stage1/geography.yaml`:

```bash
python -m sigma stage1 -c tests/fixtures/stage1/geography.yaml -o /tmp/sigma-geography-smoke
```
