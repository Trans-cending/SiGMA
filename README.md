# SiGMA

[English](README.md) | [简体中文](README_CN.md)

**Site-Grounded Multi-Modal ISAC Data Generation for Environment-Aware Communications**

SiGMA is a command-line framework for site reconstruction, synchronized
multimodal sensing, and wireless/RIS/Radar data generation. It implements the
three-stage data-generation workflow described in the IEEE WCNC 2027 paper
“SiGMA: Site-Grounded Multi-Modal ISAC Data Generation for Environment-Aware
Communications”. YAML configurations specify the site, trajectory, sensors,
propagation settings and output location.

## Architecture

| Stage | Capabilities | Products |
| --- | --- | --- |
| I · Site-grounded map construction | OSM roads and primary building footprints; screened Overture supplements; shared projection; OpenDRIVE; DTT procedural buildings, facades, textures, vegetation and terrain; geographic, road and semantic checks | Prepared geography, build reports, XODR and source map assets for Cook/Import |
| II · Multimodal sensing acquisition | Road-constrained trajectories, ordered waypoints, background traffic, FoV rules and synchronous CARLA acquisition | RGB, metric depth, LiDAR, GNSS/IMU, vehicle state, calibration and frame indexes |
| III · Wireless and Radar generation | UE–Blender–Sionna conversion, RF materials, frozen dynamic scenes, paths/CSI/Radio Map, synthetic/explicit arrays, RIS components and RCS Radar | Scene XML/meshes, complex channels, path metadata, CIR and FMCW IQ |

Stage II and III use the same saved scene state. Shared site, trajectory, frame
and simulation-time identifiers connect the modalities. Source lineage and
module boundaries appear in [source_audit.md](docs/source_audit.md); paper
correspondence appears in [paper_correspondence.md](docs/paper_correspondence.md).

## Environment

Use Python 3.10 or 3.11. Install the CLI, map/sensing dependencies and backend environments from
the project root:

```bash
python -m venv .venv
.venv/bin/python -m pip install -e '.[map,sensing,test]'
python -m venv .venv-communication
.venv-communication/bin/python -m pip install -e '.[communication]'
python -m venv .venv-radar
.venv-radar/bin/python -m pip install -e '.[radar]'
source .venv/bin/activate
```

Communication and Radar each use a separate backend environment:

| Backend | Configured interpreter | Pinned runtime |
| --- | --- | --- |
| Communication | `../.venv-communication/bin/python` | Sionna RT 2.0.1, Mitsuba 3.8.0, Dr.Jit 1.3.1 |
| Radar | `../.venv-radar/bin/python` | Sionna RT 2.2.0 |

Set `stage3.communication_python` and `stage3.radar_python` to these interpreters.
GPU execution requires a compatible driver; CPU execution requires LLVM and a
matching C++ runtime. The seed controls explicit stochastic parameters. Bitwise
repeatability of external UE procedural generation and parallel RT is pending
validation.

External tools and assets:

- Official CARLA 0.9.16 server and matching Python wheel.
- UE 4.26, the matching CARLA source/content, CarlaTools, StreetMap and Python
  editor plugins for map construction/export.
- Blender and its matching Mitsuba exporter; Blender's Python requires PyYAML.
  Blender 4.2.10 passed the local conversion fixture.
- A compatible actor-mesh library for dynamic-scene assembly.

Configure local resource paths in the chosen YAML. Paths inside YAML resolve
relative to that file; CLI `--output` resolves relative to the calling directory.
Start the server in its CARLA installation directory:

```bash
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000
```

The runner checks the official client/server version and their agreement.
Map preparation and import instructions appear in [carla.md](docs/carla.md) and
[stage1.md](docs/stage1.md).

## Configurations and workflows

| Configuration | Purpose |
| --- | --- |
| [town03.yaml](configs/town03.yaml) | Official Town03 sensing and channel deployment |
| [ris.yaml](configs/ris.yaml) | Source-derived Arizona RIS deployment and site construction |
| [town03_export.yaml](configs/town03_export.yaml) | Standalone export of the official Town03 source map |
| [ris_export.yaml](configs/ris_export.yaml) | Standalone export of the rebuilt RIS source map |
| [rf_materials.yaml](configs/rf_materials.yaml) | RF material definitions |

`python -m sigma` and the installed `sigma` command expose the same CLI.
`doctor` inspects packages and configured interpreters; `validate` checks the
YAML schema. Stage commands accept `--config`, `--output` and `--seed`.

### Official Town03

The Town03 configuration uses a 10 Hz simulation step, a 4 s capture window,
seed 42, one CAV and 29 background vehicles. The RSU is fixed at `[0,0,15]` m
with yaw 180°. `route.selection: seeded_spawn_points` selects a deterministic
forward route from the map's real spawn points and road topology. The configured
CAV suite contains eight entries: seven CARLA sensors and one virtual RCS
Radar. The RSU suite contains seventeen entries: sixteen CARLA sensors and one
virtual RCS Radar. `sionna_rcs` calibration connects these Radar entries to
Stage III processing.
Wireless settings are 28 GHz, 1 GHz bandwidth, 256 subcarriers and
`radio.arrays: [[4,4,8,8]]`. Radar generation is enabled.

Prepare the official map, matching source-map export and actor library, then
run Stage II and III in the same dataset directory:

```bash
python -m sigma validate -c configs/town03.yaml
python -m sigma doctor -c configs/town03.yaml --connect-carla
python -m sigma stage2 -c configs/town03.yaml -o outputs/town03
python -m sigma stage3 -c configs/town03.yaml -o outputs/town03
```

### Site-grounded RIS deployment

The RIS configuration reconstructs `SigmaRIS` for the Arizona deployment.
Supply the site's OSM/Overture/GHS inputs under `assets/local/ris/` and configure
local build resources in `ris.yaml`. Height enrichment uses `overture-ghs`;
vegetation supplementation uses `overture-conservative`.

| Deployment parameter | Configured value |
| --- | --- |
| RIS latitude / longitude | `33.417827, -111.926134` |
| RIS height / yaw | 2 m above the local road reference / 180° |
| RIS panel | 32×32 elements, half-wavelength spacing, 0.02 m thickness |
| RSU latitude / longitude | `33.41920874636455, -111.92717732926977` |
| RSU height / yaw | 2 m above the local road reference / 0° |
| Carrier / bandwidth / subcarriers | 60 GHz / 100 MHz / 1 |
| Array configuration / `rx_antenna_height_m` | `[[1,1,16,1]]` / 0.3 m |
| Channel / phase / reflection amplitude | `effective` / `zero` / 0.8 |
| Components / route | Full component saving / southbound, 30 km/h |
| Seed / simulation step / maximum frames | 1477 / 0.1 s / 150 |
| Background traffic / sensing | 24 vehicles / RIS camera RGB and scene states |

Each invocation captures one trajectory; assign a fresh `trajectory_id` for
another capture. [configuration.md](docs/configuration.md) records parameter
provenance and the explicit deterministic settings. Stage I follows
`stage1.mode`: `prepare` writes fused geographic inputs, and `build` adds
OpenDRIVE and UE reconstruction.

```bash
python -m sigma validate -c configs/ris.yaml
python -m sigma doctor -c configs/ris.yaml
python -m sigma stage1 -c configs/ris.yaml -o outputs/ris
```

Cook and import the generated map using the target official CARLA 0.9.16
source/content and tools. Verify the map on that server, then continue with
sensing and channel generation:

```bash
python -m sigma doctor -c configs/ris.yaml --connect-carla
python -m sigma stage2 -c configs/ris.yaml -o outputs/ris
python -m sigma stage3 -c configs/ris.yaml -o outputs/ris
```

`pipeline` executes I→II→III and records the Stage I preparation/Cook handoff.
Resume from `stage2` after the target map is ready. Explicit seed overrides and
pipeline invocation use:

```bash
python -m sigma pipeline -c configs/ris.yaml -o outputs/ris --seed 1477
```

Each stage writes `manifests/stageN.json`. A stage error records the failure,
returns a nonzero exit code and preserves completed intermediate files. Add
trajectories with fresh `trajectory_id` values. Stage III reuses results with
matching configuration, seed and input content; changed scenes/configurations
use a fresh output directory.

### Static map export

The standalone exporter uses a matching UE source project and source levels.
The supplied configurations read static geometry from
`assets/local/town03/Town03.fbx` and `assets/local/ris/SigmaRIS.fbx`.
Export directly into these asset directories:

```bash
python -m sigma.map_export.cli -c configs/town03_export.yaml -o assets/local/town03
python -m sigma.map_export.cli -c configs/ris_export.yaml -o assets/local/ris
```

The export manifest records the FBX path. `stage3.blender.static_scene` selects
that input independently of the dataset output directory. Enabling
`stage3.map_export` integrates export into Stage III. Blender conversion accepts
FBX and `.blend` scenes; see [map_export.md](docs/map_export.md).
`blender.enabled: false` reuses XML already stored in the dataset.

## Dataset layout and identity

The output retains stage-first directory names from the data protocol:

```text
outputs/<site>/
  manifests/stage{1,2,3}.json
  stage1/
  map_export/
  stage1_multimodal/
    sensor_config.yaml
    ris_metadata.yaml
    000001/
      calibration/<node>.yaml
      nodes/<node>/<modality>/<sensor>/*
      nodes/<node>/radar/<prefix>_<frame_idx>.npz
      world_state/world_state_<frame_idx>.yaml
      annotations/camera_bbox2d/...
      sensor_samples.jsonl
      samples.csv
  stage2_xml/
    static/<site>.xml
    000001/dynamic/dynamic_<frame_idx>.xml
  stage3_radio/
    radio_index.yaml
    runtime_signature.json
    000001/{paths,channels,explicit_paths,radio_maps,ris,...}/
    000001/radar_paths/<node>/<sensor>/p_<frame_idx>.npz
  logs/
```

`stage1/` holds geography and map-build reports. `stage1_multimodal/` holds
Stage II sensing, and `stage2_xml/` holds Stage III scene conversion.

Canonical state keys are `site_id`, integer `trajectory_id`, trajectory-local
`frame_idx`, CARLA `world_frame` and absolute simulation seconds `time_s`.
`trajectory_time_s` records relative trajectory time. Sensor callbacks match
world-frame/time exactly; missing samples trigger an error. CARLA uses meters
and degrees; radio coordinates are `(x,-y,z)`. Calibration stores sensor mounting
matrices, camera intrinsics and output-axis conventions.

`sample_every_n_frames` sets the shared sampling stride;
`world_state_every_n_frames` is an integer multiple of that stride. Sensor-level
`every_n_samples` controls each modality's saving interval. FoV entry establishes
the shared sampling origin.

| Modality | Format |
| --- | --- |
| RGB | PNG |
| Depth | float32 meters, `.npy` |
| Semantics | Class-label NPY |
| LiDAR | Sensor-frame `[x,y,z,intensity]` |
| GNSS/IMU | JSON from same-frame sensor measurements |
| Actor state | Frozen transforms, geometry, identities and simulation time |

Audit and export dataset indexes with:

```bash
python -m sigma.audit_dataset outputs/ris
python -m CSV_data_export.generate_samples_csv --dataset-root outputs/ris
```

## Wireless, RIS and Radar outputs

CSI `h` is complex64 `[subcarrier,rx_antenna,tx_antenna]`. Path NPZ files retain
complex coefficients, absolute delays, angles, LoS/path types, interaction
vertices, deduplication IDs and original path records. Explicit CIR uses
`[rx_antenna,tx_antenna,path]`.

`radio.reconstruction: farfield` uses synthetic-array reconstruction from center
paths. `nearfield` configures physical PlanarArrays and traces antenna pairs with
`synthetic_array=False`. `radio.arrays` entries follow
`[rx_cols,rx_rows,tx_cols,tx_rows]`. Radio Map configuration specifies `center`,
`size` and `cell_size` in Sionna meters; output saves linear `path_gain`, cell
centers, propagation settings and shared frame identity.

### RIS

RIS output includes `h_br[K,Nris,Ntx]`, `h_ru[K,Nrx,Nris]`, direct/cascaded/effective
channels `[K,Nrx,Ntx]` and `phi[K,Nris]`, using
`H_effective = H_direct + H_ru diag(phi) H_br`.

Enable `save_full_component_channels: true` to re-evaluate phase settings from
saved components:

```bash
python -m Sionna_channel_storage.rephase_ris \
  --input components.npz --output rephased.npz --phase-mode focus --reflection-amplitude 0.8
```

Optional use-case adapters in [experiments/](experiments/) provide beam subsets,
RIS-only labels and grouped-fold export. Phase codebooks and a fixed RSU beam
are explicit file inputs; see [experiments.md](docs/experiments.md).

### Radar

Stage II writes virtual `sionna_rcs` calibration. With
`stage3.radar.enabled: true`, Stage III evaluates the saved scene using the
multi-scattering-point target/RCS model and synthesizes configurable FMCW IQ.
Waveform, array and RCS parameters belong to the Stage II sensor item.

CIR includes `[Nrx,Npaths]` coefficients, absolute delays, Doppler, target and
interaction metadata. IQ is complex64 `[Nrx,Nadc,Nchirps]`, accompanied by ADC,
chirp slope/repetition, carrier frequency and frame metadata. Receiver-side
processing can use these saved path and waveform parameters. Sensor details
appear in [carla.md](docs/carla.md). Radar path records are stored under
`stage3_radio/<trajectory>/radar_paths/`; IQ records are stored under
`stage1_multimodal/<trajectory>/nodes/<node>/radar/`.

## Validation

On 2026-11-19, the regression suite completed with **124 passed, 0 failed and
0 skipped**. CLI validation passed for the Town03 and RIS deployments and the
geographic fixture. The fixture environment check and local OSM–Overture
preparation passed, producing three buildings and one road. Static compilation
passed for 138 Python files. Original-source and deployment-provenance checksum
checks matched their recorded inputs.

Run local tests and the small geographic fixture:

```bash
python -m pytest -q
python -m sigma stage1 -c tests/fixtures/stage1/geography.yaml -o /tmp/sigma-geography-smoke
```

Run each integration check in its configured Sionna environment:

```bash
python tests/backend_smoke.py --output /tmp/sigma-wireless-smoke
python tests/ris_backend_smoke.py --output /tmp/sigma-ris-smoke
PYTHONPATH=. python tests/radar_backend_smoke.py
```

Recorded checks cover local geography, CLI/schema contracts, Blender–Sionna
conversion, CPU channels/Radio Map/RIS/Radar, packaging and an RTX 4090 D GPU
Sionna probe. Official-CARLA real-site construction/Cook, live synchronized
sensing, same-site multimodal integration and full 1,024-element RIS RT remain
pending validation. Test evidence and resource details appear in
[validation.md](docs/validation.md) and [remaining_work.md](docs/remaining_work.md).

## Acknowledgements and licensing

We thank the CARLA team for the [Digital Twin Tool](https://carla.readthedocs.io/en/0.9.16/adv_digital_twin/).
SiGMA draws on its procedural map construction and extends it with OSM–Overture
geographic fusion, automated command-line construction, geographic/road/semantic
checks, synchronized sensing, RF material assignment, and wireless/RIS/Radar
generation.

Migrated platform code retains the [MIT license](LICENSES/platform-MIT.txt).
Map-construction/export code preserves its original copyright headers;
redistribution grants require rights-holder confirmation before public release.
CARLA, Unreal, geographic data and other external resources follow their own
terms. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for attribution and
[source_integrity.json](docs/source_integrity.json) for the original-source
integrity record.
