# Supplied WCNC paper: feature correspondence

The feature scope follows “SiGMA: Site-Grounded Multi-Modal ISAC
Data Generation for Environment-Aware Communications”. The table maps its sections to migrated data-generation source and recorded
execution evidence. Real-site acceptance status is tracked separately in
[remaining_work.md](remaining_work.md).

| Paper requirement | Independent-project implementation / evidence | Status and limits |
| --- | --- | --- |
| III-A: OSM road network and primary buildings | `sigma/map_construction/tools` geographic preparation | Source retained; live geographic service acquisition requires connectivity |
| III-A: complementary Overture footprints | Source Overture preparation and conflict screening under `sigma/map_construction` | OSM takes precedence; conflict-free Overture footprints supplement coverage |
| III-A: shared projection and spatial checks | Source projected-coordinate, boundary, overlap, road/water screening | Small offline geometry tests passed; complete real-site validation pending |
| III-A: OSM to OpenDRIVE and reachability | Official CARLA OSM conversion, source OpenDRIVE repair/check helpers | Official wheel and source build required where converter support is absent |
| III-A: procedural buildings, facades, textures, terrain, vegetation | Source Unreal automation in `sigma/map_construction/ue` and `bridge` | Requires compatible external Unreal plugins, assets and DTT setup |
| III-A: semantic assets and radio materials | Source semantic/tag automation and `configs/rf_materials.yaml` | External asset material/tag coverage needs site-specific validation |
| III-B: predefined road-constrained routes and traffic | `sigma/sensing/routes.py` and `runner.py` from source road-graph/traffic logic | Requires official CARLA 0.9.16 client/server and rebuilt imported custom maps |
| III-B: RGB, depth, LiDAR, GPS/IMU and actor state | `sigma/sensing/{sensors,frame_sync,writer}.py` from source sensor/synchronization/schema logic | Live callbacks and file alignment require server execution; GPS must originate in GNSS callbacks |
| III-B: frame-indexed intervals and FoV acquisition | Stage II synchronous capture and configured interval/FoV policy | Acquisition-rate differences must keep world-state frame references |
| III-B: CARLA to radio `(x,-y,z)` and orientation | `mosaic_core.coordinates`, source Blender/pose loaders | Static meshes and actors must use the same convention |
| III-C.1: Blender scene conversion and materials | `Blender_xml_storage` geometry/material/XML modules | Blender, scene meshes and Mitsuba/Sionna exporter are external runtime requirements |
| III-C.1: paths, LoS/NLoS, CSI and radio maps | `Sionna_channel_storage` RT, path, channel and map modules | Requires a compatible Sionna runtime, meshes and available GPU/backend |
| III-C.1: far-field synthetic and near-field explicit arrays | Wireless array implementation and mode validation in `Sionna_channel_storage` | Mode-specific testing must distinguish explicit tracing from synthetic reconstruction |
| III-C.1 Eq. (2): component, direct, cascade and effective RIS channels | `mosaic_core.ris`, `Sionna_channel_storage/{ris_pipeline,ris_channel,ris_writer,rephase_ris}.py` | Far-field reconstruction and physical explicit-array component tracing passed the small CPU fixture in `tests/ris_backend_smoke.py`; full geographic deployment validation pending |
| III-C.2: scattering targets, RCS, Sionna Radar CIR, gains, delays and Doppler | `Radar_channel_storage` configuration, Sionna Radar generator and scene inputs | Live Radar RT requires its compatible Sionna Radar environment |
| III-C.2: configurable FMCW `[Nrx,Nadc,Nchirps]` complex IQ | `Radar_channel_storage/fmcw.py` | Numerical synthesis can be tested offline; retained path metadata enables alternate waveform processing |
| IV-B: 32/64 beam subset optimum | `experiments/metrics.py:beam_subset` | New numerical adapter; original measured-codebook conversion delegates to an external project absent from source |
| IV-C: 1,024 RIS elements, fixed 16-port beam, single UE and 256/1,024 RIS-only labels | `experiments/ris_labels.py` plus source component channels and deployment parameters | New export adapter; angular codebooks and beam coefficients are explicit external inputs |
| IV-C: 3-fold grouped evaluation | Exported `group_id` and `test_fold` from `experiments/ris_labels.py` | Whole explicit groups remain disjoint; geometric grouping is supplied explicitly; original research memberships are external |
| IV-D Eq. (5): NG and Top-k, failures contribute zero | `experiments/metrics.py:ranking_metrics` | Tested numerical metric adapter |

The audit excluded caches, generated datasets, binary Unreal assets, weights,
historical experiment outputs, logs and environment directories. External
conversion inputs are recorded in
[experiments.md](experiments.md).

The actual RIS backend summary is stored in
[ris_backend_validation.json](ris_backend_validation.json). Its six-element
fixture checks Sionna channels, asymmetric tensor axes, shared-state identity and
saved phase re-evaluation. The panel geometry meets the far-field criterion in
both reconstruction and physical explicit-array tests. Geographic acquisition
and full 1,024-element RT remain pending acceptance checks.
