# Current acceptance work

The acceptance scope is the standalone SiGMA data-generation framework: site
construction, synchronized sensing, wireless channels, RIS components and Radar
CIR/FMCW IQ. RIS component generation and configurable phase re-evaluation remain
core features.

## GPU execution evidence

SLURM job 2312701 completed a single-GPU Sionna probe successfully. The login
process runs on `login1`; GPU jobs use the cluster's GPU partitions through the
`khaledgroup` account.

| Item | Observed value |
| --- | --- |
| Job | 2312701, completed with exit 0:0; allocation released |
| Partition / node | gpu-rtx4090d / gpu32 |
| Device | NVIDIA GeForce RTX 4090 D |
| Visible GPU memory | 23028 MiB |
| Driver | 570.211.01 |
| Backend | Sionna RT 2.0.1, Mitsuba `cuda_ad_mono_polarized` |
| Numerical check | Physical antenna-pair tracing; CIR `[1,2,1,4,1,1]`, CSI `[1,2,4,3]`, finite and positive channel energy |
| Allocation duration | 17 seconds, 1 GPU, 2 CPUs, 8 GiB host memory |

The [raw report](gpu_probe.json) records GPU/CUDA/Sionna execution. CARLA/Unreal
rendering, Radar 2.2 GPU execution and a full geographic site remain separate
runtime checks.

## Verified external resources

- Official CARLA server:
  `/project/khaledgroup/ybianaf/UE4_CARLA/CARLA_0.9.16_official`, including
  VERSION 0.9.16, launcher, Shipping binary and official wheels.
- Official client environment:
  `/home/ybianaf/miniconda3/envs/carla-osm-builder/bin/python`, Python 3.10.20.
  Distribution and actual client versions are both 0.9.16; NumPy, PyYAML,
  Pillow, PyProj and Shapely are present, and sensing imports succeed.
- UE 4.26 editor and compiled CarlaTools. The module exports
  DigitalTwinHeadlessBridge methods; bridge source matches the delivered source.
  Live map generation remains pending.
- Actor library: 41 vehicle, 7 truck and 24 human FBX files. Official-server
  geometry/blueprint matching and redistribution rights require validation.

The existing CARLA source tree starts from the official release commit and
contains historical changes to lights, building blueprints and build scripts.
A reproducible build requires an audit of the DTT/projection changes and those
additional patches. Resource inspection was read only.

## Remaining work

1. **Official runtime smoke:** allocate a GPU, start the official 0.9.16 server,
   verify client/server versions and capture rendered sensors.
2. **Real Stage I site:** build a small real OSM–Overture region using DTT and
   the editor bridge. Validate geometry, textures, semantics, OpenDRIVE and
   geographic alignment; Cook/Import and load the map on the official server.
   Recompile the native plugin if the audited build requires it.
3. **Live Stage II acquisition:** run a short road-constrained trajectory with
   background traffic and FoV acquisition. Capture RGB, Depth, LiDAR, GNSS, IMU
   and actor truth; verify callback frames, timestamps and saved file alignment.
4. **Same-site multimodal loop:** export the map and actors, validate mesh
   transforms and RF materials, then generate CSI, Radio Map, RIS components and
   Radar CIR/FMCW IQ from the saved states. Audit coordinates and all state IDs.
5. **RIS size validation:** run real component tracing and phase re-evaluation
   for a 32×32/1,024-element panel. Existing actual RT tests use six elements;
   existing 1,024-element checks cover numerical tensors and phase processing.
6. **Pipeline and release:** finalize Cook/Import handoff and resume steps,
   update deployment configurations and acceptance evidence, and confirm
   licenses for map/export code and assets selected for redistribution.

The recorded regression suite contains 124 passing tests. GPU probing adds
backend execution evidence; the live acceptance checks above retain pending
status until their own executions are recorded.
