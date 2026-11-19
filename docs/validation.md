# Validation record

Backend execution evidence was recorded on 2026-11-16 and the subsequent GPU
probe. The current source/configuration regression was executed on 2026-11-19.
[project_validation.json](project_validation.json) records the latest results;
[remaining_work.md](remaining_work.md) tracks live-site acceptance.

## Deployment validation

The full suite completed with **124 passed, 0 failed, 0 skipped** using Python
3.10.19. Both Town03/RIS deployment YAMLs, both standalone export schemas and
the local geographic fixture validated successfully. The fixture environment
check and OSM–Overture preparation passed, retaining two OSM buildings and adding
one conflict-free Overture footprint alongside one road.

The seeded Town03 route tests exercise Driving-spawn filtering, candidate
bounds, deterministic ordering, forward-road reachability and the runner's
route.json output. The official client also loaded the Town03 OpenDRIVE geometry;
live-world spawn metadata remains part of server acceptance.

All 138 Python files compiled with Python 3.10. The wheel was built, installed
under /tmp and exercised through package imports, resource access, both CLI
entry points and both deployment validators. The 781-file source snapshot,
nine configuration sources and two RIS dataset metadata files matched their
recorded checksums. README.md and README_CN.md share the same 12 command/data
blocks and matching configuration values.

Configured local deployment paths and CARLA endpoint observations are retained
in the JSON report. Install the documented environments, select the local
asset/build paths and start the target server for live execution.

## Completed checks

| Check | Result | Executed scope |
| --- | --- | --- |
| Static compilation | Passed | 138 Python files with Python 3.10.19 |
| Unit/regression suite | **124 passed, 0 failed, 0 skipped** | Python 3.10.19; NumPy, Shapely, PyProj, pytest; numerical contracts and local geography |
| CLI help/configuration | Passed | Town03/RIS deployments, export schemas, malformed YAML/flags, failure manifests and cache invalidation |
| CLI doctor | Passed for configured communication and Radar runtimes | Sionna RT 2.0.1/Mitsuba 3.8.0 and Sionna RT 2.2.0/Mitsuba 3.9.1 |
| Small OSM–Overture CLI run | Passed | Constructed geographic fixture; two OSM buildings retained, one conflict-free Overture building added in prepare mode |
| Blender static/dynamic conversion | Passed | Blender 4.2.10 with matching Mitsuba exporter; constructed ground/vehicle meshes and frozen world_state |
| Blender → Sionna | Passed | Converted meshes and materials yield finite `[3,1,4]` CSI with positive energy and paths for one constructed frame |
| Synthetic/explicit arrays | Passed on CPU backend | `[3,2,8]` CSI, distinct antenna-pair delays, 2 cm near-field geometry and front-hemisphere filtering |
| Radio Map | Passed on CPU backend | Positive finite grid gains, frozen-state identity, saved grid/propagation settings |
| RIS component pipeline | Passed on CPU backend | Both modes: `H_BR[3,6,4]`, `H_RU[3,2,6]`, direct/cascade/effective `[3,2,4]`; paths, coupling, rephase and IDs |
| 1,024-element RIS export/rephase | Passed numerically | 256/1,024 phase candidates, fixed beam, single UE, RIS-only labels and grouped folds |
| RCS Radar CIR/FMCW | Passed on CPU backend | Two vehicle targets, 10 paths, minimum range 17.75 m, finite Doppler, complex64 IQ `[4,256,128]` |
| Dataset identity audit | Passed on backend products | CSI, paths, maps and RIS share fixture site/trajectory/frame/world-frame/time; sensor-file contracts tested separately |
| Standalone wheel | Built and imported from `/tmp` | CLI, RF material registry, packaged bridge/export resources and notices |
| Runtime/dependency scan | Passed | Local imports, declared dependencies, entry points and filesystem links |
| GPU Sionna probe | Passed | RTX 4090 D on gpu32; physical array tracing, finite CIR/CSI, SLURM exit 0:0 |

CPU tests used process-local LLVM/libstdc++ settings and existing environments.
Generated scenes, channels, IQ, wheel and temporary installs were written under
`/tmp`. Small RIS/Radar summaries are retained beside this document.

The Blender 5.1 attempt reported a missing matching Mitsuba exporter. Operator
availability and Blender error-exit checks were corrected; Blender 4.2 then
passed static/dynamic conversion and the combined wireless chain. Host-dependent
CARLA version-test setup was isolated, and a real mismatched-wheel case was
added. The recorded suite completed successfully.

## Pending acceptance

| Item | Required execution or input |
| --- | --- |
| Official CARLA 0.9.16 runtime | Start the discovered official server on an allocated GPU; verify client/server versions and rendered sensors |
| Real Stage I build | Real OSM–Overture inputs, official OSM2ODR, editor/bridge generation, geographic and semantic validation |
| Native bridge build | Existing compiled bridge is present; reproduce compilation after source/patch audit as required |
| Cook/Import and custom-map loading | Use official target-version tooling, then load and validate the cooked map |
| Live trajectory and synchronized sensors | Record RGB/Depth/LiDAR/GNSS/IMU, background traffic and FoV; audit callback frames and timestamps |
| Real-site multimodal alignment | Use the same rebuilt map, actor library and saved states across Blender, wireless and Radar outputs |
| Full 1,024-element RIS RT | Execute component tracing and rephasing with a 32×32 panel |
| Geographic downloads | Execute real OSM/Overture service acquisition with recorded release and attribution |
| Actor-library matching | Validate available meshes against official-server actor geometry and blueprints |
| Public release authorization | Confirm map/export source and selected-asset redistribution rights |

Optional RIS label export accepts externally prepared phase tables and fixed-beam
files.

## Re-run commands

```bash
cd /home/ybianaf/David_PROGRAM/Python/SiGMA
python -m pytest -q
python -m sigma validate -c configs/ris.yaml
python -m sigma stage1 -c tests/fixtures/stage1/geography.yaml -o /tmp/sigma-geography-new
python tests/backend_smoke.py --output /tmp/sigma-wireless-new
python tests/ris_backend_smoke.py --output /tmp/sigma-ris-new
PYTHONPATH=. python tests/radar_backend_smoke.py
python -m sigma.audit_dataset /path/to/generated/site
```

Choose the corresponding isolated Sionna interpreter for each backend command.
For CPU, set `DRJIT_LIBLLVM_PATH` to a compatible LLVM shared library and expose
matching libstdc++ through `LD_LIBRARY_PATH`. Radar CPU smoke uses
`MITSUBA_VARIANT=llvm_ad_mono_polarized`.

Blender fixture invocation uses Blender 4.2 with
`--background --factory-startup --python-exit-code 2 --python tests/blender_fixture.py -- OUTPUT`.
The fixture writes a deployment config and `dataset/stage1_multimodal` state.
Use the generated config with `sigma stage3`, the matching plugin environment
and a fresh output containing those states. Fixture results measure small
constructed scenes; real-site status is tracked in the table above.
