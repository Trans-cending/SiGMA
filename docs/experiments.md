# Optional use-case data export

The project supplies data-generation and export interfaces for the WCNC paper's
use cases. `experiments/configs/paper_protocol.yaml` records paper parameters;
`configs/ris.yaml` records the audited fixed
source deployment. The paper parameter file records the data-export protocol; `configs/ris.yaml`
is the executable deployment. See [configuration.md](configuration.md) for
source values and deterministic capture settings.

The original deployment provenance is
`mosaic_agent/experiments/ris_camera_phase_optimization.json`. Its deterministic
parameters were extracted into the independent manifest. Maps used for new
acquisition require rebuilding and import for official CARLA 0.9.16.

Current acceptance covers generation, saved channel components, phase
re-evaluation and export.

## Beam subset adapter

`experiments.metrics.beam_subset(powers_64, 32)` keeps original zero-based
indices `0,2,...,62` and recomputes the optimum over the retained linear powers.
`beam_subset(powers_64, 64)` retains the full codebook. Inputs use linear units.
These tested numerical adapters support downstream consumers of generated data.

DeepSense 6G Scenario 3 measurements, calibration, association metadata and beam
patterns are external inputs for users conducting that research. The source
conversion wrapper delegates channel-to-beam conversion to the external
`Deepmagic_transform` project.

## Camera-assisted RIS data

The retained component pipeline supports a 32×32 panel, 16 RSU ports, a single
UE port, direct channel and both component channels. Source deployment values
are 60 GHz, one subcarrier, 100 MHz bandwidth and 0.8 reflection amplitude with
full component saving. RIS-to-vehicle visibility is checked separately from
RSU-to-UE visibility; actual component RF paths are determined by tracing.

Label export accepts explicit 256/1,024 phase codebooks and a fixed RSU beam:

```bash
python -m experiments.ris_labels --manifest ris_samples.csv \
  --codebook phases_256.npz --fixed-beam rsu_beam.npy \
  --output outputs/ris_labels_256 --seed 7
python -m experiments.ris_labels --manifest ris_samples.csv \
  --codebook phases_1024.npz --fixed-beam rsu_beam.npy \
  --output outputs/ris_labels_1024 --seed 7
```

The codebook NPZ contains complex `phi` `[256,1024]` or `[1024,1024]`, including
passive reflection amplitude, plus scalar string `metadata_json`. Its object
includes `"fixed_incident_compensation": true` to record supplied-codebook
provenance. Codebook preparation must establish angular coverage, compensation,
element order, phase sign and coordinate convention. `rsu_beam.npy` contains
one complex unit-norm `[16]` vector, fixed across samples and directed toward
the RIS. The exporter records both file hashes.

The required `ris_samples.csv` columns are:

```csv
site_id,trajectory_id,frame_id,timestamp_s,rgb_path,components_path,group_id
site3,000001,25,1.25,images/frame25.png,channels/ris_000025.npz,southbound_group1
```

Each row joins a camera image and saved RIS component record for the same state.
Relative paths resolve beside the manifest. Export validates state IDs and time
in NPZ scalars (`frame_idx`, `time_s`) and component metadata. `timestamp_s`
means canonical `world_state.time_s`, the absolute CARLA simulation time shared
by sensor callbacks, CSI/RIS and Radar. Padded trajectory IDs compare by numeric
value. Every record requires state identity, existing files and matching time.

The component convention is `h_br=[1,1024,16]`, `h_ru=[1,1,1024]` and
`h_direct=[1,1,16]`. `h_ru` is the receive-side row. Candidate powers are
`|h_ru @ diag(phi[q]) @ h_br @ fixed_beam|²`, including source projected-aperture
field coupling. Labels use the RIS-only cascaded link. An invalid component
leg produces zero powers; its row retains `has_cascaded_power=False`. A tie
among zero powers selects label 0, interpreted together with that flag.

Choose a fresh output directory. The exporter writes:

| File | Contents |
| --- | --- |
| `powers.npz` | Linear `powers=[samples,Q]`, zero-based `labels=[samples]`, `test_fold=[samples]` |
| `samples.csv` | State IDs, geometric group IDs, resolved input paths, labels and fold IDs |
| `provenance.json` | Input hashes, codebook metadata, seed and channel definition |

`group_id` joins geometrically similar trajectories. The exporter assigns whole
groups to three held-out folds using seeded frame-count balancing; each group
belongs to one fold. At least three independent groups are required. Original
paper group memberships and phase/beam files remain external research artifacts.

## Metrics and validation

`experiments.metrics.ranking_metrics(powers, rankings, [1,5,10], valid=...)`
computes normalized gain and Top-k accuracy relative to a full scan of the
same codebook. Failed detections, associations and predictions contribute zero
and retain their denominator entries. Empty rankings encode failures. All-zero
reference rows yield zero normalized gain and an explicit count.

Unit tests exercise RIS algebra, fixed-beam/tensor dimensions, group isolation,
subset labels, failed detections and state mismatch errors using constructed
numerical fixtures.

Saved component re-evaluation uses:

```bash
python -m Sionna_channel_storage.rephase_ris \
  --input components.npz --output rephased.npz \
  --phase-mode focus --reflection-amplitude 0.8
```

Zero/focus policies preserve state scalars, source schema/hash and physical
projected-aperture coupling. Repeated rephasing starts from the original
components and applies coupling once. External angular-codebook scans use the
separate label exporter above.

`python tests/ris_backend_smoke.py --output /tmp/sigma-ris-new` runs actual
Sionna RT 2.0.1 integration. Its six-element panel uses asymmetric dimensions
`[subcarrier,RIS,RSU,UE]=[3,6,4,2]`, exercising farfield reconstruction and
physical explicit-array modes, component/direct/cascaded/effective channels,
shared state IDs and saved rephasing. The CPU run passed; the summary is in
[ris_backend_validation.json](ris_backend_validation.json). This fixture lies
in the small panel's far field. Full geographic deployment and 1,024-element RT
remain separate acceptance checks.
