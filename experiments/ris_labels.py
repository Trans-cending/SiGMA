"""Export camera-associated RIS-only powers and explicit three-fold groups.

Run from the project root::

    python -m experiments.ris_labels --manifest samples.csv \
        --codebook phases.npz --fixed-beam rsu_beam.npy --output labels --seed 7

This adapter uses the source component-channel convention. Inputs comprise
256/1024 codewords (including fixed incident compensation), a single fixed
unit-norm RSU beam, and audited geometric trajectory group IDs.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from Sionna_channel_storage.ris_channel import validate_component_shapes


def candidate_powers(
    h_br: np.ndarray, h_ru: np.ndarray, phases: np.ndarray, fixed_beam: np.ndarray,
    *, field_coupling: float = 1.0,
) -> np.ndarray:
    """Compute RIS-only label powers |H_RU Phi_q H_BR f_RSU|².

    Input axes match saved Stage III products: H_BR=[1,1024,16],
    H_RU=[1,1,1024], phases=[256|1024,1024], beam=[16]. Use the
    channel coefficients in H_RU directly in the cascaded-link product.
    ``field_coupling`` preserves the source projected-aperture factor; reflection
    amplitude is part of each supplied phase coefficient.
    """
    if validate_component_shapes(h_br, h_ru) != (1, 1024, 16, 1):
        raise ValueError("paper RIS labels require 1 subcarrier, 1024 RIS elements, 16 RSU ports, 1 UE port")
    phi = np.asarray(phases, dtype=np.complex128)
    beam = np.asarray(fixed_beam, dtype=np.complex128)
    if phi.shape not in ((256, 1024), (1024, 1024)):
        raise ValueError("phases must have shape [256,1024] or [1024,1024]")
    if beam.shape != (16,) or not np.isclose(np.linalg.norm(beam), 1.0, rtol=1e-6, atol=1e-7):
        raise ValueError("fixed RSU beam must have shape [16] and unit norm")
    if not all(np.isfinite(a).all() for a in (h_br, h_ru, phi, beam)):
        raise ValueError("all channel, phase and beam coefficients must be finite")
    if np.any(np.abs(phi) > 1.0 + 1e-6):
        raise ValueError("passive phase coefficients cannot have amplitude greater than one")
    coupling = float(field_coupling)
    if not np.isfinite(coupling) or not 0 <= coupling <= 1:
        raise ValueError("field coupling must be finite and in [0,1]")
    illuminated = np.asarray(h_br, dtype=np.complex128)[0] @ beam
    response = coupling * (phi @ (np.asarray(h_ru, dtype=np.complex128)[0, 0] * illuminated))
    powers = np.abs(response) ** 2
    if not np.isfinite(powers).all():
        raise ValueError("nonfinite RIS candidate powers")
    return powers.astype(np.float64)


def grouped_folds(rows: Sequence[Mapping[str, str]], seed: int) -> list[int]:
    """Assign whole caller-supplied geometric groups to three held-out folds.

    Balancing uses frame counts, with seeded tie-breaking. The caller supplies
    geometric group IDs. Each trajectory belongs to exactly one group across
    all its frames.
    """
    groups: dict[tuple[str, str], list[int]] = {}
    trajectory_groups: dict[tuple[str, str], tuple[str, str]] = {}
    for i, row in enumerate(rows):
        if any(not str(row.get(k, "")).strip() for k in ("site_id", "trajectory_id", "group_id")):
            raise ValueError("site_id, trajectory_id and group_id must be explicit and nonempty")
        trajectory = (str(row["site_id"]), str(row["trajectory_id"]))
        group = (trajectory[0], str(row["group_id"]))
        if trajectory in trajectory_groups and trajectory_groups[trajectory] != group:
            raise ValueError(f"trajectory {trajectory} has inconsistent geometric group IDs")
        trajectory_groups[trajectory] = group
        groups.setdefault(group, []).append(i)
    if len(groups) < 3:
        raise ValueError("three-fold evaluation requires at least three independent geometric groups")
    rng = np.random.default_rng(seed)
    keys = sorted(groups)
    rng.shuffle(keys)
    keys.sort(key=lambda key: -len(groups[key]))
    totals = [0, 0, 0]
    folds = [-1] * len(rows)
    for key in keys:
        target = int(np.argmin(totals))
        for index in groups[key]:
            folds[index] = target
        totals[target] += len(groups[key])
    return folds


def _relative_input(base: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def export_labels(manifest: Path, codebook: Path, fixed_beam: Path, output: Path, seed: int) -> dict:
    """Export powers.npz, samples.csv and provenance.json into a new directory.

    The output directory must be fresh. Manifest paths resolve relative to the
    input manifest. Every sample is validated before export.
    """
    with manifest.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {"site_id", "trajectory_id", "frame_id", "timestamp_s", "rgb_path", "components_path", "group_id"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"manifest is missing {sorted(required - set(reader.fieldnames or []))}")
        rows = list(reader)
    if not rows:
        raise ValueError("manifest has no samples")
    folds = grouped_folds(rows, seed)
    with np.load(codebook, allow_pickle=False) as data:
        if "phi" not in data or "metadata_json" not in data:
            raise ValueError("codebook requires phi and metadata_json arrays")
        phases = np.asarray(data["phi"])
        codebook_metadata = json.loads(str(np.asarray(data["metadata_json"]).reshape(())))
    if codebook_metadata.get("fixed_incident_compensation") is not True:
        raise ValueError("codebook metadata must explicitly attest fixed_incident_compensation=true")
    beam = np.load(fixed_beam, allow_pickle=False)
    all_powers = []
    seen = set()
    for row in rows:
        key = tuple(row[k] for k in ("site_id", "trajectory_id", "frame_id"))
        if key in seen:
            raise ValueError(f"duplicate sample identifier {key}")
        seen.add(key)
        if not np.isfinite(float(row["timestamp_s"])) or float(row["timestamp_s"]) < 0:
            raise ValueError("timestamp_s must be finite and nonnegative")
        rgb = _relative_input(manifest.parent, row["rgb_path"])
        components = _relative_input(manifest.parent, row["components_path"])
        if not rgb.is_file():
            raise FileNotFoundError(rgb)
        with np.load(components, allow_pickle=False) as data:
            if not {"h_br", "h_ru", "metadata_json"}.issubset(data.files):
                raise ValueError(f"{components} needs saved full RIS component channels")
            metadata = json.loads(str(np.asarray(data["metadata_json"]).reshape(())))
            # The migrated writer stores state scalars at NPZ top level;
            # original products may store them only in metadata, or lack them.
            state_aliases = {"site_id": ("site_id",), "trajectory_id": ("trajectory_id",),
                             "frame_id": ("frame_id", "frame_idx"),
                             "timestamp_s": ("timestamp_s", "time_s")}
            for name, aliases in state_aliases.items():
                for alias in aliases:
                    candidates = []
                    if alias in metadata:
                        candidates.append(metadata[alias])
                    if alias in data:
                        candidates.append(np.asarray(data[alias]).reshape(()).item())
                    for value in candidates:
                        if name == "timestamp_s":
                            matches = np.isclose(float(value), float(row[name]), atol=1e-6, rtol=0)
                        elif name in ("frame_id", "trajectory_id") and str(value).isdigit() and row[name].isdigit():
                            matches = int(value) == int(row[name])
                        else:
                            matches = str(value) == row[name]
                        if not matches:
                            raise ValueError(f"component {name} disagrees with manifest for {key}")
            coupling = metadata.get("panel_projected_aperture_coupling", {}).get("field_amplitude_factor", 1.0)
            if metadata.get("br_has_valid_path") is False or metadata.get("ru_has_valid_path") is False:
                coupling = 0.0
            all_powers.append(candidate_powers(data["h_br"], data["h_ru"], phases, beam, field_coupling=coupling))
        row["rgb_path"] = str(rgb)
        row["components_path"] = str(components)
    powers = np.stack(all_powers)
    output.mkdir(parents=True, exist_ok=False)
    temp = output / f"powers.npz.tmp.{os.getpid()}"
    with temp.open("wb") as handle:
        np.savez_compressed(handle, powers=powers, labels=np.argmax(powers, axis=1), test_fold=np.asarray(folds, np.int8))
    temp.replace(output / "powers.npz")
    fields = list(dict.fromkeys([*rows[0], "test_fold", "label", "has_cascaded_power"]))
    with (output / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for i, row in enumerate(rows):
            writer.writerow({**row, "test_fold": folds[i], "label": int(np.argmax(powers[i])), "has_cascaded_power": bool(np.max(powers[i]) > 0)})
    def digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    provenance = {
        "schema_version": 1, "label_index_base": 0, "label_channel": "RIS-only cascaded",
        "samples": len(rows), "codebook_size": int(phases.shape[0]), "seed": seed,
        "manifest_sha256": digest(manifest), "codebook_sha256": digest(codebook),
        "fixed_beam_sha256": digest(fixed_beam), "codebook_metadata": codebook_metadata,
        "group_source": "explicit manifest group_id; no geometry clustering performed",
        "association_source": "explicit manifest; existing component state metadata checked where present",
        "training_implemented": False,
    }
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    return provenance


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--codebook", required=True, type=Path)
    parser.add_argument("--fixed-beam", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    print(json.dumps(export_labels(args.manifest, args.codebook, args.fixed_beam, args.output, args.seed), indent=2))


if __name__ == "__main__":
    main()
