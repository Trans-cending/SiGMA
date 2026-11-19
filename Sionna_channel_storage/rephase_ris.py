#!/usr/bin/env python3
"""Rebuild effective RIS CSI while preserving saved state and physical coupling.

Example::

    python -m Sionna_channel_storage.rephase_ris --input ris_000025.npz \
        --output rephased_000025.npz --phase-mode focus --reflection-amplitude 0.8

The input component channels and shared-state identifiers are retained. Phase
overrides include the source projected-aperture factor and invalid-leg mask;
the direct component remains separate from the RIS-only cascaded term.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from .ris_channel import synthesize_ris
from .ris_writer import RIS_OUTPUT_SCHEMA_VERSION


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phase-mode", choices=("zero", "focus"), required=True)
    parser.add_argument("--reflection-amplitude", type=float, default=1.0)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.output.exists() and not args.force:
        raise FileExistsError(f"refusing to overwrite {args.output}; pass --force")
    with np.load(args.input, allow_pickle=False) as data:
        required = {"h_direct", "h_br", "h_ru", "metadata_json"}
        missing = sorted(required - set(data.files))
        if missing:
            raise ValueError(
                f"{args.input} lacks {missing}; generate with "
                "ris.channel.save_full_component_channels=true"
            )
        # Preserve all source arrays, including schema and identifiers added by
        # the shared-state writer. Older rephased products used source_* only.
        payload = {key: np.asarray(data[key]).copy() for key in data.files}
        hash_key = "ris_config_hash" if "ris_config_hash" in data else "source_ris_config_hash"
        if hash_key not in data:
            raise ValueError(f"{args.input} lacks its source RIS configuration hash")
        direct = np.asarray(data["h_direct"], np.complex64)
        h_br = np.asarray(data["h_br"], np.complex64)
        h_ru = np.asarray(data["h_ru"], np.complex64)
        source_hash = str(np.asarray(data[hash_key]).reshape(()))
        metadata = json.loads(str(np.asarray(data["metadata_json"]).reshape(())))
    if not isinstance(metadata, dict):
        raise ValueError("metadata_json must contain an object")
    if not all(np.isfinite(array).all() for array in (direct, h_br, h_ru)):
        raise ValueError("saved RIS components contain nonfinite coefficients")
    coupling = float(metadata.get("panel_projected_aperture_coupling", {}).get("field_amplitude_factor", 1.0))
    if not np.isfinite(coupling) or not 0 <= coupling <= 1:
        raise ValueError("saved projected-aperture field factor must be in [0,1]")
    if metadata.get("br_has_valid_path") is False or metadata.get("ru_has_valid_path") is False:
        coupling = 0.0
    if not np.isfinite(args.reflection_amplitude) or not 0 <= args.reflection_amplitude <= 1:
        raise ValueError("reflection-amplitude must be finite and in [0,1]")
    result = synthesize_ris(direct, h_br, h_ru, {
        "phase_mode": args.phase_mode,
        "reflection_amplitude": args.reflection_amplitude * coupling,
    })
    metadata.update({
        "rephased_from": str(args.input.resolve()),
        "source_ris_config_hash": source_hash,
        "phase_mode": args.phase_mode,
        "reflection_amplitude": float(args.reflection_amplitude),
        "physical_reflection_field_amplitude": float(args.reflection_amplitude * coupling),
        "ray_tracing_reused": True,
    })
    phase_configuration = {"phase_mode": args.phase_mode,
                           "reflection_amplitude": args.reflection_amplitude}
    payload.update(
        h_direct=direct, h_br=h_br, h_ru=h_ru,
        phi=result["phi"], h_cascade=result["h_cascade"], h_effective=result["h_effective"],
        ris_schema_version=np.asarray(RIS_OUTPUT_SCHEMA_VERSION, dtype=np.int16),
        ris_config_hash=np.asarray(source_hash), source_ris_config_hash=np.asarray(source_hash),
        phase_configuration_hash=np.asarray(hashlib.sha256(json.dumps(phase_configuration, sort_keys=True).encode()).hexdigest()),
        metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.output.with_suffix(args.output.suffix + ".tmp")
    with tmp.open("wb") as handle:
        np.savez_compressed(handle, **payload)
    tmp.replace(args.output)
    print(args.output)


if __name__ == "__main__":
    main()
