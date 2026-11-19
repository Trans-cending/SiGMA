#!/usr/bin/env python3
"""Generate the final root-level samples.csv for the stage-first dataset.

This post-processing script scans the completed dataset and creates
a deterministic CSV index with one row per
(map_name, trajectory_id, frame_idx).

Run: python -m CSV_data_export.generate_samples_csv --dataset-root outputs/site
An explicit dataset root selects the dataset to index.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from mosaic_core.dataset import frame_idx_from_name_or_none as frame_idx_from_name

try:
    from tqdm import tqdm
except Exception:  # pragma: no cover - fallback for minimal envs
    def tqdm(iterable=None, total=None, desc=None, unit=None):
        return iterable if iterable is not None else []

BASE_FIELDS = ["map_name", "trajectory_id", "frame_idx"]
COMPACT_TRAJ_RE = re.compile(r"^\d{6}$")


def parse_trajectory_name(name: str, map_hint: str | None = None) -> Tuple[str, int]:
    value = str(name or "").strip()
    if COMPACT_TRAJ_RE.fullmatch(value) is None:
        raise ValueError(
            f"trajectory directory must use canonical six-digit name NNNNNN, got {name!r}"
        )
    if not map_hint:
        raise ValueError("map_hint is required for canonical compact trajectory directories")
    return str(map_hint), int(value)


def rel(path: Path, root: Path) -> str:
    return str(path.relative_to(root)).replace(os.sep, "/")


def safe_col_part(value: object) -> str:
    text = str(value or "").strip().lower()
    text = re.sub(r"[^a-z0-9]+", "_", text).strip("_")
    return text or "unknown"


def row_key(map_name: str, trajectory_id: int, frame_idx: int) -> Tuple[str, int, int]:
    return str(map_name), int(trajectory_id), int(frame_idx)


def ensure_row(rows: Dict[Tuple[str, int, int], dict], map_name: str, trajectory_id: int, frame_idx: int) -> dict:
    key = row_key(map_name, trajectory_id, frame_idx)
    row = rows.get(key)
    if row is None:
        row = {
            "map_name": str(map_name),
            "trajectory_id": int(trajectory_id),
            "frame_idx": int(frame_idx),
        }
        rows[key] = row
    return row


def add_value(row: dict, column: str, value: str) -> None:
    if not column or not value:
        return
    old = row.get(column, "")
    if not old or old == value:
        row[column] = value
        return
    # Preserve both paths if two files naturally map to the same compact column.
    i = 2
    while True:
        candidate = f"{column}_{i}"
        if candidate not in row or row[candidate] == value:
            row[candidate] = value
            return
        i += 1


def parse_scenario_map_filter(values: Iterable[str] | None) -> Set[str]:
    out: Set[str] = set()
    for value in values or []:
        for item in str(value or "").split(","):
            item = item.strip()
            if item:
                out.add(item)
    return out


def map_allowed(map_name: str, scenario_maps: Optional[Set[str]]) -> bool:
    return not scenario_maps or map_name in scenario_maps


def iter_trajectory_dirs(stage1_root: Path, scenario_maps: Optional[Set[str]] = None, map_hint: str | None = None) -> List[Path]:
    if not stage1_root.is_dir():
        return []
    out = []
    for path in sorted(stage1_root.iterdir()):
        if not path.is_dir() or COMPACT_TRAJ_RE.fullmatch(path.name) is None:
            continue
        map_name, _ = parse_trajectory_name(path.name, map_hint=map_hint)
        if map_allowed(map_name, scenario_maps):
            out.append(path)
    return out


def stage1_sensor_column(file_path: Path, traj_dir: Path) -> str | None:
    try:
        rel_parts = file_path.relative_to(traj_dir).parts
    except ValueError:
        return None
    if len(rel_parts) < 4 or rel_parts[0] != "nodes":
        return None
    node = rel_parts[1]
    modality = rel_parts[2]
    parts = ["stage1", node, modality]
    if modality in {"rgb", "depth", "semantic", "lidar_2d", "lidar_3d"} and len(rel_parts) >= 5:
        parts.append(rel_parts[3])
    else:
        stem = file_path.stem
        prefix = re.sub(r"_?\d{6}$", "", stem)
        if prefix and prefix not in {modality, f"{node}_{modality}"}:
            parts.append(prefix)
    return "_".join(safe_col_part(x) for x in parts)


def annotation_column(path: Path, traj_dir: Path) -> str | None:
    try:
        parts = path.relative_to(traj_dir).parts
    except ValueError:
        return None
    if len(parts) < 4 or parts[0] != "annotations":
        return None
    # annotations/camera_bbox2d/rsu_001/front/bbox.jsonl
    return "_".join(safe_col_part(x) for x in ("stage1_annotation",) + parts[1:-1])


def scan_stage1(dataset_root: Path, rows: Dict[Tuple[str, int, int], dict], scenario_maps: Optional[Set[str]] = None) -> None:
    stage1_root = dataset_root / "stage1_multimodal"
    map_hint = os.environ.get("DATASET_MAP_NAME", "").strip() or dataset_root.name
    traj_dirs = iter_trajectory_dirs(stage1_root, scenario_maps, map_hint=map_hint)
    for traj_dir in tqdm(traj_dirs, desc="scan stage1 trajectories", unit="traj"):
        map_name, traj_id = parse_trajectory_name(traj_dir.name, map_hint=map_hint)

        world_state_dir = traj_dir / "world_state"
        for ws in sorted(world_state_dir.glob("*.yaml")):
            frame_idx = frame_idx_from_name(ws.name)
            if frame_idx is None:
                continue
            row = ensure_row(rows, map_name, traj_id, frame_idx)
            row["stage1_world_state"] = rel(ws, dataset_root)

        for file_path in sorted((traj_dir / "nodes").rglob("*")) if (traj_dir / "nodes").is_dir() else []:
            if not file_path.is_file():
                continue
            frame_idx = frame_idx_from_name(file_path.stem)
            if frame_idx is None:
                continue
            col = stage1_sensor_column(file_path, traj_dir)
            if not col:
                continue
            row = ensure_row(rows, map_name, traj_id, frame_idx)
            add_value(row, col, rel(file_path, dataset_root))

        # Attach shared per-trajectory metadata paths to existing data rows.
        repeated = {
            "stage1_trajectory_meta": traj_dir / "trajectory_meta.yaml",
            "stage1_object_registry": traj_dir / "object_registry.yaml",
        }
        annotation_files = [p for p in sorted((traj_dir / "annotations").rglob("*.jsonl"))] if (traj_dir / "annotations").is_dir() else []
        traj_rows = [row for key, row in rows.items() if key[0] == map_name and key[1] == traj_id]
        for row in traj_rows:
            for col, path in repeated.items():
                if path.is_file():
                    row[col] = rel(path, dataset_root)
            for ann in annotation_files:
                col = annotation_column(ann, traj_dir)
                if col:
                    row[col] = rel(ann, dataset_root)


def static_xml_for_map(stage2_root: Path, map_name: str) -> Path | None:
    candidates = [stage2_root / "static" / f"{map_name}.xml"]
    if map_name.endswith("HD"):
        candidates.append(stage2_root / "static" / f"{map_name[:-2]}.xml")
    if map_name.endswith("HD_Opt"):
        candidates.append(stage2_root / "static" / f"{map_name[:-6]}.xml")
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def scan_stage2(dataset_root: Path, rows: Dict[Tuple[str, int, int], dict], scenario_maps: Optional[Set[str]] = None) -> None:
    stage2_root = dataset_root / "stage2_xml"
    if not stage2_root.is_dir():
        return
    map_hint = os.environ.get("DATASET_MAP_NAME", "").strip() or dataset_root.name
    traj_dirs = [p for p in sorted(stage2_root.iterdir()) if p.is_dir() and p.name != "static" and COMPACT_TRAJ_RE.fullmatch(p.name) and map_allowed(parse_trajectory_name(p.name, map_hint=map_hint)[0], scenario_maps)]
    for traj_dir in tqdm(traj_dirs, desc="scan stage2 trajectories", unit="traj"):
        map_name, traj_id = parse_trajectory_name(traj_dir.name, map_hint=map_hint)
        static_xml = static_xml_for_map(stage2_root, map_name)
        for dyn in sorted((traj_dir / "dynamic").glob("dynamic_*.xml")) if (traj_dir / "dynamic").is_dir() else []:
            frame_idx = frame_idx_from_name(dyn.name)
            if frame_idx is None:
                continue
            row = ensure_row(rows, map_name, traj_id, frame_idx)
            row["stage2_dynamic_xml"] = rel(dyn, dataset_root)
            if static_xml is not None:
                row["stage2_static_xml"] = rel(static_xml, dataset_root)


def scan_stage3(dataset_root: Path, rows: Dict[Tuple[str, int, int], dict], scenario_maps: Optional[Set[str]] = None) -> None:
    stage3_root = dataset_root / "stage3_radio"
    if not stage3_root.is_dir():
        return
    map_hint = os.environ.get("DATASET_MAP_NAME", "").strip() or dataset_root.name
    traj_dirs = [p for p in sorted(stage3_root.iterdir()) if p.is_dir() and COMPACT_TRAJ_RE.fullmatch(p.name) and map_allowed(parse_trajectory_name(p.name, map_hint=map_hint)[0], scenario_maps)]
    for traj_dir in tqdm(traj_dirs, desc="scan stage3 trajectories", unit="traj"):
        map_name, traj_id = parse_trajectory_name(traj_dir.name, map_hint=map_hint)
        for path_file in sorted((traj_dir / "paths").rglob("p_*.npz")) if (traj_dir / "paths").is_dir() else []:
            frame_idx = frame_idx_from_name(path_file.name)
            if frame_idx is None:
                continue
            try:
                parts = path_file.relative_to(traj_dir / "paths").parts
            except ValueError:
                continue
            if len(parts) < 3:
                continue
            link_type, link_id = parts[0], parts[1]
            row = ensure_row(rows, map_name, traj_id, frame_idx)
            row[f"stage3_path_{safe_col_part(link_type)}_{safe_col_part(link_id)}"] = rel(path_file, dataset_root)

        for channel_file in sorted((traj_dir / "channels").rglob("h_*.npz")) if (traj_dir / "channels").is_dir() else []:
            frame_idx = frame_idx_from_name(channel_file.name)
            if frame_idx is None:
                continue
            try:
                parts = channel_file.relative_to(traj_dir / "channels").parts
            except ValueError:
                continue
            if len(parts) < 4:
                continue
            cfg_id, link_type, link_id = parts[0], parts[1], parts[2]
            row = ensure_row(rows, map_name, traj_id, frame_idx)
            row[f"stage3_channel_{safe_col_part(cfg_id)}_{safe_col_part(link_type)}_{safe_col_part(link_id)}"] = rel(channel_file, dataset_root)


def collect_fieldnames(rows: Iterable[dict]) -> List[str]:
    fieldnames = list(BASE_FIELDS)
    for row in rows:
        for name in row.keys():
            if name not in fieldnames:
                fieldnames.append(name)
    return fieldnames


def write_csv(output_path: Path, rows: Dict[Tuple[str, int, int], dict]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ordered_keys = sorted(rows.keys(), key=lambda k: (k[0], k[1], k[2]))
    ordered_rows = [rows[k] for k in ordered_keys]
    fieldnames = collect_fieldnames(ordered_rows)
    tmp = output_path.with_suffix(output_path.suffix + f".tmp_{os.getpid()}")
    with open(tmp, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in tqdm(ordered_rows, desc="write samples.csv", unit="row"):
            writer.writerow({name: row.get(name, "") for name in fieldnames})
    os.replace(tmp, output_path)


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate final dataset samples.csv")
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--output", default=None, help="Output CSV path. Default: <dataset-root>/samples.csv")
    parser.add_argument("--scenario-map", "--map-name", dest="scenario_maps", action="append", default=[], help="Only index trajectories for this target scenario map. May be repeated or comma-separated. Default: all scenario maps.")
    parser.add_argument("--no-stage1", action="store_true", help="Omit stage1_multimodal from the index")
    parser.add_argument("--no-stage2", action="store_true", help="Omit stage2_xml from the index")
    parser.add_argument("--no-stage3", action="store_true", help="Omit stage3_radio from the index")
    args = parser.parse_args(argv)

    dataset_root = Path(args.dataset_root).resolve()
    if not dataset_root.is_dir():
        raise SystemExit(f"dataset root does not exist: {dataset_root}")
    output_path = Path(args.output).resolve() if args.output else dataset_root / "samples.csv"
    env_filter = os.environ.get("SAMPLES_CSV_SCENARIO_MAP", "")
    scenario_maps = parse_scenario_map_filter([*args.scenario_maps, env_filter])

    rows: Dict[Tuple[str, int, int], dict] = {}
    if not args.no_stage1:
        scan_stage1(dataset_root, rows, scenario_maps)
    if not args.no_stage2:
        scan_stage2(dataset_root, rows, scenario_maps)
    if not args.no_stage3:
        scan_stage3(dataset_root, rows, scenario_maps)

    write_csv(output_path, rows)
    filter_text = ",".join(sorted(scenario_maps)) if scenario_maps else "all"
    print(f"Wrote {len(rows)} rows to {output_path} (scenario_map={filter_text})", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
