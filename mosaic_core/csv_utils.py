"""CSV row normalization and locked upsert helpers shared by MOSAIC stages."""
from __future__ import annotations

import csv
import os
import time
from pathlib import Path


def stringify_csv_row(fieldnames, row):
    out = {}
    for name in fieldnames:
        value = (row or {}).get(name, "")
        if value is None:
            value = ""
        if name in {"trajectory_id", "frame_idx"} and value != "":
            try:
                value = int(str(value).strip())
            except Exception:
                pass
        out[name] = str(value)
    return out


def merged_fieldnames(existing, requested, rows, base_fields=None):
    out = []
    for name in list(base_fields or []) + list(existing or []) + list(requested or []):
        if name and name not in out:
            out.append(name)
    for row in rows or []:
        for name in row.keys():
            if name and name not in out:
                out.append(name)
    return out


def upsert_csv_rows(path, key_fields, fieldnames, rows, base_fields=None):
    rows = list(rows or [])
    if not rows:
        return path
    path_obj = Path(path)
    path_obj.parent.mkdir(parents=True, exist_ok=True)
    lock_dir = path_obj.with_name(path_obj.name + ".lock")
    deadline = time.time() + 60.0
    while True:
        try:
            lock_dir.mkdir()
            break
        except FileExistsError:
            if time.time() > deadline:
                raise TimeoutError(f"timed out waiting for samples.csv lock: {lock_dir}")
            time.sleep(0.05)
    try:
        existing_fields = []
        old_rows = []
        if path_obj.is_file():
            with open(path_obj, "r", encoding="utf-8", newline="") as f:
                reader = csv.DictReader(f)
                existing_fields = list(reader.fieldnames or [])
                old_rows = list(reader)
        all_fields = merged_fieldnames(
            existing_fields, fieldnames, rows, base_fields=base_fields or key_fields)
        by_key = {}
        for old in old_rows:
            norm_old = stringify_csv_row(all_fields, old)
            key = tuple(norm_old.get(k, "") for k in key_fields)
            by_key[key] = norm_old
        for row in rows:
            norm = stringify_csv_row(all_fields, row)
            key = tuple(norm.get(k, "") for k in key_fields)
            merged = by_key.get(key, {name: "" for name in all_fields})
            for name, value in norm.items():
                if value != "":
                    merged[name] = value
            by_key[key] = merged
        tmp = path_obj.with_name(path_obj.name + f".tmp_{os.getpid()}")
        with open(tmp, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=all_fields)
            writer.writeheader()
            for row in sorted(by_key.values(), key=lambda r: tuple(r.get(k, "") for k in key_fields)):
                writer.writerow(row)
        os.replace(tmp, path_obj)
        return path
    finally:
        try:
            lock_dir.rmdir()
        except OSError:
            pass
