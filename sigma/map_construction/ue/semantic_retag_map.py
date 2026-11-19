# -*- coding: utf-8 -*-
"""Run all semantic Level passes inside one long-lived UE4Editor process.

The established semantic_retag_level.py remains the only implementation of
classification, asset duplication, Level saving, and pass-report generation.
This runner only changes orchestration: it loads each Level, invokes that
script for every actor batch, and writes a phase heartbeat/final report.
"""

from __future__ import print_function

import gc
import json
import os
import runpy
import time
import traceback
from pathlib import Path

import unreal


MAP_NAME = os.environ["MOSAIC_MAP_NAME"]
MANIFEST_PATH = Path(os.environ["MOSAIC_SEMANTIC_LEVEL_MANIFEST"])
PHASE_REPORT_PATH = Path(os.environ["MOSAIC_SEMANTIC_PHASE_REPORT"])
LEVEL_SCRIPT = Path(os.environ["MOSAIC_SEMANTIC_LEVEL_SCRIPT"])
REPORTS_DIR = Path(os.environ["MOSAIC_SEMANTIC_REPORTS_DIR"])
MODE = os.environ["MOSAIC_RETAG_MODE"].strip().lower()
BATCH_SIZE = int(os.environ.get("MOSAIC_RETAG_ACTOR_BATCH_SIZE", "240"))
CHECKPOINT_EVERY = int(os.environ.get("MOSAIC_RETAG_CHECKPOINT_EVERY", "25"))

if MODE not in ("retag", "verify"):
    raise RuntimeError("MOSAIC_RETAG_MODE must be retag or verify")
if BATCH_SIZE < 1:
    raise RuntimeError("MOSAIC_RETAG_ACTOR_BATCH_SIZE must be positive")
if CHECKPOINT_EVERY < 1:
    raise RuntimeError("MOSAIC_RETAG_CHECKPOINT_EVERY must be positive")


def log(message):
    unreal.log("[MOSAIC_SEMANTIC_MAP] " + str(message))


def package_path(object_path):
    return str(object_path).split(".", 1)[0]


def current_world_package():
    world = unreal.EditorLevelLibrary.get_editor_world()
    if world is None:
        raise RuntimeError("Editor world is unavailable")
    return package_path(world.get_path_name())


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json_atomic(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, indent=2, ensure_ascii=False)
    last_error = None
    for attempt in range(1, 6):
        temporary = path.with_suffix(
            path.suffix + ".tmp.{}".format(os.getpid())
        )
        try:
            temporary.write_text(text, encoding="utf-8")
            os.replace(str(temporary), str(path))
            return
        except OSError as exc:
            last_error = exc
            try:
                temporary.unlink()
            except OSError:
                pass
            time.sleep(float(attempt))
    raise RuntimeError(
        "Failed to write phase report after retries: {}".format(last_error)
    )


def load_manifest():
    data = read_json(MANIFEST_PATH)
    levels = data.get("levels")
    if data.get("map_name") != MAP_NAME:
        raise RuntimeError("Semantic manifest map_name mismatch")
    if not isinstance(levels, list) or not levels:
        raise RuntimeError("Semantic manifest must contain Levels")

    normalized = []
    for index, item in enumerate(levels):
        if not isinstance(item, dict):
            raise RuntimeError("Semantic manifest Level is not an object")
        package = str(item.get("package", "")).strip()
        label = str(item.get("label", "")).strip()
        if not package or not label:
            raise RuntimeError(
                "Semantic manifest Level {} lacks package/label".format(index)
            )
        normalized.append({"package": package, "label": label})
    return normalized


LEVELS = load_manifest()
STATE = {
    "release_version": "1.0.8",
    "runner": "persistent-editor-semantic-map-phase",
    "map_name": MAP_NAME,
    "mode": MODE,
    "expected_level_count": len(LEVELS),
    "completed_level_count": 0,
    "batch_size": BATCH_SIZE,
    "checkpoint_every": CHECKPOINT_EVERY,
    "current_level": None,
    "current_label": None,
    "current_actor_offset": None,
    "report_paths": [],
    "failure_count": 0,
    "failures": [],
    "checkpoint": True,
    "success": False,
    "started_at": time.time(),
}


def save_phase_report():
    STATE["updated_at"] = time.time()
    write_json_atomic(PHASE_REPORT_PATH, STATE)


def load_level(level_package):
    loaded = current_world_package()
    if loaded.lower() == level_package.lower():
        return

    log("LOAD_LEVEL={}".format(level_package))
    result = unreal.EditorLevelLibrary.load_level(level_package)
    if result is False:
        raise RuntimeError("Failed to load Level: {}".format(level_package))

    loaded = current_world_package()
    if loaded.lower() != level_package.lower():
        raise RuntimeError(
            "Loaded Level mismatch: expected {}, got {}".format(
                level_package, loaded
            )
        )


def release_level_references():
    gc.collect()
    collect = getattr(unreal.SystemLibrary, "collect_garbage", None)
    if callable(collect):
        collect()


def validate_pass_report(path, level_package, label, mode, offset):
    data = read_json(path)
    checks = {
        "success": data.get("success") is True,
        "checkpoint": data.get("checkpoint") is False,
        "target_level":
            str(data.get("target_level", "")).lower()
            == level_package.lower(),
        "target_label": str(data.get("target_label", "")) == label,
        "mode": str(data.get("mode", "")).lower() == mode,
        "actor_offset": int(data.get("actor_offset", -1) or 0) == offset,
        "failure_count": int(data.get("failure_count", -1) or 0) == 0,
    }
    if mode == "verify":
        checks["remaining_noncanonical"] = (
            int(data.get("remaining_noncanonical_count", -1)) == 0
        )
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise RuntimeError(
            "Invalid semantic pass report {}: {}".format(
                path, ",".join(failed)
            )
        )
    return data


def run_level_script(level_package, label, mode, offset, report_path):
    os.environ["MOSAIC_TARGET_LEVEL_PACKAGE"] = level_package
    os.environ["MOSAIC_TARGET_LEVEL_LABEL"] = label
    os.environ["MOSAIC_LEVEL_RETAG_REPORT"] = str(report_path)
    os.environ["MOSAIC_RETAG_MODE"] = mode
    os.environ["MOSAIC_RETAG_ACTOR_OFFSET"] = str(offset)
    os.environ["MOSAIC_RETAG_ACTOR_LIMIT"] = str(BATCH_SIZE)
    os.environ["MOSAIC_RETAG_CHECKPOINT_EVERY"] = str(CHECKPOINT_EVERY)

    try:
        report_path.unlink()
    except OSError:
        pass

    log("RUN_LEVEL_PACKAGE={}".format(level_package))
    log("RUN_LEVEL_MODE={}".format(mode))
    log("RUN_LEVEL_ACTOR_OFFSET={}".format(offset))
    log("RUN_LEVEL_REPORT={}".format(report_path))

    namespace = runpy.run_path(
        str(LEVEL_SCRIPT),
        run_name="__mosaic_semantic_level_pass__",
    )
    namespace.clear()
    del namespace

    if not report_path.is_file():
        raise RuntimeError(
            "Semantic Level pass did not produce report: {}".format(
                report_path
            )
        )
    return validate_pass_report(
        report_path,
        level_package,
        label,
        mode,
        offset,
    )


def run_retag_level(level_package, label):
    offset = 0
    while True:
        report = REPORTS_DIR / (
            "{}_retag_{:07d}.json".format(label, offset)
        )
        STATE["current_actor_offset"] = offset
        save_phase_report()
        data = run_level_script(
            level_package, label, "retag", offset, report
        )
        STATE["report_paths"].append(str(report))
        save_phase_report()

        total = int(data.get("all_loaded_actor_count", 0) or 0)
        processed = int(data.get("processed_actor_count", 0) or 0)
        if processed <= 0 and offset < total:
            raise RuntimeError(
                "Semantic batch made no progress at {} offset {}".format(
                    level_package, offset
                )
            )
        next_offset = offset + BATCH_SIZE
        if next_offset >= total:
            return
        offset = next_offset


def run_verify_level(level_package, label):
    report = REPORTS_DIR / "{}_verify.json".format(label)
    STATE["current_actor_offset"] = 0
    save_phase_report()
    run_level_script(level_package, label, "verify", 0, report)
    STATE["report_paths"].append(str(report))
    save_phase_report()


def run_phase():
    REPORTS_DIR.mkdir(parents=True, exist_ok=True)
    save_phase_report()

    for index, level in enumerate(LEVELS):
        level_package = level["package"]
        label = level["label"]
        STATE["current_level"] = level_package
        STATE["current_label"] = label
        STATE["current_actor_offset"] = 0
        save_phase_report()

        load_level(level_package)
        if MODE == "retag":
            run_retag_level(level_package, label)
        else:
            run_verify_level(level_package, label)

        STATE["completed_level_count"] = index + 1
        STATE["current_level"] = None
        STATE["current_label"] = None
        STATE["current_actor_offset"] = None
        save_phase_report()
        release_level_references()

    STATE["checkpoint"] = False
    STATE["success"] = True
    STATE["finish_reason"] = "persistent_semantic_{}_completed".format(MODE)
    STATE["finished_at"] = time.time()
    save_phase_report()
    log("PHASE_REPORT={}".format(PHASE_REPORT_PATH))
    log("PHASE_SUCCESS=1")


try:
    run_phase()
except Exception:
    STATE["failure_count"] = int(STATE.get("failure_count", 0)) + 1
    STATE["failures"].append(
        {
            "level": STATE.get("current_level"),
            "offset": STATE.get("current_actor_offset"),
            "exception": traceback.format_exc(),
        }
    )
    STATE["checkpoint"] = False
    STATE["success"] = False
    STATE["finish_reason"] = "persistent_semantic_{}_failed".format(MODE)
    STATE["finished_at"] = time.time()
    save_phase_report()
    unreal.log_error(STATE["failures"][-1]["exception"])
    raise
