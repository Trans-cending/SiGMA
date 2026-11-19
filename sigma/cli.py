"""Unified CLI, with durable failure reports and explicit deterministic inputs.

Examples: ``python -m sigma doctor``; ``python -m sigma validate -c FILE``;
``python -m sigma pipeline -c FILE --output outputs/site --seed 42``.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import random
import sys
import traceback

from .config import ConfigurationError, config_hash, load_config, output_path, validate_config


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False, default=str) + "\n", encoding="utf-8")
    tmp.replace(path)


def execute(stage, config, output, seed):
    report = {"stage": stage, "seed": seed, "config_hash": config_hash(config),
              "started_at": datetime.now(timezone.utc).isoformat(), "status": "running"}
    path = output / "manifests" / f"{stage}.json"
    write_json(path, report)
    try:
        if stage == "stage1":
            from .stage1 import run_stage1
            result = run_stage1(config, output / "stage1", seed)
        elif stage == "stage2":
            from .sensing import run_stage2
            result = run_stage2(config, output, seed)
        else:
            from .stage3 import run_stage3
            result = run_stage3(config, output, seed)
        report.update(status="completed", result=result)
    except Exception as exc:
        report.update(status="failed", error_type=type(exc).__name__, error=str(exc),
                      traceback=traceback.format_exc())
        write_json(path, report)
        raise
    write_json(path, report)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="SiGMA site-grounded multi-modal ISAC generation")
    sub = parser.add_subparsers(dest="command", required=True)
    doctor_parser = sub.add_parser("doctor", help="inspect packages and configured runtimes")
    doctor_parser.add_argument("-c", "--config", type=Path)
    doctor_parser.add_argument("--connect-carla", action="store_true")
    val = sub.add_parser("validate", help="validate YAML structure and configuration")
    val.add_argument("-c", "--config", required=True, type=Path)
    for name in ("stage1", "stage2", "stage3", "pipeline"):
        command = sub.add_parser(name, help=f"run {name} with YAML configuration")
        command.add_argument("-c", "--config", required=True, type=Path)
        command.add_argument("-o", "--output", type=Path)
        command.add_argument("--seed", type=int)
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config) if args.config else None
        if args.command == "doctor":
            if config:
                errors = validate_config(config)
                if errors:
                    raise ConfigurationError("\n".join(errors))
            from .doctor import doctor
            result = doctor(config, args.connect_carla)
            print(json.dumps(result, indent=2, ensure_ascii=False))
            return 0 if result["ok"] else 1
        stages = ["stage1", "stage2", "stage3"] if args.command == "pipeline" else [args.command]
        errors = validate_config(config, None if args.command == "validate" else stages)
        if errors:
            raise ConfigurationError("\n".join(errors))
        if args.command == "validate":
            print("Configuration valid")
            return 0
        seed = args.seed if args.seed is not None else config.get("seed", 0)
        if seed < 0:
            raise ConfigurationError("seed must be non-negative")
        config["seed"] = seed
        random.seed(seed)
        import numpy as np
        np.random.seed(seed)
        output = output_path(config, args.output)
        for stage in stages:
            result = execute(stage, config, output, seed)
            if args.command == "pipeline" and stage == "stage1" and (not result.get("geometry_built", False) or result.get("cook_required", False)):
                raise RuntimeError("Stage I only prepared inputs; pipeline requires a built, cooked, imported CARLA 0.9.16 map")
        print(f"Completed {args.command}: {output}")
        return 0
    except (ConfigurationError, OSError, RuntimeError, ValueError, ImportError, KeyError) as exc:
        print(f"SiGMA error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
