"""Load strict YAML configuration for ``python -m sigma validate -c FILE``.

Paths inside configuration resolve relative to that YAML file. CLI --output
resolves relative to the invoking directory. Validation runs locally.
"""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import yaml


class ConfigurationError(ValueError):
    pass


def load_config(path):
    path = Path(path).resolve()
    try:
        with path.open(encoding="utf-8") as handle:
            config = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"Invalid YAML in {path}: {exc}") from exc
    if not isinstance(config, dict):
        raise ConfigurationError("YAML root must be a mapping")
    if any(not isinstance(key, str) for key in config):
        raise ConfigurationError("YAML root option names must be strings")
    config["_config_dir"] = str(path.parent)
    config["_config_file"] = str(path)
    return config


def validate_config(config, stages=None):
    errors = []
    if not isinstance(config, dict):
        return ["configuration must be a mapping"]
    errors += [f"unknown root option: {key}" for key in sorted(set(config) - {
        "schema_version", "site_id", "output_dir", "seed", "stage1", "stage2", "stage3",
        "ris", "_config_dir", "_config_file"})]
    if config.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if not isinstance(config.get("site_id"), str) or not config["site_id"].strip():
        errors.append("site_id must be a non-empty string")
    if type(config.get("seed", 0)) is not int or config.get("seed", 0) < 0:
        errors.append("seed must be a non-negative integer")
    selected = stages or [key for key in ("stage1", "stage2", "stage3") if key in config]
    if not selected:
        errors.append("configure at least one of stage1, stage2, stage3")
    for stage in selected:
        section = config.get(stage)
        if not isinstance(section, dict):
            errors.append(f"{stage} must be configured as a mapping")
            continue
        if stage == "stage1":
            from .stage1 import validate_stage1_config
            errors += validate_stage1_config(section)
        elif stage == "stage2":
            from .sensing import validate_stage2_config
            errors += validate_stage2_config(config)
        else:
            from .stage3 import validate_stage3_config
            errors += validate_stage3_config(section)
    if "ris" in config:
        from mosaic_core.ris import normalize_ris_config
        try:
            normalize_ris_config(config["ris"])
        except (ValueError, TypeError, KeyError) as exc:
            errors.append(f"ris: {exc}")
    return errors


def config_hash(config):
    public = {k: v for k, v in config.items() if not k.startswith("_")}
    return hashlib.sha256(json.dumps(public, sort_keys=True).encode()).hexdigest()


def output_path(config, override=None):
    if override:
        return Path(override).expanduser().resolve()
    path = Path(config.get("output_dir", "outputs/" + config["site_id"])).expanduser()
    return path.resolve() if path.is_absolute() else (Path(config["_config_dir"]) / path).resolve()
