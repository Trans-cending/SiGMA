"""CLI failure persistence and strict configuration regression checks."""
import json
import os
from pathlib import Path

import pytest
import yaml

from sigma.cli import main
from sigma.config import validate_config


def minimal_config():
    return {"schema_version": 1, "site_id": "testsite", "seed": 7,
            "stage3": {"blender": {"enabled": False}, "radio": {"enabled": False}, "radar": {"enabled": False}}}


def test_validation_typo_returns_error_without_creating_output(tmp_path, capsys):
    config = minimal_config()
    config["stage3"]["radio"]["num_subcarier"] = 64
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(config))
    assert main(["stage3", "-c", str(path), "-o", str(tmp_path / "outputs")]) == 2
    assert "unknown stage3.radio option: num_subcarier" in capsys.readouterr().err
    assert not (tmp_path / "outputs").exists()


@pytest.mark.parametrize("option,value", [
    ("arrays", None), ("arrays", []), ("arrays", [[1, True, 16, 1]]),
    ("num_subcarriers", 1.5), ("num_subcarriers", True),
    ("frequency_hz", float("nan")), ("bandwidth_hz", float("inf")),
    ("enabled", "false"), ("radio_map", {"enabled": True, "center": None, "size": [1, 1]}),
    ("radio_map", {"enabeld": True}),
])
def test_stage3_malformed_values_report_validation_errors(option, value):
    config = minimal_config()
    config["stage3"]["radio"][option] = value
    assert validate_config(config), f"malformed {option} was accepted"


def test_cli_stage_failure_keeps_intermediates_and_writes_manifest(tmp_path, monkeypatch, capsys):
    import sigma.stage3
    output = tmp_path / "output"
    reusable = output / "stage2_xml" / "000001" / "scene.xml"
    reusable.parent.mkdir(parents=True)
    reusable.write_text("<scene/>")
    config_file = tmp_path / "stage3.yaml"
    config_file.write_text(yaml.safe_dump(minimal_config()))
    def fail(config, output_dir, seed):
        assert output_dir == output
        assert seed == 19
        raise RuntimeError("fixture RT backend is unavailable")
    monkeypatch.setattr(sigma.stage3, "run_stage3", fail)
    assert main(["stage3", "-c", str(config_file), "-o", str(output), "--seed", "19"]) == 2
    assert "fixture RT backend is unavailable" in capsys.readouterr().err
    report = json.loads((output / "manifests" / "stage3.json").read_text())
    assert report["status"] == "failed"
    assert report["seed"] == 19
    assert report["error_type"] == "RuntimeError"
    assert len(report["config_hash"]) == 64
    assert reusable.read_text() == "<scene/>"


def test_output_path_is_relative_to_yaml_directory(tmp_path):
    from sigma.config import load_config, output_path
    config = minimal_config()
    config["output_dir"] = "mydata/site"
    path = tmp_path / "nested" / "site.yaml"
    path.parent.mkdir()
    path.write_text(yaml.safe_dump(config))
    loaded = load_config(path)
    assert output_path(loaded) == (path.parent / "mydata/site").resolve()


def test_ambient_physics_and_old_output_paths_do_not_override_yaml(monkeypatch):
    from sigma.stage3 import clean_environment
    from sigma.stage1 import _environment
    for name in ("PATH_GEOMETRY_DEDUP_ENABLED", "PATH_DEDUP_DELAY_TOL_S", "DYNAMIC_SUBDIR",
                 "WORK_SUBDIR", "MODEL_EXTRA_ROT_DEG", "MOSAIC_MAP_NAME", "TX_ANTENNA_OFFSET_CARLA_M"):
        monkeypatch.setenv(name, "stale_previous_run")
    monkeypatch.setenv("BLENDER_USER_SCRIPTS", "/external/matching_addons")
    env = clean_environment()
    for name in ("PATH_GEOMETRY_DEDUP_ENABLED", "PATH_DEDUP_DELAY_TOL_S", "DYNAMIC_SUBDIR",
                 "WORK_SUBDIR", "MODEL_EXTRA_ROT_DEG", "MOSAIC_MAP_NAME", "TX_ANTENNA_OFFSET_CARLA_M"):
        assert name not in env
    assert env["BLENDER_USER_SCRIPTS"] == "/external/matching_addons"
    assert "MOSAIC_MAP_NAME" not in _environment()


def test_cli_negative_seed_is_rejected(tmp_path, capsys):
    path = tmp_path / "site.yaml"
    path.write_text(yaml.safe_dump(minimal_config()))
    assert main(["stage3", "-c", str(path), "--seed", "-1"]) == 2
    assert "seed must be non-negative" in capsys.readouterr().err


def test_invalid_yaml_has_clean_cli_error(tmp_path, capsys):
    path = tmp_path / "broken.yaml"
    path.write_text("site_id: [unfinished")
    assert main(["validate", "-c", str(path)]) == 2
    error = capsys.readouterr().err
    assert "Invalid YAML" in error
    assert "Traceback" not in error


def test_stage3_resume_checks_content_and_seed(tmp_path):
    from sigma.stage3 import run_stage3
    config = minimal_config()
    state = tmp_path / "stage1_multimodal" / "000001" / "world_state" / "world_state_000000.yaml"
    state.parent.mkdir(parents=True)
    state.write_text("site_id: testsite\nframe: 0\n")
    xml = tmp_path / "stage2_xml" / "000001" / "dynamic_000000.xml"
    xml.parent.mkdir(parents=True)
    xml.write_text("<scene/>\n")
    run_stage3(config, tmp_path, 7)
    # Exporters may rewrite unchanged files. Only changed physics/content
    # should invalidate reusing the same frozen scene products.
    stat = xml.stat()
    os.utime(xml, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000))
    run_stage3(config, tmp_path, 7)
    with pytest.raises(RuntimeError, match="changed"):
        run_stage3(config, tmp_path, 8)
    state.write_text("site_id: testsite\nframe: 1\n")
    with pytest.raises(RuntimeError, match="changed"):
        run_stage3(config, tmp_path, 7)
