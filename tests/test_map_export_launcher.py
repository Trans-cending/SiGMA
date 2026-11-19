"""Validate export boundaries and durable-report rules using process fixtures."""
import copy
import json
import time
from pathlib import Path

import pytest

from sigma.map_export import run_export, validate_export_config
from sigma.map_export.tools.supervise_ue_export import valid_report


def test_export_rejects_unknown_options_and_invalid_types():
    errors = validate_export_config({"enabled": True, "prompts": True, "format": "glb", "meshify_curves": "false", "minimum_actor_count": 0})
    assert len(errors) == 4
    assert any("unknown" in item for item in errors)


def test_export_failure_is_reported_and_config_is_not_mutated(tmp_path):
    config = {"site_id": "site", "stage3": {"map_export": {"enabled": True, "map_name": "SiteMap", "carla_root": str(tmp_path / "missing_carla"), "unreal_editor_cmd": str(tmp_path / "missing_editor")}}}
    before = copy.deepcopy(config)
    with pytest.raises(FileNotFoundError, match="CARLA source project"):
        run_export(config, tmp_path / "export")
    assert config == before
    manifest = json.loads((tmp_path / "export/manifest.json").read_text())
    assert manifest["status"] == "failed"
    assert manifest["site_id"] == "site"
    assert manifest["error_type"] == "FileNotFoundError"


def test_source_supervisor_rejects_incomplete_or_mismatched_exports(tmp_path):
    report_path = tmp_path / "report.json"
    output = tmp_path / "scene.fbx"
    output.write_bytes(b"test-only FBX placeholder")
    report_path.write_text("{}")
    started = time.time() - 1
    data = {"success": True, "checkpoint": False, "finish_reason": "official_level_export_completed", "failure_count": 0, "complete_map_audit_passed": True, "export_result": {"export_task_success": True, "output_file": str(output)}}
    assert valid_report(data, report_path, output, started) == (True, "valid")
    data["complete_map_audit_passed"] = False
    assert valid_report(data, report_path, output, started)[0] is False
    data["complete_map_audit_passed"] = True
    data["export_result"]["output_file"] = str(tmp_path / "other.fbx")
    assert valid_report(data, report_path, output, started)[1] == "output_path_mismatch"
