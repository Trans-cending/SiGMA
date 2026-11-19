"""Inspect runtime availability through lightweight dependency checks.

``python -m sigma doctor [-c FILE] [--connect-carla]`` reports actual package
versions in each configured interpreter. Missing dependencies yield exit code 1.
"""
from __future__ import annotations
from importlib import metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys


def inspect_python(interpreter, packages):
    script = """import importlib.metadata as m,json,sys
result={'python':sys.version.split()[0],'executable':sys.executable,'packages':{}}
for p in sys.argv[1:]:
 try: result['packages'][p]=m.version(p)
 except m.PackageNotFoundError: result['packages'][p]=None
print(json.dumps(result))
"""
    try:
        run = subprocess.run([str(interpreter), "-c", script, *packages], text=True,
                             capture_output=True, timeout=20, check=True)
        return json.loads(run.stdout)
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        return {"error": str(exc), "packages": {p: None for p in packages}}


def doctor(config=None, connect=False):
    from .stage3 import executable, resolve_path
    config = config or {}
    report = {"python": sys.version.split()[0], "checks": []}
    def check(name, ok, detail):
        report["checks"].append({"name": name, "ok": bool(ok), "detail": detail})
    check("python", (3, 10) <= sys.version_info[:2] < (3, 12), sys.version)
    for name in ("numpy", "PyYAML"):
        try:
            check(name, True, metadata.version(name))
        except metadata.PackageNotFoundError:
            check(name, False, "not installed")
    if "stage1" in config:
        stage1 = config["stage1"]
        runtime = inspect_python(sys.executable, ["shapely", "pyproj"])
        check("geographic", all(runtime["packages"].values()), runtime)
        if stage1.get("mode", "build") == "build":
            editor = stage1.get("unreal_editor")
            check("unreal_editor", editor and resolve_path(editor, config).is_file(), editor)
            root = stage1.get("carla_root")
            check("carla_source", root and resolve_path(root, config).is_dir(), root)
    if "stage2" in config:
        runtime = inspect_python(sys.executable, ["carla", "Pillow", "pyproj"])
        check("sensing", runtime["packages"].get("carla") == "0.9.16" and
              all(runtime["packages"].values()), runtime)
        if connect:
            try:
                import carla
                from .sensing.runtime import check_versions
                client = carla.Client(config["stage2"].get("host", "127.0.0.1"), int(config["stage2"].get("port", 2000)))
                client.set_timeout(float(config["stage2"].get("timeout", 10)))
                check_versions(client)
                check("carla_server", True, {"client": client.get_client_version(),
                                             "server": client.get_server_version()})
            except Exception as exc:
                check("carla_server", False, str(exc))
    if "stage3" in config:
        stage3 = config["stage3"]
        if stage3.get("blender", {}).get("enabled", True):
            value = executable(stage3.get("blender", {}).get("executable", "blender"), config)
            check("blender", bool(shutil.which(value)), value)
        for name, enabled, interpreter, version in (
            ("communication", stage3.get("radio", {}).get("enabled", True),
             stage3.get("communication_python"), "2.0.1"),
            ("radar", stage3.get("radar", {}).get("enabled", False), stage3.get("radar_python"), "2.2.0")):
            if enabled:
                packages = ["sionna-rt", "mitsuba", "numpy", "PyYAML"]
                if name == "radar":
                    packages.append("trimesh")
                runtime = inspect_python(executable(interpreter, config), packages)
                check(name, runtime["packages"].get("sionna-rt") == version and
                      all(runtime["packages"].values()), runtime)
    report["ok"] = all(x["ok"] for x in report["checks"])
    return report
