"""Export complete official CARLA source-map geometry from YAML.

Example: python -m sigma.map_export.cli -c configs/town03_export.yaml -o assets/local/town03
The input uses map_export or stage3.map_export settings documented in docs/map_export.md.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
from sigma.config import load_config
from .launcher import run_export


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-c', '--config', type=Path, required=True)
    parser.add_argument('-o', '--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = run_export(load_config(args.config), args.output)
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print('SiGMA map export error: ' + str(exc), file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
