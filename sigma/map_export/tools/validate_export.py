#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path

def main():
    p = argparse.ArgumentParser()
    p.add_argument('report', type=Path)
    a = p.parse_args()
    data = json.loads(a.report.read_text(encoding='utf-8'))
    failures = []
    if data.get('success') is not True: failures.append('success is not true')
    if data.get('checkpoint') is not False: failures.append('checkpoint is not false')
    if data.get('finish_reason') != 'official_level_export_completed': failures.append('unexpected finish_reason')
    if data.get('complete_map_audit_passed') is not True: failures.append('complete-map audit not passed')
    audit = data.get('world_audit')
    if not isinstance(audit, dict): failures.append('world_audit missing')
    result = data.get('export_result')
    output = None
    if not isinstance(result, dict):
        failures.append('export_result missing')
    else:
        if result.get('export_task_success') is not True: failures.append('export task failed')
        output = Path(str(result.get('output_file', '')))
        if not output.is_file(): failures.append('output missing')
        elif output.stat().st_size <= 0: failures.append('output empty')
    print('REPORT={}'.format(a.report.resolve()))
    if output is not None:
        print('OUTPUT={}'.format(output.resolve()))
        if output.is_file(): print('OUTPUT_SIZE_BYTES={}'.format(output.stat().st_size))
    if failures:
        for x in failures: print('VALIDATION_FAILURE={}'.format(x))
        print('EXPORT_VALIDATION_SUCCESS=0')
        return 1
    print('LOADED_LEVEL_COUNT={}'.format(audit.get('loaded_level_count')))
    print('ACTOR_COUNT={}'.format(audit.get('actor_count')))
    print('STATIC_MESH_COMPONENT_COUNT={}'.format(audit.get('static_mesh_component_count')))
    print('EXPORT_VALIDATION_SUCCESS=1')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
