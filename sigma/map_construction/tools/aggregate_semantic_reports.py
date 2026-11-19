#!/usr/bin/env python3
"""Aggregate batched semantic-retag and verification reports."""

from __future__ import annotations

import json
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Set


def main() -> int:
    if len(sys.argv) < 3:
        raise SystemExit(
            "usage: aggregate_semantic_reports.py "
            "OUTPUT REPORT..."
        )

    output = Path(sys.argv[1])
    report_paths = [
        Path(value)
        for value in sys.argv[2:]
    ]

    reports: List[Dict[str, Any]] = []
    retag_reports: List[Dict[str, Any]] = []
    verify_reports: List[Dict[str, Any]] = []

    source_counts: Counter[str] = Counter()
    replacement_counts: Counter[str] = Counter()
    canonical_counts: Counter[str] = Counter()
    failures = []
    unmatched_samples = []

    actors_per_level: Dict[str, int] = {}
    processed_actors = 0
    static_mesh_components = 0
    classified_components = 0
    unmatched_components = 0

    for path in report_paths:
        data = json.loads(
            path.read_text(encoding="utf-8")
        )
        data["report_path"] = str(path)
        reports.append(data)

        target_level = str(
            data.get("target_level", "")
        )
        actors_per_level[target_level] = max(
            actors_per_level.get(target_level, 0),
            int(
                data.get(
                    "all_loaded_actor_count",
                    0,
                )
                or 0
            ),
        )

        mode = str(
            data.get("mode", "retag")
        ).lower()

        if mode == "verify":
            verify_reports.append(data)
            continue

        retag_reports.append(data)
        source_counts.update(
            data.get("source_counts", {})
        )
        replacement_counts.update(
            data.get("replacement_counts", {})
        )
        canonical_counts.update(
            data.get(
                "already_canonical_counts",
                {},
            )
        )
        failures.extend(
            data.get("failures", [])
        )

        processed_actors += int(
            data.get(
                "processed_actor_count",
                0,
            )
            or 0
        )
        static_mesh_components += int(
            data.get(
                "static_mesh_component_count",
                0,
            )
            or 0
        )
        classified_components += int(
            data.get(
                "classified_component_count",
                0,
            )
            or 0
        )
        unmatched_components += int(
            data.get(
                "unmatched_component_count",
                0,
            )
            or 0
        )

        for item in data.get(
            "unmatched_context_samples",
            [],
        ):
            if len(unmatched_samples) >= 300:
                break
            unmatched_samples.append(item)

    retag_levels: Set[str] = {
        str(item.get("target_level", ""))
        for item in retag_reports
    }
    verify_levels: Set[str] = {
        str(item.get("target_level", ""))
        for item in verify_reports
    }

    tagged_counts = {
        key: (
            int(replacement_counts.get(key, 0))
            + int(canonical_counts.get(key, 0))
        )
        for key in (
            set(source_counts)
            | set(replacement_counts)
            | set(canonical_counts)
        )
    }

    remaining_noncanonical_by_level = {
        str(item.get("target_level", "")):
            int(
                item.get(
                    "remaining_noncanonical_count",
                    -1,
                )
            )
        for item in verify_reports
    }

    class_checks = {}
    for semantic_class, source_count in (
        source_counts.items()
    ):
        if int(source_count) <= 0:
            continue
        class_checks[
            semantic_class + "_source_was_tagged"
        ] = (
            tagged_counts.get(semantic_class, 0)
            > 0
        )

    checks = {
        "all_pass_processes_success":
            all(
                bool(item.get("success"))
                for item in reports
            ),
        "no_component_failures":
            len(failures) == 0,
        "loaded_actors_discovered":
            sum(actors_per_level.values()) > 0,
        "all_loaded_actors_processed_once":
            processed_actors
            == sum(actors_per_level.values()),
        "static_mesh_components_discovered":
            static_mesh_components > 0,
        "semantic_components_classified":
            classified_components > 0,
        "road_components_tagged":
            tagged_counts.get("Road", 0) > 0,
        "verification_level_coverage":
            bool(retag_levels)
            and retag_levels == verify_levels,
        "no_remaining_noncanonical_components":
            bool(verify_reports)
            and all(
                value == 0
                for value in (
                    remaining_noncanonical_by_level.values()
                )
            ),
    }
    checks.update(class_checks)

    if not checks[
        "all_pass_processes_success"
    ]:
        diagnosis = (
            "At least one semantic batch or verification "
            "process failed."
        )
    elif not checks[
        "verification_level_coverage"
    ]:
        diagnosis = (
            "Not every retagged Level received a final "
            "verification pass."
        )
    elif not checks[
        "no_remaining_noncanonical_components"
    ]:
        diagnosis = (
            "Final verification found classifiable mesh "
            "references outside canonical CARLA semantic "
            "folders."
        )
    elif tagged_counts.get("Road", 0) == 0:
        diagnosis = (
            "No Road component was tagged."
        )
    else:
        diagnosis = (
            "Batched semantic retag and final verification "
            "completed."
        )

    result = {
        "release_version": "1.0.6",
        "runner":
            "batched-canonical-root-report-supervised-semantic-retag-aggregate",
        "reports": reports,
        "pass_count": len(reports),
        "retag_pass_count":
            len(retag_reports),
        "verification_pass_count":
            len(verify_reports),
        "level_count": len(retag_levels),
        "actors_per_level":
            actors_per_level,
        "total_loaded_actor_count":
            sum(actors_per_level.values()),
        "total_processed_actor_count":
            processed_actors,
        "total_static_mesh_component_count":
            static_mesh_components,
        "total_classified_component_count":
            classified_components,
        "total_unmatched_component_count":
            unmatched_components,
        "total_source_counts":
            dict(source_counts),
        "total_replacement_counts":
            dict(replacement_counts),
        "total_already_canonical_counts":
            dict(canonical_counts),
        "total_tagged_counts":
            tagged_counts,
        "remaining_noncanonical_by_level":
            remaining_noncanonical_by_level,
        "canonical_policy":
            "complete_carla_class_root",
        "failure_count": len(failures),
        "failures": failures[:300],
        "unmatched_context_samples":
            unmatched_samples,
        "checks": checks,
        "diagnosis": diagnosis,
        "success": all(checks.values()),
        "finish_reason":
            "batched_semantic_retag_completed",
        "finished_at": time.time(),
    }

    output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )
    output.write_text(
        json.dumps(
            result,
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    for key, value in sorted(checks.items()):
        print(
            "CHECK_{}={}".format(
                key.upper(),
                int(bool(value)),
            )
        )

    print(
        "TOTAL_LOADED_ACTORS={}".format(
            result["total_loaded_actor_count"]
        )
    )
    print(
        "TOTAL_PROCESSED_ACTORS={}".format(
            processed_actors
        )
    )
    print(
        "TOTAL_STATIC_MESH_COMPONENTS={}".format(
            static_mesh_components
        )
    )
    print(
        "TOTAL_CLASSIFIED_COMPONENTS={}".format(
            classified_components
        )
    )
    print(
        "TOTAL_REPLACEMENT_COUNTS={}".format(
            dict(replacement_counts)
        )
    )
    print(
        "TOTAL_ALREADY_CANONICAL_COUNTS={}".format(
            dict(canonical_counts)
        )
    )
    print(
        "REMAINING_NONCANONICAL_BY_LEVEL={}".format(
            remaining_noncanonical_by_level
        )
    )
    print("DIAGNOSIS={}".format(diagnosis))
    print(
        "SEMANTIC_RETAG_SUCCESS={}".format(
            int(bool(result["success"]))
        )
    )

    return (
        0
        if result["success"]
        else 32
    )


if __name__ == "__main__":
    raise SystemExit(main())
