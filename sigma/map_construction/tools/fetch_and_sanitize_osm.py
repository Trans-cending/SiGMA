#!/usr/bin/env python3
"""Fetch OSM XML and sanitize building labels for CARLA DTT.

CARLA 0.9.16 Digital Twin Tool can use OSM building names directly in
Unreal asset package paths. Valid OSM names such as "McDonald's" then
produce invalid UE package paths. This script keeps geometry intact and only
normalizes tag values that can become generated building asset names.
Run: python -m sigma.map_construction.tools.fetch_and_sanitize_osm --help
The SiGMA HTTP user agent identifies this independent project.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Iterable, List

ASSET_SAFE_RE = re.compile(r"[^A-Za-z0-9_]+")
TARGET_TAGS = {
    "name",
    "building",
    "brand",
    "operator",
}


def asset_safe(value: str) -> str:
    value = ASSET_SAFE_RE.sub("_", value.strip())
    value = re.sub(r"_+", "_", value).strip("_")
    return value or "unnamed"


def fetch(url: str, timeout: float) -> bytes:
    request = urllib.request.Request(
        url,
        headers={"User-Agent": "SiGMA-site-reconstruction/1.0"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


def tags_by_key(element: ET.Element) -> Dict[str, ET.Element]:
    result: Dict[str, ET.Element] = {}
    for tag in element.findall("tag"):
        key = tag.attrib.get("k")
        if key:
            result[key] = tag
    return result


def sanitize_elements(elements: Iterable[ET.Element]) -> List[Dict[str, Any]]:
    changes: List[Dict[str, Any]] = []
    for element in elements:
        tags = tags_by_key(element)
        if "building" not in tags:
            continue

        changed_tags = []
        for key in sorted(TARGET_TAGS):
            tag = tags.get(key)
            if tag is None:
                continue
            old_value = tag.attrib.get("v", "")
            new_value = asset_safe(old_value)
            if new_value == old_value:
                continue
            tag.set("v", new_value)
            changed_tags.append(
                {"key": key, "old": old_value, "new": new_value}
            )

        if changed_tags:
            changes.append(
                {
                    "element": element.tag,
                    "id": element.attrib.get("id", ""),
                    "changes": changed_tags,
                }
            )
    return changes


def sanitize_osm(raw: bytes) -> tuple[bytes, List[Dict[str, Any]]]:
    root = ET.fromstring(raw)
    changes = []
    changes.extend(sanitize_elements(root.findall("way")))
    changes.extend(sanitize_elements(root.findall("relation")))

    ET.indent(root, space=" ")
    output = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    return output + b"\n", changes


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fetch and sanitize OSM XML for CARLA DTT."
    )
    parser.add_argument("--url", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-output", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--attempts", type=int, default=4)
    parser.add_argument("--retry-delay", type=float, default=5.0)
    parser.add_argument("--cache-input", type=Path)
    parser.add_argument("--cache-output", type=Path)
    args = parser.parse_args()

    started_at = time.time()
    fetch_errors = []
    raw = None
    source = "network"
    for attempt in range(1, max(1, args.attempts) + 1):
        try:
            raw = fetch(args.url, args.timeout)
            break
        except Exception as exc:
            fetch_errors.append(
                {"attempt": attempt, "error": repr(exc)}
            )
            if attempt < max(1, args.attempts):
                time.sleep(args.retry_delay * attempt)
    if raw is None:
        if args.cache_input is None or not args.cache_input.is_file():
            raise RuntimeError(
                "OSM fetch failed and no cache is available: {}".format(
                    fetch_errors
                )
            )
        raw = args.cache_input.read_bytes()
        source = "cache"
    elif args.cache_output is not None:
        args.cache_output.parent.mkdir(parents=True, exist_ok=True)
        args.cache_output.write_bytes(raw)

    sanitized, changes = sanitize_osm(raw)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.raw_output is not None:
        args.raw_output.parent.mkdir(parents=True, exist_ok=True)
        args.raw_output.write_bytes(raw)
    args.output.write_bytes(sanitized)

    report = {
        "success": True,
        "url": args.url,
        "source": source,
        "fetch_errors": fetch_errors,
        "cache_input": str(args.cache_input) if args.cache_input else None,
        "cache_output": str(args.cache_output) if args.cache_output else None,
        "output": str(args.output),
        "raw_output": str(args.raw_output) if args.raw_output else None,
        "raw_bytes": len(raw),
        "sanitized_bytes": len(sanitized),
        "changed_element_count": len(changes),
        "changes": changes[:200],
        "change_count_truncated": max(0, len(changes) - 200),
        "elapsed_seconds": round(time.time() - started_at, 3),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    print("OSM_SANITIZE_SUCCESS=1")
    print(f"OSM_SOURCE={source}")
    print(f"OSM_FETCH_ERROR_COUNT={len(fetch_errors)}")
    print(f"OSM_SANITIZED_OUTPUT={args.output}")
    print(f"OSM_SANITIZE_CHANGED_ELEMENTS={len(changes)}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"OSM_SANITIZE_SUCCESS=0 error={exc!r}", file=sys.stderr)
        raise
