#!/usr/bin/env python3
"""Supervise UE4Editor-Cmd with report-first success handling."""
from __future__ import annotations
import argparse, json, os, signal, subprocess, time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

FATAL_PATTERNS = (
    "segmentation fault", "signal 11", "assertion failed", "fatal error",
    "unhandled exception", "core dumped", "out of memory",
)
KEY_PATTERNS = (
    "[MOSAIC_UE_EXPORT]", "LogPython:", "LogExit:", "Fatal",
    "Assertion", "Segmentation fault", "signal 11",
)

def args_ns():
    p = argparse.ArgumentParser()
    p.add_argument("--report", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--process-log", required=True, type=Path)
    p.add_argument("--hard-timeout-seconds", type=float, default=21600)
    p.add_argument("--report-exit-grace-seconds", type=float, default=60)
    p.add_argument("--term-grace-seconds", type=float, default=20)
    p.add_argument("--report-settle-seconds", type=float, default=10)
    p.add_argument("--stream-mode", choices=("key", "full"), default="key")
    p.add_argument("command", nargs=argparse.REMAINDER)
    a = p.parse_args()
    if a.command and a.command[0] == "--":
        a.command = a.command[1:]
    if not a.command:
        p.error("missing command after --")
    return a

def read_report(path: Path) -> Optional[Dict[str, Any]]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None

def valid_report(report, report_path, output_path, started_wall) -> Tuple[bool, str]:
    if report is None:
        return False, "report_missing_or_invalid"
    try:
        if report_path.stat().st_mtime < started_wall - 1:
            return False, "report_is_stale"
    except OSError:
        return False, "report_stat_failed"
    if report.get("success") is not True:
        return False, "success_is_not_true"
    if report.get("checkpoint") is not False:
        return False, "checkpoint_is_not_false"
    if report.get("finish_reason") != "official_level_export_completed":
        return False, "unexpected_finish_reason"
    if int(report.get("failure_count", 1)) != 0:
        return False, "failure_count_nonzero"
    if report.get("complete_map_audit_passed") is not True:
        return False, "complete_map_audit_not_passed"
    result = report.get("export_result")
    if not isinstance(result, dict) or result.get("export_task_success") is not True:
        return False, "export_result_invalid"
    try:
        if Path(result["output_file"]).resolve() != output_path.resolve():
            return False, "output_path_mismatch"
    except Exception:
        return False, "output_path_invalid"
    if not output_path.is_file() or output_path.stat().st_size <= 0:
        return False, "output_missing_or_empty"
    return True, "valid"

def fatal_markers(path: Path) -> List[str]:
    try:
        text = path.read_bytes().replace(b"\x00", b"").decode("utf-8", "replace").lower()
    except OSError:
        return []
    return [p for p in FATAL_PATTERNS if p in text]

def emit_new(path: Path, offset: int, mode: str):
    if not path.is_file():
        return offset, 0, 0
    with path.open("r", encoding="utf-8", errors="replace") as h:
        h.seek(offset)
        data = h.read()
        new_offset = h.tell()
    seen = emitted = 0
    for raw in data.splitlines():
        seen += 1
        line = raw.replace("\x00", "")
        if mode == "full" or any(p in line for p in KEY_PATTERNS):
            print(line, flush=True)
            emitted += 1
    return new_offset, seen, emitted

def terminate_group(process, term_grace):
    if process.poll() is not None:
        return False
    try:
        os.killpg(process.pid, signal.SIGTERM)
        print("SUPERVISOR_TERM_SENT=1", flush=True)
    except ProcessLookupError:
        return True
    deadline = time.monotonic() + term_grace
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return True
        time.sleep(0.5)
    try:
        os.killpg(process.pid, signal.SIGKILL)
        print("SUPERVISOR_KILL_SENT=1", flush=True)
    except ProcessLookupError:
        pass
    return True

def main():
    a = args_ns()
    for p in (a.report.parent, a.output.parent, a.process_log.parent):
        p.mkdir(parents=True, exist_ok=True)
    for p in (a.report, a.output, a.process_log):
        try: p.unlink()
        except FileNotFoundError: pass

    started_wall = time.time()
    started_mono = time.monotonic()
    print("EXPORT_SUPERVISOR_VERSION=1.0.0", flush=True)
    print("SUPERVISOR_REPORT={}".format(a.report), flush=True)
    print("SUPERVISOR_OUTPUT={}".format(a.output), flush=True)
    print("SUPERVISOR_PROCESS_LOG={}".format(a.process_log), flush=True)
    print("SUPERVISOR_COMMAND={}".format(" ".join(a.command)), flush=True)

    writer = a.process_log.open("w", encoding="utf-8", errors="replace", buffering=1)
    try:
        process = subprocess.Popen(
            a.command, stdout=writer, stderr=subprocess.STDOUT,
            text=True, start_new_session=True, env=os.environ.copy()
        )
    finally:
        writer.close()

    offset = seen_total = emitted_total = 0
    report_complete = False
    valid_since = None
    valid_reason = ""
    forced = False
    hard_timeout = False

    while True:
        offset, seen, emitted = emit_new(a.process_log, offset, a.stream_mode)
        seen_total += seen
        emitted_total += emitted
        valid, reason = valid_report(read_report(a.report), a.report, a.output, started_wall)
        if valid and not report_complete:
            report_complete = True
            valid_since = time.monotonic()
            valid_reason = reason
            print("SUPERVISOR_REPORT_COMPLETE=1", flush=True)
            print("SUPERVISOR_REPORT_VALIDATION={}".format(reason), flush=True)
        raw = process.poll()
        if raw is not None:
            break
        now = time.monotonic()
        if report_complete and valid_since is not None and now - valid_since >= a.report_exit_grace_seconds:
            print("SUPERVISOR_POST_REPORT_EXIT_GRACE_EXPIRED=1", flush=True)
            forced = terminate_group(process, a.term_grace_seconds)
            break
        if now - started_mono >= a.hard_timeout_seconds:
            hard_timeout = True
            print("SUPERVISOR_HARD_TIMEOUT=1", flush=True)
            forced = terminate_group(process, a.term_grace_seconds)
            break
        time.sleep(1)

    try:
        raw_exit = process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        forced = terminate_group(process, a.term_grace_seconds)
        raw_exit = process.wait(timeout=5)
    offset, seen, emitted = emit_new(a.process_log, offset, a.stream_mode)
    seen_total += seen
    emitted_total += emitted

    if not report_complete:
        deadline = time.monotonic() + a.report_settle_seconds
        while time.monotonic() < deadline:
            valid, reason = valid_report(read_report(a.report), a.report, a.output, started_wall)
            if valid:
                report_complete = True
                valid_reason = reason
                print("SUPERVISOR_REPORT_COMPLETE=1", flush=True)
                break
            time.sleep(0.5)

    fatals = fatal_markers(a.process_log)
    print("SUPERVISOR_RAW_EDITOR_EXIT={}".format(raw_exit), flush=True)
    print("SUPERVISOR_FORCED_SHUTDOWN={}".format(int(forced)), flush=True)
    print("SUPERVISOR_FATAL_MARKER_COUNT={}".format(len(fatals)), flush=True)
    print("SUPERVISOR_STREAM_LINES_SEEN={}".format(seen_total), flush=True)
    print("SUPERVISOR_STREAM_LINES_EMITTED={}".format(emitted_total), flush=True)

    if hard_timeout:
        print("SUPERVISOR_SUCCESS=0", flush=True); return 124
    if fatals:
        print("SUPERVISOR_FATAL_MARKERS={}".format(",".join(fatals)), flush=True)
        print("SUPERVISOR_SUCCESS=0", flush=True); return 90
    if not report_complete:
        print("SUPERVISOR_SUCCESS=0", flush=True); return 91
    # UE4Editor-Cmd can finish the Python export, write a valid report, then
    # linger during editor shutdown. After the post-report grace expires we
    # terminate the process group ourselves. Some UE builds return 1 for
    # this controlled shutdown. A valid report and a clear fatal-marker scan
    # allow Blender post-processing and deterministic validation to continue.
    controlled = forced and report_complete
    if raw_exit != 0 and not controlled:
        print("SUPERVISOR_SUCCESS=0", flush=True); return 92
    print("SUPERVISOR_FINISH_REASON={}".format(
        "valid_report_then_controlled_process_group_shutdown" if controlled
        else "valid_report_then_clean_editor_exit"
    ), flush=True)
    print("SUPERVISOR_REPORT_VALIDATION={}".format(valid_reason or "valid"), flush=True)
    print("SUPERVISOR_SUCCESS=1", flush=True)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
