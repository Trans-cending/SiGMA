#!/usr/bin/env python3
"""Supervise one non-rendering UE4Editor pass using its final JSON report.

UE4Editor 4.26 can finish an ExecutePythonScript pass, save the level, write a
valid report and print ``LogExit: Exiting.``, yet remain alive in shutdown for
an unbounded amount of time with streamed levels and NullRHI. The SiGMA CLI
uses this migrated deterministic supervisor to validate saved reports and
terminate only its own completed editor process group after a grace period.

Success policy:
  * the report must be fresh, JSON, ``success == true`` and have ``finished_at``;
  * when present, ``target_level`` and ``loaded_world_package`` must match the
    requested level;
  * the process log must be clear of fatal/crash markers;
  * after a valid report, the editor gets a grace period to exit normally;
    remaining shutdown-only processes are terminated as their work is already
    durably reported after save_level();
  * if ``LogExit: Exiting.`` appears while the valid report is still pending,
    fail immediately.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

FATAL_PATTERNS = (
    re.compile(r"segmentation fault", re.IGNORECASE),
    re.compile(r"\bsignal\s+11\b", re.IGNORECASE),
    re.compile(r"\bcaught\s+signal\s+11\b", re.IGNORECASE),
    re.compile(r"assertion failed", re.IGNORECASE),
    re.compile(r"\bfatal error\b", re.IGNORECASE),
    re.compile(r"\bunhandled exception\b", re.IGNORECASE),
    re.compile(r"\bcore dumped\b", re.IGNORECASE),
    re.compile(r"VK_ERROR_DEVICE_LOST", re.IGNORECASE),
    re.compile(r"VendorId\s*!=\s*EGpuVendorId::Unknown", re.IGNORECASE),
)
EXIT_PATTERNS = (
    re.compile(r"LogExit:\s*Exiting\.", re.IGNORECASE),
    re.compile(r"LogExit:\s*Preparing to exit", re.IGNORECASE),
)


def parse_duration(value: str) -> int:
    text = str(value).strip().lower()
    match = re.fullmatch(r"([1-9][0-9]*)([smhd]?)", text)
    if match is None:
        raise argparse.ArgumentTypeError("duration must match <integer>[s|m|h|d]")
    amount = int(match.group(1))
    unit = match.group(2) or "s"
    return amount * {"s": 1, "m": 60, "h": 3600, "d": 86400}[unit]


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def read_report(path: Path) -> Tuple[Optional[Dict[str, Any]], str]:
    if not path.is_file():
        return None, "report_missing"
    if path.stat().st_size <= 0:
        return None, "report_empty"
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return None, "report_unreadable:{}".format(exc)
    if not isinstance(data, dict):
        return None, "report_not_object"
    return data, "report_loaded"


def normalize_package(value: Any) -> str:
    return str(value or "").split(".", 1)[0].lower()


def validate_report(path: Path, *, started_wall: float, expected_level: str) -> Tuple[bool, str, Optional[Dict[str, Any]]]:
    data, reason = read_report(path)
    if data is None:
        return False, reason, None
    try:
        mtime = path.stat().st_mtime
    except OSError as exc:
        return False, "report_stat_failed:{}".format(exc), data
    if mtime < started_wall - 5.0:
        return False, "report_is_stale", data
    failed = []
    if data.get("success") is not True:
        failed.append("success")
    try:
        if float(data.get("finished_at", 0.0) or 0.0) <= 0.0:
            failed.append("finished_at")
    except (TypeError, ValueError):
        failed.append("finished_at")

    expected = normalize_package(expected_level)
    target = normalize_package(data.get("target_level"))
    loaded = normalize_package(data.get("loaded_world_package"))
    if target and target != expected:
        failed.append("target_level")
    if loaded and loaded != expected:
        failed.append("loaded_world_package")

    # The pass scripts report save_result when they explicitly save a level.
    save_result = data.get("save_result")
    if isinstance(save_result, dict) and "save_ok" in save_result:
        if save_result.get("save_ok") is not True:
            failed.append("save_result")

    if failed:
        return False, "report_checks_failed:" + ",".join(failed), data
    return True, "report_complete", data


def scan_markers(path: Path) -> Tuple[int, list[str], bool]:
    if not path.is_file():
        return 0, [], False
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return 0, [], False
    fatal_samples: list[str] = []
    fatal_count = 0
    exit_seen = False
    for line in text.splitlines():
        if any(pattern.search(line) for pattern in FATAL_PATTERNS):
            fatal_count += 1
            if len(fatal_samples) < 20:
                fatal_samples.append(line[:1000])
        if any(pattern.search(line) for pattern in EXIT_PATTERNS):
            exit_seen = True
    return fatal_count, fatal_samples, exit_seen


def terminate_group(proc: subprocess.Popen, grace: int) -> Tuple[bool, bool]:
    if proc.poll() is not None:
        return False, False
    term_sent = False
    kill_sent = False
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        term_sent = True
    except ProcessLookupError:
        return False, False
    deadline = time.monotonic() + grace
    while proc.poll() is None and time.monotonic() < deadline:
        time.sleep(0.2)
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
            kill_sent = True
        except ProcessLookupError:
            pass
    return term_sent, kill_sent


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--editor", required=True)
    p.add_argument("--uproject", required=True)
    p.add_argument("--level-package", required=True)
    p.add_argument("--python-script", required=True)
    p.add_argument("--report", required=True)
    p.add_argument("--ue-log", required=True)
    p.add_argument("--process-log", required=True)
    p.add_argument("--label", required=True)
    p.add_argument("--timeout", type=parse_duration, default=parse_duration("30m"))
    p.add_argument("--report-exit-grace-seconds", type=positive_int, default=45)
    p.add_argument("--exit-marker-report-grace-seconds", type=positive_int, default=60)
    p.add_argument("--term-grace-seconds", type=positive_int, default=15)
    p.add_argument("--report-settle-seconds", type=positive_int, default=5)
    p.add_argument("--poll-seconds", type=float, default=1.0)
    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.poll_seconds <= 0:
        raise SystemExit("--poll-seconds must be positive")

    report = Path(args.report)
    ue_log = Path(args.ue_log)
    process_log = Path(args.process_log)
    for path in (report.parent, ue_log.parent, process_log.parent):
        path.mkdir(parents=True, exist_ok=True)
    try:
        report.unlink()
    except FileNotFoundError:
        pass
    for stale in report.parent.glob(report.name + ".tmp.*"):
        try:
            stale.unlink()
        except OSError:
            pass

    command = [
        args.editor,
        args.uproject,
        args.level_package,
        "-nullrhi",
        "-windowed",
        "-ResX=1920",
        "-ResY=1080",
        "-enablehighdpi",
        "-unattended",
        "-nop4",
        "-NoSourceControl",
        "-nosplash",
        "-NoSound",
        "-NoLoadingScreen",
        "-stdout",
        "-FullStdOutLogOutput",
        "-ExecutePythonScript={}".format(args.python_script),
        "-log",
        "-abslog={}".format(args.ue_log),
    ]
    print("POSTPROCESS_SUPERVISOR_LABEL={}".format(args.label), flush=True)
    print("POSTPROCESS_SUPERVISOR_COMMAND={}".format(" ".join(command)), flush=True)
    print("POSTPROCESS_SUPERVISOR_TIMEOUT_SECONDS={}".format(args.timeout), flush=True)

    started_wall = time.time()
    started_mono = time.monotonic()
    hard_deadline = started_mono + args.timeout
    report_complete = False
    report_complete_at: Optional[float] = None
    report_reason = "report_not_checked"
    exit_marker_seen_at: Optional[float] = None
    forced_shutdown = False
    term_sent = False
    kill_sent = False
    timed_out = False
    exit_without_report = False
    process_exited_at: Optional[float] = None

    with process_log.open("w", encoding="utf-8", errors="replace") as writer:
        proc = subprocess.Popen(
            command,
            stdout=writer,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
            start_new_session=True,
        )
    print("POSTPROCESS_EDITOR_PID={}".format(proc.pid), flush=True)

    while True:
        now = time.monotonic()
        valid, reason, _data = validate_report(
            report,
            started_wall=started_wall,
            expected_level=args.level_package,
        )
        report_reason = reason
        if valid and not report_complete:
            report_complete = True
            report_complete_at = now
            print("POSTPROCESS_REPORT_COMPLETE=1", flush=True)
            print(
                "POSTPROCESS_REPORT_COMPLETE_AFTER_SECONDS={:.3f}".format(now - started_mono),
                flush=True,
            )

        fatal_count, _fatal_samples, exit_seen = scan_markers(process_log)
        if exit_seen and exit_marker_seen_at is None:
            exit_marker_seen_at = now
            print("POSTPROCESS_LOG_EXIT_MARKER_SEEN=1", flush=True)

        return_code = proc.poll()
        if return_code is not None:
            if process_exited_at is None:
                process_exited_at = now
            if report_complete:
                break
            if now - process_exited_at >= args.report_settle_seconds:
                break

        if fatal_count > 0 and not report_complete:
            print("POSTPROCESS_FATAL_MARKER_BEFORE_REPORT=1", flush=True)
            term_sent, kill_sent = terminate_group(proc, args.term_grace_seconds)
            break

        if (
            report_complete
            and report_complete_at is not None
            and proc.poll() is None
            and now - report_complete_at >= args.report_exit_grace_seconds
        ):
            forced_shutdown = True
            print("POSTPROCESS_POST_REPORT_EXIT_GRACE_EXPIRED=1", flush=True)
            term_sent, kill_sent = terminate_group(proc, args.term_grace_seconds)
            break

        if (
            not report_complete
            and exit_marker_seen_at is not None
            and proc.poll() is None
            and now - exit_marker_seen_at >= args.exit_marker_report_grace_seconds
        ):
            exit_without_report = True
            print("POSTPROCESS_EXIT_MARKER_WITHOUT_REPORT=1", flush=True)
            term_sent, kill_sent = terminate_group(proc, args.term_grace_seconds)
            break

        if not report_complete and now >= hard_deadline:
            timed_out = True
            print("POSTPROCESS_HARD_TIMEOUT=1", flush=True)
            term_sent, kill_sent = terminate_group(proc, args.term_grace_seconds)
            break

        time.sleep(args.poll_seconds)

    try:
        raw_exit = proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        term2, kill2 = terminate_group(proc, args.term_grace_seconds)
        term_sent = term_sent or term2
        kill_sent = kill_sent or kill2
        raw_exit = proc.wait(timeout=5)

    final_valid, final_reason, _data = validate_report(
        report,
        started_wall=started_wall,
        expected_level=args.level_package,
    )
    if final_valid:
        report_complete = True
        report_reason = final_reason

    fatal_count, fatal_samples, _ = scan_markers(process_log)
    normalized_success = (
        report_complete
        and fatal_count == 0
        and (raw_exit == 0 or forced_shutdown)
        and not timed_out
        and not exit_without_report
    )

    if timed_out:
        finish_reason = "hard_timeout_without_valid_report"
    elif exit_without_report:
        finish_reason = "editor_exit_marker_without_valid_report"
    elif not report_complete:
        finish_reason = "final_report_missing_or_invalid"
    elif fatal_count > 0:
        finish_reason = "fatal_marker_detected"
    elif raw_exit == 0:
        finish_reason = "editor_exit_zero_with_valid_report"
    elif forced_shutdown:
        finish_reason = "valid_report_then_controlled_shutdown"
    else:
        finish_reason = "unexpected_nonzero_editor_exit"

    print("POSTPROCESS_RAW_EDITOR_EXIT={}".format(raw_exit), flush=True)
    print("POSTPROCESS_REPORT_VALID={}".format(int(report_complete)), flush=True)
    print("POSTPROCESS_REPORT_REASON={}".format(report_reason), flush=True)
    print("POSTPROCESS_FORCED_SHUTDOWN={}".format(int(forced_shutdown)), flush=True)
    print("POSTPROCESS_TERM_SENT={}".format(int(term_sent)), flush=True)
    print("POSTPROCESS_KILL_SENT={}".format(int(kill_sent)), flush=True)
    print("POSTPROCESS_FATAL_MARKER_COUNT={}".format(fatal_count), flush=True)
    for sample in fatal_samples:
        print("POSTPROCESS_FATAL_SAMPLE={}".format(sample), flush=True)
    print("POSTPROCESS_FINISH_REASON={}".format(finish_reason), flush=True)
    print("POSTPROCESS_SUCCESS={}".format(int(normalized_success)), flush=True)

    if normalized_success:
        return 0
    if timed_out:
        return 51
    if exit_without_report:
        return 52
    if not report_complete:
        return 53
    if fatal_count > 0:
        return 54
    return 55


if __name__ == "__main__":
    raise SystemExit(main())
