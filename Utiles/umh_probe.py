#!/usr/bin/env python3
"""Live state probe for the UMH controller (ST-Link via the running OpenOCD).

Reads a snapshot through gdb + DWARF so static symbols always resolve to the
addresses of the running image.  One halt per invocation: never probe *while* a
modulated stream runs, because a halt longer than the FPGA link watchdog ends
the session.  Use it before/after a run for the accumulated CPU shares instead.

  python Utiles/umh_probe.py
  python Utiles/umh_probe.py --tasks
  python Utiles/umh_probe.py --reset-stats
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys

GDB = r"D:\SOFTWARE\GCC-ARM-NONE-EABI-10.3-2021.10\BIN\arm-none-eabi-gdb.exe"
ELF = r"build\UMH_Controller.elf"
HOST = ":3333"

MOD_FIELDS = [
    "state", "ring_head", "ring_tail", "prebuffer_samples", "envelope_ceiling",
    "base_step_q16", "frac_q16", "step_q16", "fill_error_q12",
    "clock_correction_ppm", "underrun_count", "overrun_count",
    "packet_loss_count", "rendered_samples", "max_service_cycles",
    "underrun_grace", "gain", "waiting_refill", "sequence_valid", "poll_tick",
]

MARK = "@@"


def gdb_batch(commands: list[str], timeout: float = 60.0) -> str:
    args = [GDB, "-batch", "-nx", ELF, "-ex", "set pagination off",
            "-ex", "set print elements 64", "-ex", f"target extended-remote {HOST}"]
    for cmd in commands:
        args += ["-ex", cmd]
    args += ["-ex", "detach"]
    proc = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    return proc.stdout + proc.stderr


def read_marked(exprs: list[str]) -> dict[str, str]:
    commands: list[str] = []
    for i, expr in enumerate(exprs):
        commands.append(f'echo {MARK}{i}{MARK}\\n')
        commands.append(f"p {expr}")
    text = gdb_batch(commands)
    out: dict[str, str] = {}
    for i, expr in enumerate(exprs):
        m = re.search(rf"{MARK}{i}{MARK}\n(.*?)(?=\n{MARK}|\Z)", text, re.S)
        if not m:
            raise RuntimeError(f"no marker {i} for {expr}")
        payload = m.group(1).strip()
        value = re.search(r"^\$\d+ = (.*)$", payload, re.M | re.S)
        out[expr] = (value.group(1) if value else payload).strip()
    return out


def as_int(raw: str) -> int:
    raw = raw.strip()
    if not raw:
        return 0
    token = raw.split()[0]
    try:
        return int(token, 0)
    except ValueError:
        return 0


def snapshot() -> dict:
    exprs = [f"(int)aperture_mod.{f}" if f == "state" else f"aperture_mod.{f}"
             for f in MOD_FIELDS]
    exprs += [
        "umh_render_profile", "umh_render_burst_yields", "fpga_link.status",
        "fpga_link.levels_valid", "fpga_link.full_frames", "fpga_link.phase_frames",
        "fpga_link.transaction_sequence",
        "'task_stats.c'::stats.busy_percent_x100",
        "'task_stats.c'::stats.avg_busy_x100",
        "'task_stats.c'::stats.peak_busy_x100",
        "'task_stats.c'::stats.window_ms",
        "'task_stats.c'::accum_windows",
    ]
    vals = read_marked(exprs)
    mod = {f: as_int(vals[exprs[i]]) for i, f in enumerate(MOD_FIELDS)}
    fpga_raw = vals["fpga_link.status"]
    fpga = {}
    for key in ("protocol_version", "mod_fill", "fifo_credit", "fifo_depth",
                "status_flags", "fpga_time", "accepted_sequence"):
        m = re.search(rf"{key} = (\d+)", fpga_raw)
        fpga[key] = int(m.group(1)) if m else None
    return {
        "mod": mod,
        "render_profile": [int(v) for v in re.findall(r"\d+", vals["umh_render_profile"])],
        "burst_yields": as_int(vals["umh_render_burst_yields"]),
        "fpga": fpga,
        "levels_valid": as_int(vals["fpga_link.levels_valid"]),
        "full_frames": as_int(vals["fpga_link.full_frames"]),
        "phase_frames": as_int(vals["fpga_link.phase_frames"]),
        "link_txn": as_int(vals["fpga_link.transaction_sequence"]),
        "busy_x100": as_int(vals["'task_stats.c'::stats.busy_percent_x100"]),
        "avg_busy_x100": as_int(vals["'task_stats.c'::stats.avg_busy_x100"]),
        "peak_busy_x100": as_int(vals["'task_stats.c'::stats.peak_busy_x100"]),
        "window_ms": as_int(vals["'task_stats.c'::stats.window_ms"]),
        "windows": as_int(vals["'task_stats.c'::accum_windows"]),
    }


def task_shares() -> list[dict]:
    commands = [f'echo {MARK}WIN{MARK}\\n']
    commands.append("p 'task_stats.c'::accum_windows")
    for i in range(10):
        commands.append(f'echo {MARK}NAME{i}{MARK}\\n')
        commands.append(f"p 'task_stats.c'::stats.entries[{i}].pcTaskName")
        commands.append(f'echo {MARK}SUM{i}{MARK}\\n')
        commands.append(f"p 'task_stats.c'::accum_sum_x100[{i}]")
    text = gdb_batch(commands)

    def grab(tag: str) -> str:
        m = re.search(rf"{MARK}{tag}{MARK}\n(.*?)(?=\n{MARK}|\Z)", text, re.S)
        if not m:
            return ""
        payload = m.group(1).strip()
        v = re.search(r"^\$\d+ = (.*)$", payload, re.M | re.S)
        return (v.group(1) if v else payload).strip()

    windows = as_int(grab("WIN")) or 1
    out = []
    for i in range(10):
        raw_name = grab(f"NAME{i}")
        m = re.search(r'"([^"]*)"', raw_name)
        if not m:
            continue
        out.append({"task": m.group(1), "avg_x100": as_int(grab(f"SUM{i}")) // windows})
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tasks", action="store_true")
    ap.add_argument("--reset-stats", action="store_true")
    args = ap.parse_args()
    try:
        if args.reset_stats:
            gdb_batch(["set var 'task_stats.c'::stats.reset_request = 1"])
            print("task stats reset requested")
            return 0
        if args.tasks:
            for row in task_shares():
                print(f"{row['task']:10s} {row['avg_x100'] / 100.0:6.2f} %")
            return 0
        print(json.dumps(snapshot(), ensure_ascii=False))
    except Exception as exc:  # noqa: BLE001
        print(f"probe error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
