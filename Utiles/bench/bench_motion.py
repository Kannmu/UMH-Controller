"""Motion-engine performance benchmark for UMH v7.

Measures steady-state throughput with the device-side counters.  A settle
window after MOTION_START avoids the first-frame deadline warm-up (fixed in
firmware after the first benchmark run), then the wall-clock frame rate and
the device counters are compared over the measurement window.

Usage:  python bench_motion.py [--port COM4] [--quick] [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time

sys.path.insert(0, __file__.rsplit("\\", 1)[0])

from umh_link import (  # noqa: E402
    MOTION_FLAG_DIRECT, MOTION_FLAG_LOOP, MOTION_FLAG_RGB, UmhLink, make_path,
)

FLAGS_PATH = MOTION_FLAG_RGB | MOTION_FLAG_LOOP


def run_case(link: UmhLink, name: str, points, mode: int, flags: int,
             rate_hz: int, loop_ms: int, trap_mode: int = 0,
             trap_radius_um: int = 2000, duration_s: float = 2.0,
             live_target_hz: float = 50.0, level: int = 200,
             settle_s: float = 0.4) -> dict:
    try:
        link.motion_stop()
    except Exception:
        pass
    link.clear_plan()
    link.motion_config(mode, flags, rate_hz, loop_ms, 3000, 65535, level,
                       trap_mode=trap_mode, trap_radius_um=trap_radius_um,
                       palette_count=16, palette=[(255, 80, 20)] * 16)
    if points:
        link.motion_upload(points)
    link.motion_start()
    start = time.monotonic()
    try:
        # settle: let the first-frame deadline warm-up pass and let the
        # device-side fps window settle.
        time.sleep(settle_s)
        before = link.get_motion_status()
        t0 = time.monotonic()
        live_period = 1.0 / max(1.0, live_target_hz)
        next_send = t0
        index = 0
        while time.monotonic() - t0 < duration_s:
            if mode == 1:
                now = time.monotonic()
                if now >= next_send:
                    next_send += live_period
                    angle = index * 0.05
                    link.motion_target(int(12_000 * math.cos(angle)),
                                       int(12_000 * math.sin(angle)),
                                       70_000, level, index % 16)
                    index += 1
            time.sleep(0.001)
        t1 = time.monotonic()
        after = link.get_motion_status()
        fps_samples = []
        for _ in range(4):
            time.sleep(0.05)
            fps_samples.append(link.get_motion_status().fps_x100 / 100.0)
    finally:
        try:
            link.motion_stop()
        except Exception as exc:  # noqa: BLE001
            print(f"  stop failed: {exc}")
        time.sleep(0.05)

    wall = max(1e-6, t1 - t0)
    return {
        "case": name,
        "mode": mode,
        "points": len(points) if points else 0,
        "rate_hz": rate_hz,
        "loop_ms": loop_ms,
        "trap_mode": trap_mode,
        "real_fps": round((after.frames - before.frames) / wall, 1),
        "device_fps": round(statistics.mean(fps_samples), 1) if fps_samples else 0.0,
        "device_fps_min": min(fps_samples) if fps_samples else 0.0,
        "missed": after.missed_deadlines - before.missed_deadlines,
        "errors": after.frame_errors - before.frame_errors,
        "service_max_us": after.service_max_us,
        "service_avg_us": after.service_avg_us,
        "render_max_us": after.render_max_us,
        "render_avg_us": after.render_avg_us,
        "submit_max_us": after.submit_max_us,
        "submit_avg_us": after.submit_avg_us,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--json", default="")
    args = parser.parse_args()

    link = UmhLink(args.port)
    results = []
    try:
        profile = link.get_profile()
        print(f"device {profile.model} fw {profile.firmware} caps=0x{profile.capability_flags:x}")
        rate_list = [200, 500, 800, 1000, 1500, 2000] if not args.quick else [200, 800, 1500]
        point_list = [32, 64, 128, 160] if not args.quick else [64, 160]
        trap_list = [0, 1, 2, 3, 4, 5] if not args.quick else [0, 3]

        for count in point_list:
            path = make_path("circle", points=count)
            for rate in rate_list:
                results.append(run_case(link, f"path-circle-{count}p-{rate}Hz", path, 0,
                                        FLAGS_PATH, rate, 160, duration_s=1.5))

        path = make_path("sphere", points=128)
        for trap in trap_list:
            for rate in ([500, 1000, 2000] if not args.quick else [800]):
                results.append(run_case(link, f"trap{trap}-128p-{rate}Hz", path, 0,
                                        FLAGS_PATH, rate, 160, trap_mode=trap,
                                        duration_s=1.5))

        for rate in ([500, 1000, 2000] if not args.quick else [1000]):
            results.append(run_case(link, f"live-{rate}Hz", None, 1,
                                    MOTION_FLAG_RGB, rate, 2000, duration_s=1.5))

        header = (f"{'case':26s} {'real':>7s} {'missed':>7s} {'err':>4s} "
                  f"{'svc_max':>7s} {'render':>7s} {'submit':>7s}")
        print(header)
        for row in results:
            print(f"{row['case']:26s} {row['real_fps']:7.1f} {row['missed']:7d} {row['errors']:4d} "
                  f"{row['service_max_us']:7d} {row['render_max_us']:7d} {row['submit_max_us']:7d}")

        if args.json:
            with open(args.json, "w", encoding="utf-8") as handle:
                json.dump({"profile": profile.__dict__, "results": results}, handle, indent=2)
            print(f"wrote {args.json}")
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
