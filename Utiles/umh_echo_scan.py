"""Scan all 84 channels with CAL_PROBE and save [channel, gate, mic] complex profiles.

    python Utiles/umh_echo_scan.py --level 16 --repeats 8 --save Utiles/cal_capture/scan_l16_a.npz
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("\\", 1)[0] if "\\" in __file__ else ".")
from umh_cal_probe import UmhProbe, CHANNELS  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--level", type=int, default=16)
    ap.add_argument("--repeats", type=int, default=8)
    ap.add_argument("--gates", type=int, default=64)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--step", type=int, default=1)
    ap.add_argument("--width", type=int, default=1)
    ap.add_argument("--settle", type=int, default=20000)
    ap.add_argument("--channels", default="all")
    ap.add_argument("--save", required=True)
    a = ap.parse_args(argv)
    chans = range(CHANNELS) if a.channels == "all" else [int(c) for c in a.channels.split(",")]
    out = np.zeros((CHANNELS, a.gates, 4), complex)
    sat = np.zeros(CHANNELS, int)
    t0 = time.time()
    with UmhProbe(a.port) as p:
        for ch in chans:
            z, s = p.single(ch, a.level, gate_count=a.gates, gate_start=a.start,
                            gate_step=a.step, gate_width=a.width, settle_us=a.settle,
                            repeats=a.repeats)
            out[ch] = z / a.width
            sat[ch] = s
    np.savez(a.save, z=out, sat=sat, level=a.level, start=a.start, step=a.step,
             width=a.width, repeats=a.repeats, t=time.time())
    print("scan done %.1f s, saturated blocks %d" % (time.time() - t0, sat.sum()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
