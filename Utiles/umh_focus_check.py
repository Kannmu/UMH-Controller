"""Independent physical focus check of a phase map on the table echo.

Random N-channel sets (drawn from the whole array, not only the calibration
cone) are focused onto each microphone's mirror image with host-computed
production-renderer phases, and the echo magnitude is compared between maps.
Maps: zero, the current EEPROM map, and optionally a saved .npy/.bin map.

    python Utiles/umh_focus_check.py --sets 40 --channels 4 --level 8
"""
from __future__ import annotations

import argparse
import struct
import sys

import numpy as np

import umh_echo_cal_ref as R
from umh_cal_probe import UmhProbe
from umh_echo_geom import ELEM_XY, MIC_XY, SRC_Z

K = 2 * np.pi * 40000 / 343000.0


def eeprom_phase(p: UmhProbe) -> np.ndarray:
    tid = p._send(0x40, b"")
    _, _, _, rec = p._recv(tid, 3.0)
    return (np.array(struct.unpack_from("<84H", rec, 12)) >> 8).astype(np.uint8)


def focus_phase(cal: np.ndarray, mic: int, V: float) -> np.ndarray:
    dx = ELEM_XY[:, 0] - MIC_XY[mic, 0]
    dy = ELEM_XY[:, 1] - MIC_XY[mic, 1]
    r = np.sqrt(dx * dx + dy * dy + (2 * V + SRC_Z) ** 2)
    ph = cal.astype(float) - r * K / (2 * np.pi) * 256
    return np.mod(np.round(ph), 256).astype(np.uint8)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--sets", type=int, default=40)
    ap.add_argument("--channels", type=int, default=4)
    ap.add_argument("--level", type=int, default=8)
    ap.add_argument("--repeats", type=int, default=4)
    ap.add_argument("--table", type=float, default=93.2)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--exclude", type=int, nargs="*", default=[61])
    ap.add_argument("--force", type=int, nargs="*", default=[],
                    help="each set contains one of these channels (round robin)")
    a = ap.parse_args(argv)
    V = a.table
    start, t_on = R.b_plan(V)
    base, echo = R.b_windows(start, t_on, V)
    rng = np.random.default_rng(a.seed)
    pool = np.array([i for i in range(84) if i not in a.exclude])
    with UmhProbe(a.port) as p:
        maps = {"zero": np.zeros(84, np.uint8), "eeprom": eeprom_phase(p)}
        res = {k: [] for k in maps}
        for s in range(a.sets):
            mic = s % 4
            ch = rng.choice(pool, a.channels, replace=False)
            if a.force:
                f = a.force[s % len(a.force)]
                if f not in ch:
                    ch[0] = f
            lv = np.zeros(84, np.uint8)
            lv[ch] = a.level
            for name, cal in maps.items():
                z, _ = p.probe(focus_phase(cal, mic, V), lv, gate_count=16, gate_start=start, gate_step=2,
                               gate_width=2, repeats=a.repeats, diff=True)
                d = z[echo, mic].mean() - z[base, mic].mean()
                res[name].append(abs(d))
        z0 = np.array(res["zero"])
        for name in maps:
            v = np.array(res[name])
            db = 20 * np.log10(v / z0)
            print("%-7s mean |echo| %6.1f   vs zero %+5.2f dB (median %+5.2f)  better in %d/%d"
                  % (name, v.mean(), 20 * np.log10(v.mean() / z0.mean()), np.median(db),
                     int((v > z0).sum()), len(v)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
