"""Trigger the on-device axial table-echo calibration and print the result.

    python Utiles/umh_cal_run.py [--port COM4] [--runs 3] [--save prefix]
    python Utiles/umh_cal_run.py --result-only

The array must face the table (flat surface 5..25 cm away, a ~10 cm patch
below the centre free).  A run takes ~45 s.  The device writes EEPROM only
when its own focus test accepts the candidate (committed = 1).
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

import numpy as np

from umh_cal_probe import UmhProbe

MSG_CAL_RESULT, MSG_CAL_START = 0x81, 0x82
MSG_ACK, MSG_NACK = 0x70, 0x71
N = 84
LEGACY = "<84sBBBBBBBHH10f"
EXT = "<3fHBBBBBB84s84s"
STATES = ["IDLE", "WAIT", "MEASURE", "SOLVE", "VERIFY", "OK", "FAIL"]
QFLAGS = ["COUPLING", "MICS", "FIT", "CONSISTENCY", "LEVEL", "GEOM", "VERIFY", "MEASURE"]


def parse(data: bytes) -> dict:
    n0 = struct.calcsize(LEGACY)
    v = struct.unpack_from(LEGACY, data, 0)
    r = dict(phase=np.frombuffer(v[0], np.uint8), progress=v[1], good_mics=v[2], fault=v[3],
             quality_flags=v[4], state=v[5], gate_count=v[6], gate_width=v[7], gate_start=v[8],
             block_count=v[9], rms_before_deg=v[10], rms_after_deg=v[11], mic_consistency_deg=v[12],
             saturated=v[13], table_mm=v[14] * 1000.0, tilt_x_deg=v[15], tilt_y_deg=v[16],
             echo_mag=v[17], amp_spread_db=v[18], verify_gain_db=v[19])
    if len(data) >= n0 + struct.calcsize(EXT):
        e = struct.unpack_from(EXT, data, n0)
        r.update(correction_rms_deg=e[0], amp_p10=e[1], amp_p90=e[2], pairs=e[3], covered=e[4],
                 dead=e[5], verify_wins=e[6], verify_sets=e[7], committed=e[8], method=e[9],
                 amplitude=np.frombuffer(e[10], np.uint8), coverage=np.frombuffer(e[11], np.uint8))
    return r


def read_result(p: UmhProbe) -> dict:
    tid = p._send(MSG_CAL_RESULT, b"")
    t, _, _, data = p._recv(tid, 3.0)
    if t != MSG_CAL_RESULT:
        raise RuntimeError("CAL_RESULT failed type 0x%02x" % t)
    return parse(data)


def run_once(p: UmhProbe, timeout=180.0, verbose=True) -> dict:
    tid = p._send(MSG_CAL_START, b"")
    t, _, _, data = p._recv(tid, 3.0)
    if t != MSG_ACK:
        raise RuntimeError("CAL_START rejected: type 0x%02x status %s" % (t, data[:1].hex()))
    t0 = time.time()
    last = -1
    while time.time() - t0 < timeout:
        time.sleep(1.0)
        r = read_result(p)
        if verbose and r["progress"] != last:
            print("  %5.1fs %-7s %3d%%" % (time.time() - t0, STATES[r["state"]] if r["state"] < 7 else r["state"],
                                          r["progress"]), flush=True)
            last = r["progress"]
        if r["progress"] >= 100 and r["state"] in (5, 6):
            r["elapsed_s"] = time.time() - t0
            return r
    raise TimeoutError("calibration did not finish")


def summary(r: dict) -> str:
    q = [QFLAGS[b] for b in range(8) if r["quality_flags"] >> b & 1]
    s = ("%s fault=%d flags=%s committed=%s  table %.1f mm  echo %.1f  pairs %s covered %s dead %s mics %d\n"
         "  static rms %.1f deg -> applied correction rms %.1f deg, pair sd %.1f deg, tilt %.2f/%.2f deg\n"
         "  focus test %+.2f dB (%s/%s sets better)  amp p10/p90 %.2f/%.2f  saturated blocks %d"
         % (STATES[r["state"]] if r["state"] < 7 else r["state"], r["fault"], q or "-", r.get("committed"),
            r["table_mm"], r["echo_mag"], r.get("pairs"), r.get("covered"), r.get("dead"), r["good_mics"],
            r["rms_before_deg"], r.get("correction_rms_deg", 0), r["rms_after_deg"], r["tilt_x_deg"],
            r["tilt_y_deg"], r["verify_gain_db"], r.get("verify_wins"), r.get("verify_sets"),
            r.get("amp_p10", 0), r.get("amp_p90", 0), r["saturated"]))
    if "coverage" in r:
        dead = np.where(r["coverage"] == 0xFF)[0]
        ph = r["phase"].astype(int)
        ph = np.where(ph > 127, ph - 256, ph) * 360 / 256
        big = [(int(i), round(float(ph[i]))) for i in np.where(np.abs(ph) >= 30)[0]]
        s += "\n  dead channels %s  |correction|>=30 deg: %s" % (list(map(int, dead)), big)
    return s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--save", default=None)
    ap.add_argument("--result-only", action="store_true")
    a = ap.parse_args(argv)
    with UmhProbe(a.port) as p:
        if a.result_only:
            print(summary(read_result(p)))
            return 0
        res = []
        for k in range(a.runs):
            print("run %d" % (k + 1))
            r = run_once(p)
            print(summary(r))
            res.append(r)
            if a.save:
                np.savez("%s_%d.npz" % (a.save, k), **{kk: np.asarray(v) for kk, v in r.items()})
        if len(res) > 1:
            q = np.array([np.where(r["phase"] > 127, r["phase"].astype(int) - 256, r["phase"]) for r in res]) * 360 / 256
            for k in range(1, len(res)):
                d = (q[k] - q[0] + 180) % 360 - 180
                print("repeat run %d vs 1: phase byte diff rms %.1f deg, max %.0f deg" % (k + 1, np.sqrt((d ** 2).mean()), np.abs(d).max()))
    return 0 if all(r["state"] == 5 for r in res) else 1


if __name__ == "__main__":
    sys.exit(main())
