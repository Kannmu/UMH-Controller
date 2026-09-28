"""Stream a test envelope and trace the audio engine over SWD (core keeps running).

Needs openocd running with `-c "tcl_port 6666"`.  Samples ring head/tail,
rendered_samples, underrun_grace and TIM2 (1 MHz system time) and reports
stalls: intervals where rendered_samples advanced slower than 20 kHz.

  python Utiles/umh_audio_trace.py --case tone1k --secs 3
"""
import argparse
import struct
import sys
import threading
import time
import os

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from ocd_tcl import Ocd  # noqa: E402
from umh_audio_bench import V7Audio, make_case, stream, RATE, PREBUF  # noqa: E402

ENG = 0x20000A10
HEADTAIL = ENG + 2144
BLOCK = ENG + 3000          # next_due .. max_service_cycles
TIM2_CNT = 0x40000024


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="tone1k")
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--port", default="COM4")
    a = ap.parse_args()
    o = Ocd()
    dev = V7Audio(a.port)
    try:
        dev.request(0x22)
        dev.request(0x23)
        cfg = struct.pack("<iiiBBHHH", 0, 0, 1_000_000, 0, 255, RATE, PREBUF, 0)
        assert dev.request(0x90, cfg)[1] == 0x70
        dev.request(0x91)
        env = make_case(a.case, a.secs + 1.0)
        stats = []
        th = threading.Thread(target=stream, args=(dev, env, stats))
        th.start()
        time.sleep(0.5)
        rows = []
        t_end = time.time() + a.secs
        while time.time() < t_end:
            t = o.read32(TIM2_CNT)[0]
            ht = o.read32(HEADTAIL)[0]
            blk = o.read32(BLOCK, 17)
            rows.append((t, ht & 0xFFFF, ht >> 16, blk))
        th.join()
        dev.request(0x93)
    finally:
        dev.close()
    t = np.array([r[0] for r in rows], dtype=np.int64)
    head = np.array([r[1] for r in rows]); tail = np.array([r[2] for r in rows])
    fill = (head - tail) & 0xFFFF
    b = np.array([r[3] for r in rows], dtype=np.uint64)
    next_due = ((b[:, 1] << 32) | b[:, 0]) >> 16
    grace = b[:, 8] & 0xFFFF
    ppm = b[:, 9].astype(np.uint32).astype(np.int32)
    und = b[:, 10]
    rendered = b[:, 13]
    svc = b[:, 14]
    dt = np.diff(t) / 1e6
    dr = np.diff(rendered.astype(np.int64))
    rate = dr / dt
    lag = ((t - next_due.astype(np.int64) + (1 << 31)) % (1 << 32)) - (1 << 31)  # >0: engine behind
    print("samples %d  mean interval %.2f ms" % (len(t), dt.mean() * 1e3))
    print("fill min/median/max %d/%d/%d" % (fill.min(), np.median(fill), fill.max()))
    print("grace nonzero in %.1f%% of samples, underruns %d->%d" % (np.mean(grace > 0) * 100, und[0], und[-1]))
    print("ppm range %d..%d" % (ppm.min(), ppm.max()))
    print("lag (now - next_due) us: median %d  p99 %d  max %d  min %d" % (
        np.median(lag), np.percentile(lag, 99), lag.max(), lag.min()))
    print("render rate per interval: median %.0f  p1 %.0f  p99 %.0f" % (
        np.median(rate), np.percentile(rate, 1), np.percentile(rate, 99)))
    slow = np.where(rate < 15000)[0]
    print("intervals with rate<15 kHz: %d / %d" % (len(slow), len(rate)))
    for i in slow[:15]:
        print("   t=%.3f s dt=%.2f ms rendered+%d lag %d->%d us fill %d grace %d" % (
            (t[i] - t[0]) / 1e6, dt[i] * 1e3, dr[i], lag[i], lag[i + 1], fill[i], grace[i]))
    print("max service cycles %d (%.1f us @170MHz)" % (svc.max(), svc.max() / 170.0))


if __name__ == "__main__":
    main()
