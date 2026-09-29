"""Pop / click bench: impulsive errors in the sound a listener actually hears.

The PC microphone demodulates the 40 kHz carrier by its own nonlinearity, so
its audio band is not what reaches an ear.  At 40 kHz the mic is linear: the
emitted envelope E(t) is recovered from the carrier (analytic band fc+-8 kHz,
48 kHz complex baseband) and the audible sound is modelled from it (Berktay,
p ~ d2/dt2 E^2).  Per case, over 200..7000 Hz:

  e  fractional envelope error E/mean(E) - 1
  b  audible proxy (E/mean E)^2, w^2-weighted (unity at 1 kHz)
  p  carrier phase, rad (diagnostic)
  a  raw mic audio band (reported only: contains the mic's own demodulation)

Stationary lines (programme tone and harmonics, alias spurs, hum) are notched
out, leaving the broadband floor plus clicks.  A click is |r| > k*MAD-sigma;
exceedances closer than 5 ms are one event.  References: off (TX off, gives
the additive mic floor) and dc64 (constant level: the MCU submits nothing).

Cases: off, dcN, altA_B (A,B,A,B.. per 50 us sample), toneF / hitoneF
(64+-40 / 88+-40 sine, F in Hz or e.g. 1k).  Lower the mic first:

  powershell -File Utiles/mic_volume.ps1 -Level 0.02
  python Utiles/umh_pop_bench.py --cases off,dc64,alt63_64,tone1k --secs 10
  python Utiles/umh_pop_bench.py --cases tone1k --secs 170 --save long.npz
  python Utiles/umh_pop_bench.py --load long.npz
  powershell -File Utiles/mic_volume.ps1 -Level 1.0
"""
import argparse
import json
import os
import struct
import sys
import tempfile
import threading
import time

import numpy as np
from scipy import fft as sfft
from scipy.ndimage import median_filter
from scipy.signal.windows import tukey

sys.path.insert(0, os.path.dirname(__file__))
from umh_audio_bench import FS, PREBUF, RATE, V7Audio, record_start, stream  # noqa: E402

DEC = 2
FSB = FS // DEC          # baseband rate; products of |z| stay below Nyquist
LO, HI = 200.0, 7000.0   # analysis band, Hz
HALF_BW = 7500.0         # carrier passband +-, tapered to 0 at +-8 kHz
EDGE = 1.0               # s dropped at both ends (taper, notch ringing)
TAPER = 0.3              # s Tukey taper
DEAD = 0.005             # s, exceedances closer than this are one event


def tone_freq(case):
    for pre in ("hitone", "tone"):
        if case.startswith(pre):
            s = case[len(pre):]
            return float(s[:-1]) * 1000 if s.endswith("k") else float(s)
    return None


def make_env(case, secs):
    n = int(secs * RATE)
    if case.startswith("dc"):
        v = np.full(n, float(case[2:]))
    elif case.startswith("alt"):
        a, b = (float(t) for t in case[3:].split("_"))
        v = np.where(np.arange(n) % 2 == 0, a, b)
    else:
        f = tone_freq(case)
        if f is None:
            raise ValueError("unknown case " + case)
        v = (88.0 if case.startswith("hi") else 64.0) + 40.0 * np.sin(2 * np.pi * f * np.arange(n) / RATE)
    return np.clip(np.round(v), 0, 128).astype(np.uint8)


def rc_gain(f, lo, hi, t):
    """1 inside [lo, hi], raised-cosine edges of width t outside it, 0 beyond."""
    g = np.ones(f.shape)
    d = np.maximum(lo - f, f - hi)
    m = d > 0
    g[m] = np.where(d[m] < t, 0.5 + 0.5 * np.cos(np.pi * d[m] / t), 0.0)
    return g


def find_lines(p, df, lo, hi, blk_hz=2.0, win_hz=100.0, thr=10.0):
    """Stationary lines in power spectrum p: blocks > thr x local median."""
    nb = max(1, int(round(blk_hz / df)))
    nblk = len(p) // nb
    pb = p[:nblk * nb].reshape(nblk, nb).sum(1) + 1e-30
    fl = median_filter(pb, size=2 * int(win_hz / (2 * nb * df)) + 1, mode="nearest")
    f0 = (np.arange(nblk) + 0.5) * nb * df
    hot = np.flatnonzero((pb > thr * fl) & (f0 > lo - 50) & (f0 < hi + 50))
    lines = []
    for g in np.split(hot, np.flatnonzero(np.diff(hot) > 1) + 1) if hot.size else []:
        lines.append((g[0] * nb * df, (g[-1] + 1) * nb * df, float(10 * np.log10((pb[g] / fl[g]).max()))))
    return lines


def clean(x, fs, lines=None, weight=False):
    """Band-limit to LO..HI, notch stationary lines, optionally w^2-weight.

    Returns (residual with EDGE trimmed, lines, windowed spectrum, df, ms scale)."""
    n = len(x)
    w = tukey(n, min(1.0, 2 * TAPER * fs / n))
    nf = sfft.next_fast_len(n, real=True)
    X = sfft.rfft((x - x.mean()) * w, nf)
    df = fs / nf
    if lines is None:
        lines = find_lines(np.abs(X) ** 2, df, LO, HI)
    f = np.arange(len(X)) * df
    g = rc_gain(f, LO, HI, 100.0)
    tw = max(2.0, 2 * df)
    for a, b, _ in lines:
        i0, i1 = int(max(0, a - tw) / df), min(len(g), int((b + tw) / df) + 2)
        g[i0:i1] *= 1.0 - rc_gain(f[i0:i1], a, b, tw)
    if weight:
        g *= (f / 1000.0) ** 2
    r = sfft.irfft(X * g, nf)[:n]
    k = int(EDGE * fs)
    # Parseval: mean square of the unwindowed signal from a band of X
    ms = 2.0 / (nf * np.sum(w * w))
    return r[k:n - k], lines, X, df, ms


def band_ms(X, df, ms, f0, half=5.0):
    i0, i1 = int(max(0.0, f0 - half) / df), int((f0 + half) / df) + 1
    return ms * float(np.sum(np.abs(X[i0:i1]) ** 2))


def detect(r, k, fs):
    sig = 1.4826 * float(np.median(np.abs(r))) + 1e-30
    idx = np.flatnonzero(np.abs(r) > k * sig)
    ev = []
    if idx.size:
        for g in np.split(idx, np.flatnonzero(np.diff(idx) > int(DEAD * fs)) + 1):
            j = g[np.argmax(np.abs(r[g]))]
            ev.append((int(j), float(abs(r[j]) / sig)))
    return sig, ev


def stats(r, k, fs):
    sig, ev = detect(r, k, fs)
    m2 = float(np.mean(r * r)) + 1e-300
    return dict(sigma=sig, rms=m2 ** 0.5, kurt=float(np.mean(r ** 4) / m2 ** 2),
                events=ev, peak=float(np.max(np.abs(r)) / sig), n=len(r))


def analyse(xi, case, k):
    """xi: int16 capture at FS.  Returns a JSON-able dict."""
    n = len(xi) - len(xi) % DEC
    x = xi[:n].astype(np.float32) / 32768.0
    out = dict(case=case, secs=n / FS, clip=float(np.mean(np.abs(x) > 0.98)), carrier=False)
    m = sfft.next_fast_len(-(-n // DEC))
    nf = m * DEC
    X = sfft.rfft(x, nf)
    del x
    df = FS / nf
    nb = n // DEC
    # raw mic audio band, decimated in the frequency domain
    fa = np.arange(m // 2 + 1) * df
    ra, la, _, _, _ = clean(sfft.irfft(X[:m // 2 + 1] * rc_gain(fa, -1.0, 8000.0, 500.0), m)[:nb] / DEC, FSB)
    out["a"], out["lines_a"] = stats(ra, k, FSB), la
    # carrier and complex baseband (analytic band kc +- 8 kHz)
    i0 = int(39000 / df)
    p = np.abs(X[i0:int(41000 / df)]) ** 2
    kc = i0 + int(np.argmax(p))
    carrier = bool(p.max() > 1e4 * np.median(p))
    if not carrier:
        kc = int(round(40000 / df))
    kb = int(8000 / df)
    o = np.arange(-kb, kb + 1)
    o = o[(kc + o >= 0) & (kc + o < len(X))]
    Z = np.zeros(m, np.complex64)
    Z[o % m] = X[kc + o] * rc_gain(np.abs(o) * df, -1.0, HALF_BW, 500.0)
    z = sfft.ifft(Z) * (2.0 / DEC)
    z = z[:nb]
    del Z
    if not carrier:
        rI = clean(z.real.astype(np.float64), FSB)[0]
        rQ = clean(z.imag.astype(np.float64), FSB)[0]
        out["floor"] = float(np.sqrt(0.5 * (np.mean(rI ** 2) + np.mean(rQ ** 2))))
        return out
    out["carrier"] = True
    out["fc"] = kc * df
    f0 = tone_freq(case)
    if f0:
        pc = float(np.sum(np.abs(X[kc - int(5 / df):kc + int(5 / df) + 1]) ** 2))

        def sb(f):
            j = int(round(f / df))
            return 10 * np.log10(float(np.sum(np.abs(X[j - int(5 / df):j + int(5 / df) + 1]) ** 2)) / pc + 1e-30)
        out["sidebands"] = [(sb(out["fc"] - q * f0), sb(out["fc"] + q * f0)) for q in (1, 2, 3)]
    del X
    E = np.abs(z).astype(np.float64)
    ke = int(EDGE * FSB)
    Em = float(np.mean(E[ke:nb - ke]))
    out["Em"], out["dbfs"] = Em, 20 * np.log10(Em + 1e-30)
    re_, le, Xe, dfe, mse = clean(E / Em - 1.0, FSB)
    rb, lb, Xb, _, _ = clean((E / Em) ** 2, FSB, weight=True)
    ph = np.unwrap(np.angle(z).astype(np.float64))
    del z, E
    t = np.linspace(-1.0, 1.0, nb)
    ph -= np.polyval(np.polyfit(t[::64], ph[::64], 2), t)
    rp, lp, _, _, _ = clean(ph, FSB)
    del ph, t
    if f0:
        pt = band_ms(Xe, dfe, mse, f0)
        out["depth"] = float(np.sqrt(2 * pt))
        out["snr_e"] = 10 * np.log10(pt / float(np.mean(re_ ** 2)))
        out["harm"] = [10 * np.log10(band_ms(Xe, dfe, mse, q * f0) / pt + 1e-30) for q in (2, 3, 4, 5)]
        # audible tone vs audible residual (both w^2-weighted, b residual already is)
        out["snr_b"] = 10 * np.log10(band_ms(Xb, dfe, mse, f0) * (f0 / 1000.0) ** 4 / float(np.mean(rb ** 2)))
    del Xe, Xb
    for key, r, ln in (("e", re_, le), ("b", rb, lb), ("p", rp, lp)):
        out[key], out["lines_" + key] = stats(r, k, FSB), ln
    top = sorted(out["b"]["events"], key=lambda ev: -ev[1])[:8]
    h = int(0.001 * FSB)
    out["top"] = [(j, s,
                   100 * float(np.max(np.abs(re_[max(0, j - h):j + h + 1]))),
                   float(np.max(np.abs(rp[max(0, j - h):j + h + 1]))),
                   float(np.max(np.abs(ra[max(0, j - h):j + h + 1])) / out["a"]["sigma"]))
                  for j, s in top]
    return out


def fmt_ch(tag, s, val):
    return "  %s  sigma %-14s rms/sig %5.2f  kurt %7.2f  peak %6.1f sig  events %4d  %7.1f/min" % (
        tag, val, s["rms"] / s["sigma"], s["kurt"], s["peak"], len(s["events"]),
        len(s["events"]) * 60.0 / (s["n"] / FSB))


def report(res, meta, floor):
    print("\n== %s  %.1f s  clip %.4f%%" % (res["case"], res["secs"], 100 * res["clip"]))
    if not res["carrier"]:
        print("  no carrier; baseband floor %.3e FS" % res["floor"])
    else:
        s = "  fc %.2f Hz  carrier %.1f dBFS" % (res["fc"], res["dbfs"])
        if floor:
            s += "  (mic floor in e: %.4f%%)" % (100 * floor / res["Em"])
        print(s)
        if "depth" in res:
            print("  tone depth %.3f  SNR e %.1f dB  audible SNR b %.1f dB  H2..H5 %s dBc" % (
                res["depth"], res["snr_e"], res["snr_b"], " ".join("%.1f" % h for h in res["harm"])))
            print("  carrier sidebands L/U dBc: %s" % "  ".join("%.1f/%.1f" % tuple(q) for q in res["sidebands"]))
        for tag, sc, u in (("e", 100, "%"), ("b", 100, "%"), ("p", 1000, " mrad")):
            print(fmt_ch(tag, res[tag], "%.4f%s" % (res[tag]["sigma"] * sc, u)))
    print(fmt_ch("a", res["a"], "%.1f dBFS" % (20 * np.log10(res["a"]["sigma"]))))
    for tag in ("e", "a"):
        ln = sorted(res.get("lines_" + tag) or [], key=lambda q: -q[2])[:10]
        if ln:
            print("  lines %s: %s" % (tag, " ".join("%.0f(%.0f)" % (0.5 * (q[0] + q[1]), q[2]) for q in ln)))
    st = [s for s in meta.get("stats", []) if "error" not in s]
    if len(st) >= 2:
        d = [st[-1][q] - st[0][q] for q in ("und", "ovr", "loss")]
        dr = (st[-1]["rendered"] - st[0]["rendered"]) / (st[-1]["t"] - st[0]["t"])
        print("  device: und %+d ovr %+d loss %+d  svc max %d  ppm %d..%d  fill %d..%d  rate %+.0f ppm  timeouts %d" % (
            d[0], d[1], d[2], max(s["svc"] for s in st), min(s["ppm"] for s in st), max(s["ppm"] for s in st),
            min(s["fill"] for s in st), max(s["fill"] for s in st), (dr / RATE - 1) * 1e6,
            len(meta["stats"]) - len(st)))
        off = meta.get("t_rec", 0.0) + 0.3
        ch = ["%.1f%s" % (s["t"] - off, q[0]) for s0, s in zip(st, st[1:])
              for q in ("und", "ovr", "loss") if s[q] != s0[q]]
        if ch:
            print("  counter changes at capture t (s, +ffmpeg latency): %s" % " ".join(ch[:16]))
    if res.get("top"):
        print("  top b events:  t(s)   b(sig)   e(%)   p(mrad)  a(sig)")
        for j, s, e, p, aa in res["top"]:
            print("            %8.3f %7.1f %7.3f %8.2f %7.1f" % (j / FSB + EDGE, s, e, 1000 * p, aa))
    if res["secs"] > 40:
        for tag in [t for t in ("b", "e", "a") if t in res]:
            j = np.array([int((q[0] / FSB + EDGE) // 5) for q in res[tag]["events"]], dtype=int)
            cnt = np.bincount(j, minlength=int(res["secs"] // 5) + 1)
            print("  %s events / 5 s: %s" % (tag, "".join(str(min(9, c)) if c else "." for c in cnt)))


def summary(results):
    print("\ncase           e sig%   b sig%   b ev/min  b kurt  a ev/min  SNR b dB")
    for r in results:
        if not r["carrier"]:
            print("%-12s %8s %8s %9s %7s %9.1f" % (r["case"], "-", "-", "-", "-",
                                                  len(r["a"]["events"]) * 60.0 / (r["a"]["n"] / FSB)))
            continue
        rate = lambda s: len(s["events"]) * 60.0 / (s["n"] / FSB)  # noqa: E731
        print("%-12s %8.4f %8.4f %9.1f %7.2f %9.1f %9s" % (
            r["case"], 100 * r["e"]["sigma"], 100 * r["b"]["sigma"], rate(r["b"]), r["b"]["kurt"],
            rate(r["a"]), "%.1f" % r["snr_b"] if "snr_b" in r else "-"))


def stop(dev):
    try:
        dev.request(0x93)
    except TimeoutError:
        pass


def capture(a):
    dev = V7Audio(a.port)
    tmp = tempfile.mkdtemp(prefix="umhpop_")
    caps, meta = {}, {}
    try:
        for case in a.cases.split(","):
            path = os.path.join(tmp, case + ".raw")
            stats_log, th, m = [], None, {}
            stop(dev)
            if case == "off":
                time.sleep(0.5)
                rec = record_start(a.secs + 0.3, path)
            else:
                for q in (0x22, 0x23):
                    try:
                        dev.request(q)
                    except TimeoutError:
                        pass
                cfg = struct.pack("<iiiBBHHH", a.x, 0, a.z, 0, a.level, RATE, PREBUF, 0)
                r = dev.request(0x90, cfg)
                if r[1] != 0x70:
                    raise RuntimeError("configure rejected: %r" % (r,))
                dev.request(0x91)
                th = threading.Thread(target=stream, args=(dev, make_env(case, a.secs + 2.5), stats_log))
                t0 = time.perf_counter()
                th.start()
                time.sleep(0.6)
                m["t_rec"] = time.perf_counter() - t0
                rec = record_start(a.secs + 0.3, path)
            rc = rec.wait()
            if th:
                th.join()
                stop(dev)
            if rc != 0:
                raise RuntimeError("ffmpeg failed (%d)" % rc)
            x = np.fromfile(path, dtype="<i2")[int(0.3 * FS):]
            if len(x) < 4 * FS:
                raise RuntimeError("capture too short: %d samples" % len(x))
            m["stats"] = stats_log
            caps[case], meta[case] = x, m
            print("captured %-12s %.1f s" % (case, len(x) / FS), flush=True)
    finally:
        stop(dev)
        dev.close()
        for fn in os.listdir(tmp):
            os.remove(os.path.join(tmp, fn))
        os.rmdir(tmp)
    return caps, meta


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--cases", default="off,dc64,alt64_65,alt63_64,alt32_96,tone1k,hitone1k")
    ap.add_argument("--secs", type=float, default=12.0, help="capture length; 2*EDGE is not analysed")
    ap.add_argument("--z", type=int, default=1_000_000, help="focus z (um)")
    ap.add_argument("--x", type=int, default=0, help="focus x (um)")
    ap.add_argument("--level", type=int, default=255, help="config level 0..255 (envelope ceiling)")
    ap.add_argument("--k", type=float, default=7.0, help="click threshold, MAD sigmas")
    ap.add_argument("--save", default="", help="store captures (.npz)")
    ap.add_argument("--load", default="", help="analyse stored captures instead of measuring")
    a = ap.parse_args()
    if a.load:
        d = np.load(a.load)
        meta = json.loads(str(d["_meta"]))
        caps = {c: d[c] for c in meta}
    else:
        caps, meta = capture(a)
        if a.save:
            np.savez_compressed(a.save, _meta=json.dumps(meta), **caps)
    results, floor = [], None
    for case in sorted(caps, key=lambda c: c != "off"):
        r = analyse(caps[case], case, a.k)
        if not r["carrier"]:
            floor = r["floor"]
        report(r, meta[case], floor)
        results.append(r)
    summary(results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
