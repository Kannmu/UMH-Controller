#!/usr/bin/env python3
"""Offline calibration workbench for the UMH-84 near-field calibration.

The device can stream a raw dataset with UMH_MSG_CAL_RAW:

    raw[p][m][I,Q]  int16 little-endian
    p = pattern index, m = microphone slot (FPGA order)

The projection sign matrix is not stored; it is regenerated here with the
same splitmix64 generator as Core/Src/us_calibration.c.  This lets us iterate
the solver and fit on a PC first and only port the final algorithm to C.

Usage examples:
    python Utiles/umh_cal_dump_check.py --selftest
    python Utiles/umh_cal_dump_check.py --raw Utiles/cal_capture/umh_raw_level8_gate232.bin
    python Utiles/umh_cal_dump_check.py --raw ... --localize
"""
import argparse, json, math, os, re, struct, sys
import numpy as np

CH = 84
MIC = 4
MASK64 = (1 << 64) - 1
CS_SEED = 0x243F6A8885A308D3
DEFAULT_C = 343.0
DEFAULT_Z = 7.0

# FPGA demodulator slot order -> microphone acoustic-hole coordinates (mm).
MIC_X = np.array([-43.305, 43.298, 0.0, -0.004], dtype=np.float64)
MIC_Y = np.array([24.994, 24.994, 0.0, -49.994], dtype=np.float64)


_ELEMENTS_CACHE = {}


def load_elements(path):
    if path in _ELEMENTS_CACHE:
        return _ELEMENTS_CACHE[path]
    rx = re.compile(r"\{\s*[-0-9]+,\s*[-0-9]+,\s*[-0-9]+,\s*([-0-9]+),\s*([-0-9]+),")
    coords = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            m = rx.search(line)
            if m:
                coords.append((int(m.group(1)) * 0.001, int(m.group(2)) * 0.001))
    if len(coords) != CH:
        raise SystemExit("expected %d element coordinates, got %d" % (CH, len(coords)))
    arr = np.array(coords, dtype=np.float64)
    _ELEMENTS_CACHE[path] = arr
    return arr


def mix32(state):
    state = (state + 0x9E3779B97F4A7C15) & MASK64
    z = state
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    return (z ^ (z >> 31)) & 0xFFFFFFFF, state


def cs_row(pattern):
    state = (CS_SEED ^ (((pattern + 1) & MASK64) * 0xD1B54A32D192ED03)) & MASK64
    bits = []
    for _ in range(3):
        v, state = mix32(state)
        bits.append(v)
    bits[2] &= 0x000FFFFF
    return bits


def projection_matrix(patterns):
    x = np.empty((patterns, CH), dtype=np.float64)
    for p in range(patterns):
        bits = cs_row(p)
        for i in range(CH):
            x[p, i] = -1.0 if ((bits[i >> 5] >> (i & 31)) & 1) else 1.0
    return x


def load_raw(path, patterns=None):
    data = np.fromfile(path, dtype="<i2")
    if data.size % (MIC * 2) != 0:
        raise SystemExit("raw size is not a multiple of 16 bytes")
    total = data.size // (MIC * 2)
    if patterns is not None:
        if patterns > total:
            raise SystemExit("requested %d patterns but file has %d" % (patterns, total))
        total = patterns
    data = data[: total * MIC * 2].reshape(total, MIC, 2)
    return data[:, :, 0].astype(np.float64) + 1j * data[:, :, 1].astype(np.float64)


def load_meta(path):
    if path and os.path.exists(path):
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh)
    return {}


def path_distances(z_mm):
    dx = load_elements(os.path.join(os.path.dirname(__file__), "..", "Reference",
                                    "UMH 7 Element Layout", "umh7_element_map.h"))[:, 0][None, :] - MIC_X[:, None]
    dy = load_elements(os.path.join(os.path.dirname(__file__), "..", "Reference",
                                    "UMH 7 Element Layout", "umh7_element_map.h"))[:, 1][None, :] - MIC_Y[:, None]
    return np.sqrt(dx * dx + dy * dy + z_mm * z_mm)


def wave_k(c_mps):
    return 2.0 * math.pi * 40000.0 / (c_mps * 1000.0)


def centered_ls(x, y):
    xc = x - x.mean(axis=0, keepdims=True)
    yc = y - y.mean(axis=0, keepdims=True)
    z, _, _, _ = np.linalg.lstsq(xc, yc, rcond=None)
    return z  # CH x MIC


def fit_rank1(d, iters=200):
    # d is MIC x CH; model d[m,i] = mu[m] + a[i]*rho[m].
    m_count, n_count = d.shape
    a = np.ones(n_count, dtype=np.complex128)
    rho = np.ones(m_count, dtype=np.complex128)
    mu = np.zeros(m_count, dtype=np.complex128)
    for _ in range(iters):
        num = np.zeros(n_count, dtype=np.complex128)
        den = np.zeros(n_count, dtype=np.float64)
        for m in range(m_count):
            e = d[m] - mu[m]
            w = np.abs(e)
            num += w * e * np.conj(rho[m])
            den += w * np.abs(rho[m]) ** 2
        a = num / (den + 1.0e-30)
        for m in range(m_count):
            e = d[m] - mu[m]
            w = np.abs(e)
            num_m = np.sum(w * e * np.conj(a))
            den_m = np.sum(w * np.abs(a) ** 2)
            rho[m] = num_m / (den_m + 1.0e-30)
        for m in range(m_count):
            mu[m] = np.mean(d[m] - a * rho[m])
    model = mu[:, None] + a[None, :] * rho[:, None]
    rel = float(np.sum(np.abs(d - model) ** 2) /
                (np.sum(np.abs(d - mu[:, None]) ** 2) + 1.0e-30))
    resid = []
    for m in range(m_count):
        numv = d[m] - mu[m]
        mod = a * rho[m]
        ok = (np.abs(numv) > 1.0e-12) & (np.abs(mod) > 1.0e-12)
        resid.extend(np.angle(numv[ok] / mod[ok]).tolist())
    rms = math.degrees(math.sqrt(float(np.mean(np.square(resid))))) if resid else 999.0
    return a, rho, mu, rel, rms


def gauge_phase(phi_rad, coords):
    """Remove a global phase and a linear phase plane, as the firmware does."""
    mean = math.atan2(float(np.mean(np.sin(phi_rad))), float(np.mean(np.cos(phi_rad))))
    v = np.array([math.remainder(p - mean, 2 * math.pi) for p in phi_rad])
    a_mat = np.column_stack([coords[:, 0], coords[:, 1], np.ones(CH)])
    sol, _, _, _ = np.linalg.lstsq(a_mat, v, rcond=None)
    return np.array([math.remainder(v[i] - float(a_mat[i] @ sol), 2 * math.pi)
                     for i in range(CH)])


def correction_bytes(fit_deg):
    q = np.round(-fit_deg * 256.0 / 360.0).astype(np.int64) % 256
    return q.astype(np.uint8)


def solve(raw_path, patterns=None, sign=1, z_mm=DEFAULT_Z, c_mps=DEFAULT_C,
          iters=200, verbose=True):
    meta = load_meta(os.path.splitext(raw_path)[0] + ".json")
    y = load_raw(raw_path, patterns)
    p_used = y.shape[0]
    x = projection_matrix(p_used)
    z = centered_ls(x, y)
    coords = load_elements(os.path.join(os.path.dirname(__file__), "..", "Reference",
                                        "UMH 7 Element Layout", "umh7_element_map.h"))
    dist = path_distances(z_mm)
    d = z.T * np.exp(1j * sign * wave_k(c_mps) * dist)
    a, rho, mu, rel, rms = fit_rank1(d, iters=iters)
    phi = np.angle(a)
    fit_deg = np.degrees(gauge_phase(phi, coords))
    q = correction_bytes(fit_deg)
    if verbose:
        print("raw=%s" % raw_path)
        if meta:
            print("capture level=%s gate_start=%s width=%s burst_us=%s patterns=%s"
                  % (meta.get("level"), meta.get("gate_start"), meta.get("gate_width"),
                     meta.get("burst_us"), meta.get("patterns")))
        print("patterns_used=%d sign=%+d z=%.2fmm c=%.1f m/s iters=%d"
              % (p_used, sign, z_mm, c_mps, iters))
        print("fit relative residual=%.5f  phase RMS=%.2f deg" % (rel, rms))
        print("phase correction bytes:")
        for i in range(0, CH, 28):
            print("  " + " ".join("%3d" % v for v in q[i:i + 28]))
    return dict(meta=meta, y=y, x=x, z=z, q=q, fit_deg=fit_deg, rel=rel, rms=rms,
                a=a, rho=rho, mu=mu, coords=coords)


def scan_geometry(raw_path, patterns=1024, iters=60):
    y = load_raw(raw_path, patterns)
    x = projection_matrix(y.shape[0])
    z = centered_ls(x, y)
    coords = load_elements(os.path.join(os.path.dirname(__file__), "..", "Reference",
                                        "UMH 7 Element Layout", "umh7_element_map.h"))
    best = []
    for sign in (1, -1):
        for c in np.arange(330.0, 361.0, 2.0):
            k = wave_k(float(c))
            for zz in np.arange(0.0, 15.01, 0.5):
                dist = path_distances(float(zz))
                d = z.T * np.exp(1j * sign * k * dist)
                _, _, _, rel, rms = fit_rank1(d, iters=iters)
                best.append((rms, rel, sign, float(c), float(zz)))
    best.sort(key=lambda v: (v[0], v[1]))
    print("geometry scan (rank-1 phase RMS, relative residual, sign, c, z):")
    for row in best[:15]:
        print("  rms=%6.2f rel=%.5f sign=%+d c=%6.1f z=%5.2f" % row)
    return best


def selftest():
    coords = load_elements(os.path.join(os.path.dirname(__file__), "..", "Reference",
                                        "UMH 7 Element Layout", "umh7_element_map.h"))
    rng = np.random.default_rng(1234)
    true_phi = rng.uniform(-math.pi, math.pi, CH)
    true_phi -= math.atan2(np.mean(np.sin(true_phi)), np.mean(np.cos(true_phi)))
    gain = 0.5 + rng.random(CH)
    rho = np.exp(1j * rng.uniform(-math.pi, math.pi, MIC))
    dist = path_distances(DEFAULT_Z)
    z_true = (gain * np.exp(1j * true_phi))[:, None] * rho[None, :] * \
             np.exp(-1j * wave_k(DEFAULT_C) * dist.T)
    patterns = 384
    x = projection_matrix(patterns)
    y = x @ z_true + (rng.normal(scale=0.2, size=(patterns, MIC)) +
                      1j * rng.normal(scale=0.2, size=(patterns, MIC)))
    z_hat = centered_ls(x, y)
    d = z_hat.T * np.exp(1j * wave_k(DEFAULT_C) * dist)
    a, rho_hat, mu, rel, rms = fit_rank1(d, iters=200)
    fit_deg = np.degrees(gauge_phase(np.angle(a), coords))
    q = correction_bytes(fit_deg)
    # Compare against a gauge-corrected truth; global sign is free.
    true_fit = np.degrees(gauge_phase(np.angle(np.exp(1j * true_phi)), coords))
    err = np.array([math.remainder(math.radians(fit_deg[i] - true_fit[i]), 2 * math.pi)
                    for i in range(CH)])
    rms_deg = math.degrees(math.sqrt(float(np.mean(np.square(err)))))
    print("selftest: fit rms=%.2f deg, truth comparison=%.2f deg, rel=%.5f" %
          (rms, rms_deg, rel))
    ok = rms < 8.0 and rms_deg < 5.0 and np.all(q < 256)
    print("selftest:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    default_elements = os.path.join(os.path.dirname(__file__), "..", "Reference",
                                    "UMH 7 Element Layout", "umh7_element_map.h")
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", help="raw UMH_MSG_CAL_RAW capture")
    ap.add_argument("--patterns", type=int, default=None)
    ap.add_argument("--sign", choices=["both", "plus", "minus"], default="plus")
    ap.add_argument("--z", type=float, default=DEFAULT_Z)
    ap.add_argument("--c", type=float, default=DEFAULT_C)
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--localize", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    # Retain a compatible --elements argument even though the helper below
    # always reads the repository table; this keeps old command lines valid.
    ap.add_argument("--elements", default=default_elements)
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.raw:
        ap.error("--raw FILE is required unless --selftest is used")
    if args.localize:
        scan_geometry(args.raw, patterns=args.patterns or 1024)
        return 0
    if args.sign == "both":
        for sign in (1, -1):
            solve(args.raw, args.patterns, sign, args.z, args.c, args.iters)
    else:
        solve(args.raw, args.patterns, 1 if args.sign == "plus" else -1,
              args.z, args.c, args.iters)
    return 0


if __name__ == "__main__":
    sys.exit(main())
