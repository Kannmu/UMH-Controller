"""Offline table-echo phase/gain fit for CAL_PROBE channel scans.

    python Utiles/umh_echo_fit.py Utiles/cal_capture/scan_l16_a.npz [more.npz ...]
"""
from __future__ import annotations

import sys

import numpy as np

sys.path.insert(0, "Utiles")
from umh_echo_geom import (ELEM_XY, C_MM_US, TABLE_MM, r_direct, r_echo,  # noqa: E402
                           ref_phase_bytes)

K = 2 * np.pi * 40000 / 343000.0     # rad/mm


def echo_phasors(npz, lat=80.0, skip=75.0, width=350.0, pre=125.0, table=TABLE_MM):
    d = np.load(npz)
    z = d["z"]
    t = int(d["start"]) * 25 + 25 * np.arange(z.shape[1])
    re = r_echo(table)
    e = np.zeros((4, 84), complex)
    b = np.zeros((4, 84), complex)
    for ch in range(84):
        for m in range(4):
            ta = re[m, ch] / C_MM_US + lat
            win = (t >= ta + skip) & (t < ta + skip + width)
            bw = (t >= ta - pre) & (t < ta - 12)
            e[m, ch] = z[ch, win, m].mean()
            b[m, ch] = z[ch, bw, m].mean() if bw.any() else 0
    return e, b


def gauge(ph):
    """Remove common phase and plane (x, y) from a phase vector (rad); return wrapped residual."""
    zc = np.exp(1j * ph)
    best = None
    x, y = ELEM_XY[:, 0], ELEM_XY[:, 1]
    for gx in np.linspace(-0.2, 0.2, 81):
        for gy in np.linspace(-0.2, 0.2, 81):
            w = zc * np.exp(-1j * (gx * x + gy * y))
            s = abs(w.sum())
            if best is None or s > best[0]:
                best = (s, gx, gy)
    _, gx, gy = best
    # refine with LS on unwrapped residual
    for _ in range(3):
        w = zc * np.exp(-1j * (gx * x + gy * y))
        c = np.angle(w.sum())
        r = np.angle(w * np.exp(-1j * c))
        A = np.c_[np.ones(84), x, y]
        sol = np.linalg.lstsq(A, r, rcond=None)[0]
        gx += sol[1]
        gy += sol[2]
    w = zc * np.exp(-1j * (gx * x + gy * y))
    c = np.angle(w.sum())
    return np.angle(w * np.exp(-1j * c)), (gx, gy)


def fit(e, weight, sign=1.0, table=TABLE_MM, iters=60):
    re = r_echo(table)
    g = np.exp(-1j * sign * K * re) / re
    y = e / g                       # = A_i R_m
    R = np.ones(4, complex)
    A = np.zeros(84, complex)
    for _ in range(iters):
        A = (weight * np.conj(R)[:, None] * y).sum(0) / ((weight * abs(R)[:, None] ** 2).sum(0) + 1e-12)
        R = (weight * np.conj(A)[None, :] * y).sum(1) / ((weight * abs(A)[None, :] ** 2).sum(1) + 1e-12)
        R *= np.exp(-1j * np.angle(R[2])) / max(abs(R).max(), 1e-12)
    model = A[None, :] * R[:, None]
    res = np.angle(y * np.conj(model))
    rms = np.degrees(np.sqrt((weight * res ** 2).sum() / weight.sum()))
    return A, R, rms, res


def weights(e, b, direct_frac=0.5):
    w = np.ones_like(abs(e))
    w[abs(b) > direct_frac * abs(e)] = 0.0   # direct-dominated or saturated pairs
    w[r_direct() < 25] = 0.0
    return w


def analyse(path, verbose=True, **kw):
    e, b = echo_phasors(path, **kw)
    ee = e - b
    w = weights(e, b)
    out = {}
    for s in (1.0, -1.0):
        A, R, rms, _ = fit(ee, w, s)
        out[s] = (A, R, rms)
    s = min(out, key=lambda k: out[k][2])
    A, R, rms = out[s]
    q = -np.angle(A)                        # correction (rad), command convention
    qg, plane = gauge(q)
    if verbose:
        print("%s: sign %+d rms %.1f deg (other %.1f), pairs %d/336, |R| %s" % (
            path, s, rms, out[-s][2], int(w.sum()), np.round(abs(R) / abs(R).max(), 2)))
    return qg, abs(A), rms, s


def compare(q, qref, label):
    d, _ = gauge(q - qref)
    print("%-28s vs: rms %.1f deg  max %.1f deg" % (label, np.degrees(np.sqrt((d ** 2).mean())),
                                                   np.degrees(abs(d).max())))


def main(paths):
    ref = ref_phase_bytes() * 2 * np.pi / 256
    refg, _ = gauge(ref)
    res = [analyse(p) for p in paths]
    for (q, a, rms, s), p in zip(res, paths):
        compare(q, refg, p.split("/")[-1] + " ref")
        compare(q, -refg, p.split("/")[-1] + " -ref")
        print("  gain spread (max/min) %.2f, std/mean %.2f" % (a.max() / a.min(), a.std() / a.mean()))
    for i in range(1, len(res)):
        compare(res[i][0], res[0][0], "repeat %d vs 0" % i)
        g0, g1 = res[0][1] / res[0][1].mean(), res[i][1] / res[i][1].mean()
        print("  gain repeat rel rms %.3f" % np.sqrt(((g1 - g0) ** 2).mean()))


if __name__ == "__main__":
    main(sys.argv[1:])
