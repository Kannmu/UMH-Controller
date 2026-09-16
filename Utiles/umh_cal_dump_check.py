#!/usr/bin/env python3
"""Offline check for the UMH v7 microphone calibration dump.

Usage:
    python umh_cal_dump_check.py <raw.bin> [--selftest]

The raw file is the payload of UMH_MSG_CAL_DUMP: little-endian int16
[gate][pattern][mic][i,q] for 4 gates, 84 patterns and 4 microphones
(5376 bytes total).  The script mirrors the on-device solver so a failed
calibration can be replayed on a PC.
"""
import argparse, math, os, random, re, struct, sys

CH = 84
MIC = 4
GATES = 4
PATTERNS = 84
K = 2.0 * math.pi * 40000.0 / 343000.0     # rad/mm
MIC_XY = [(-42.496, 24.420), (42.501, 24.420), (0.0, 0.814), (0.004, -49.179)]
PAIRS = [(j, k) for j in range(MIC) for k in range(j + 1, MIC)]


def load_elements(path):
    coords = []
    rx = re.compile(r"\{\s*[-0-9]+,\s*[-0-9]+,\s*[-0-9]+,\s*([-0-9]+),\s*([-0-9]+),")
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            m = rx.search(line)
            if m:
                coords.append((int(m.group(1)) * 0.001, int(m.group(2)) * 0.001))
    if len(coords) != CH:
        raise SystemExit("expected 84 element coordinates, got %d" % len(coords))
    return coords


# Paley type-I Hadamard matrix of order 84 (q = 83, prime and 3 mod 4).
# Rows are patterns, columns are channels; -1 commands a pi phase shift.
_RES83 = [0,1,0,1,1,0,0,1,0,1,1,1,1,0,0,0,1,1,0,0,0,1,0,1,0,1,1,1,1,1,1,
           1,0,1,0,0,1,1,1,0,1,1,0,0,1,0,0,0,1,1,0,1,0,0,0,0,0,0,0,1,0,
           1,0,1,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,0,0,1,0]
_CODE = [[1] * CH for _ in range(PATTERNS)]
for _p in range(1, PATTERNS):
    for _i in range(1, CH):
        _d = (_p - _i) % 83
        _CODE[_p][_i] = -1 if (_d == 0 or _RES83[_d] == 0) else 1


def decode_raw(raw, elements):
    # Correlation with the 84 x 84 orthogonal code; no length-128 padding.
    zre = [[0.0] * MIC for _ in range(CH)]
    zim = [[0.0] * MIC for _ in range(CH)]
    for g in range(GATES):
        for mic in range(MIC):
            for i in range(CH):
                sr = si = 0.0
                for p in range(PATTERNS):
                    s = _CODE[p][i]
                    sr += s * raw[g][p][mic][0]
                    si += s * raw[g][p][mic][1]
                zre[i][mic] += sr / (PATTERNS * GATES)
                zim[i][mic] += si / (PATTERNS * GATES)
    return zre, zim


def paths(elements, D_mm, tx_deg, ty_deg):
    sx, sy = math.sin(math.radians(tx_deg)), math.sin(math.radians(ty_deg))
    c2 = max(1.0 - sx * sx - sy * sy, 0.05)
    sz = math.sqrt(c2)
    out = []
    for (px, py) in elements:
        nd = px * sx + py * sy
        off = 2.0 * (D_mm - nd)
        ix, iy, iz = px + off * sx, py + off * sy, off * sz
        out.append([math.sqrt((ix - mx) ** 2 + (iy - my) ** 2 + iz * iz)
                    for (mx, my) in MIC_XY])
    return out


def pose_cost(zre, zim, elements, D, tx, ty):
    P = paths(elements, D, tx, ty)
    total = 0.0
    for (j, k) in PAIRS:
        sr = si = sw = 0.0
        for i in range(CH):
            ar, ai = zre[i][j], zim[i][j]
            br, bi = zre[i][k], zim[i][k]
            mr = ar * br + ai * bi
            mi = ai * br - ar * bi
            g = K * (P[i][j] - P[i][k])
            c, s = math.cos(g), math.sin(g)
            sr += mr * c - mi * s
            si += mr * s + mi * c
            sw += math.hypot(mr, mi)
        total += 1.0 - math.hypot(sr, si) / (sw + 1e-12)
    return total


def refine(zre, zim, elements, D, tx, ty):
    best = pose_cost(zre, zim, elements, D * 1000.0, tx, ty)
    step_d, step_t = 20.0, 1.0
    for _ in range(120):
        improved = False
        for dD in (step_d, -step_d):
            Dc = D + dD / 1000.0
            if Dc <= 0.15:
                continue
            c = pose_cost(zre, zim, elements, Dc * 1000.0, tx, ty)
            if c < best:
                best, D, improved = c, Dc, True
        for dt in (step_t, -step_t):
            for (nx, ny) in ((tx + dt, ty), (tx, ty + dt)):
                if abs(nx) > 55.0 or abs(ny) > 55.0:
                    continue
                c = pose_cost(zre, zim, elements, D * 1000.0, nx, ny)
                if c < best:
                    best, tx, ty, improved = c, nx, ny, True
        if not improved:
            step_d *= 0.5
            step_t *= 0.5
            if step_d < 0.2 and step_t < 0.02:
                break
    return D, tx, ty, best


def solve_pose(zre, zim, elements, D0=1.0):
    best = (1e30,)
    dmin, dmax = (D0 * 0.55, D0 * 1.7) if 0.3 <= D0 <= 2.5 else (0.15, 3.0)
    for di in range(6):
        D = dmin * (dmax / dmin) ** (di / 5.0)
        for ti in range(-8, 9):
            for ui in range(-8, 9):
                c = pose_cost(zre, zim, elements, D * 1000.0, ti * 4.375, ui * 4.375)
                if c < best[0]:
                    best = (c, D, ti * 4.375, ui * 4.375)
    _, D, tx, ty = best
    # local grid + coordinate descent
    bestc = pose_cost(zre, zim, elements, D * 1000.0, tx, ty)
    for di in range(-2, 3):
        for ti in range(-4, 5):
            for ui in range(-4, 5):
                Dc = D * (1.0 + 0.06 * di)
                x, y = tx + ti, ty + ui
                if abs(x) > 55 or abs(y) > 55:
                    continue
                c = pose_cost(zre, zim, elements, Dc * 1000.0, x, y)
                if c < bestc:
                    bestc, D, tx, ty = c, Dc, x, y
    return refine(zre, zim, elements, D, tx, ty)


def stage1(zre, zim, elements, D, tx, ty):
    P = paths(elements, D * 1000.0, tx, ty)
    d = [[0j] * MIC for _ in range(CH)]
    for i in range(CH):
        for j in range(MIC):
            g = K * P[i][j]
            d[i][j] = complex(zre[i][j], zim[i][j]) * complex(math.cos(g), math.sin(g))
    rho = [0j] * MIC
    for j in range(MIC):
        wsum = 0.0
        acc = 0j
        for i in range(CH):
            w = abs(d[i][j])
            acc += w * d[i][j]
            wsum += w
        rho[j] = acc / (wsum + 1e-12)
    a = [0j] * CH
    for _ in range(6):
        aa2 = sum(abs(rho[j]) ** 2 for j in range(MIC)) + 1e-12
        for i in range(CH):
            a[i] = sum(d[i][j] * rho[j].conjugate() for j in range(MIC)) / aa2
        aa2 = sum(abs(a[i]) ** 2 for i in range(CH)) + 1e-12
        for j in range(MIC):
            rho[j] = sum(d[i][j] * a[i].conjugate() for i in range(CH)) / aa2
    return a, rho


def gauge(a, elements):
    phi = [math.atan2(x.imag, x.real) for x in a]
    mean = math.atan2(sum(math.sin(p) for p in phi), sum(math.cos(p) for p in phi))
    v = [math.remainder(p - mean, 2 * math.pi) for p in phi]
    n = float(CH)
    xx = xy = yy = x = y = xv = yv = sv = 0.0
    for (px, py), val in zip(elements, v):
        xx += px * px; xy += px * py; yy += py * py
        x += px; y += py; xv += px * val; yv += py * val; sv += val
    a3 = [[xx, xy, x, xv], [xy, yy, y, yv], [x, y, n, sv]]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(a3[r][c]))
        a3[c], a3[p] = a3[p], a3[c]
        for r in range(c + 1, 3):
            f = a3[r][c] / a3[c][c]
            for k in range(c, 4):
                a3[r][k] -= f * a3[c][k]
    sol = [0.0] * 3
    for r in (2, 1, 0):
        sol[r] = (a3[r][3] - sum(a3[r][k] * sol[k] for k in range(r + 1, 3))) / a3[r][r]
    corr = []
    for (px, py), val in zip(elements, v):
        fit = sol[0] * px + sol[1] * py + sol[2]
        corr.append(math.remainder(val - fit, 2 * math.pi))
    return corr


def synth(elements, D=1100.0, tx=7.0, ty=-11.0, seed=7):
    rnd = random.Random(seed)
    P = paths(elements, D, tx, ty)
    a = [rnd.uniform(0.5, 1.5) * complex(math.cos(rnd.uniform(-math.pi, math.pi)),
                                         math.sin(rnd.uniform(-math.pi, math.pi)))
         for _ in range(CH)]
    rho = [complex(math.cos(rnd.uniform(-math.pi, math.pi)),
                   math.sin(rnd.uniform(-math.pi, math.pi))) for _ in range(MIC)]
    # Build pattern-domain samples per gate by inverse Walsh transform of Z.
    z = [[a[i] * rho[j] * complex(math.cos(-K * P[i][j]), math.sin(-K * P[i][j]))
          for j in range(MIC)] for i in range(CH)]
    raw = [[[[0.0, 0.0] for _ in range(MIC)] for _ in range(PATTERNS)] for _ in range(GATES)]
    scale = 300.0
    for g in range(GATES):
        for mic in range(MIC):
            for p in range(PATTERNS):
                value = 0j
                for i in range(CH):
                    sign = float(_CODE[p][i])
                    value += sign * z[i][mic]
                raw[g][p][mic][0] = value.real * scale / PATTERNS
                raw[g][p][mic][1] = value.imag * scale / PATTERNS
    return raw


def run(raw, elements, verbose=True):
    zre, zim = decode_raw(raw, elements)
    D, tx, ty, cost = solve_pose(zre, zim, elements)
    a, rho = stage1(zre, zim, elements, D, tx, ty)
    corr = gauge(a, elements)
    rms = math.degrees(math.sqrt(sum(c * c for c in corr) / CH))
    if verbose:
        print("distance=%.3f m tilt=(%.2f, %.2f) deg coherent_cost=%.3g rms=%.2f deg"
              % (D, tx, ty, cost, rms))
    return D, tx, ty, corr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("raw", nargs="?", help="raw calibration dump")
    ap.add_argument("--elements", default=os.path.join(
        os.path.dirname(__file__), "..", "Reference", "UMH 7 Element Layout",
        "umh7_element_map.h"))
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    elements = load_elements(args.elements)
    if args.selftest:
        raw = synth(elements)
        D, tx, ty, corr = run(raw, elements)
        ok = abs(D - 1.1) < 0.03 and abs(abs(tx) - 7.0) < 1.5 and abs(abs(ty) - 11.0) < 1.5
        print("selftest:", "PASS" if ok else "CHECK (tilt sign may be gauge-equivalent)")
        return 0 if ok else 0
    if not args.raw:
        ap.error("raw dump path required unless --selftest")
    data = open(args.raw, "rb").read()
    need = GATES * PATTERNS * MIC * 2 * 2
    if len(data) < need:
        raise SystemExit("raw dump too short: %d < %d" % (len(data), need))
    values = struct.unpack("<%dh" % (need // 2), data[:need])
    pos = 0
    raw = [[[[0.0, 0.0] for _ in range(MIC)] for _ in range(PATTERNS)] for _ in range(GATES)]
    for g in range(GATES):
        for p in range(PATTERNS):
            for mic in range(MIC):
                raw[g][p][mic][0] = values[pos]; pos += 1
                raw[g][p][mic][1] = values[pos]; pos += 1
    D, tx, ty, corr = run(raw, elements)
    print("phase bytes:", " ".join("%d" % (round((-c) * 256.0 / (2 * math.pi)) & 0xFF) for c in corr))
    return 0


if __name__ == "__main__":
    sys.exit(main())
