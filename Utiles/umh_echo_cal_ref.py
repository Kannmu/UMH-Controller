"""Reference implementation of the firmware axial table-echo calibration.

Mirrors Core/Src/us_calibration.c (cal_echo_*) step by step so the firmware
result can be regression-checked offline:

  A  onset:   64 x 25 us profile of a few channels -> table distance V
  B  scan:    per channel 16 x 50 us gates around the echo onset
              baseline = pre-echo steady state, echo = +100..+400 us
  C  solve:   y = (echo - base) * r_e * exp(-j k r_e) inside a 17 deg cone,
              per-mic common phase + per-channel phase, gauge (1, x, y, x^2+y^2),
              Wiener shrinkage by the per-channel standard error
  D  health:  dead channels from the echo magnitude

    python Utiles/umh_echo_cal_ref.py --scan Utiles/cal_capture/scan_l16_a.npz
    python Utiles/umh_echo_cal_ref.py --live            (drives the device via CAL_PROBE)
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

from umh_echo_geom import ELEM_XY, MIC_XY, SRC_Z

C_MM_US = 0.343
K = 2 * np.pi * 40000 / 343000.0          # rad/mm
ONSET_LAT_US = 95.0                        # t50 detector latency (A-profile)
A_START, A_GATES = 8, 64                   # A: 200 us .. 1775 us, 25 us gates
B_GATES, B_W = 16, 2                       # B: 16 gates x 50 us
B_PRE = 14                                 # B start = t_on - 14*25 us
CONE_H_MM, MIN_RD_MM, BASE_RATIO = 60.0, 14.0, 0.6
A_CHANNELS_R = (25.0, 35.0)                # A channels: ring radius from centre


def hdist():
    return np.hypot(ELEM_XY[None, :, 0] - MIC_XY[:, None, 0], ELEM_XY[None, :, 1] - MIC_XY[:, None, 1])


def a_channels():
    r = np.hypot(ELEM_XY[:, 0], ELEM_XY[:, 1])
    return [i for i in range(84) if A_CHANNELS_R[0] <= r[i] <= A_CHANNELS_R[1]][:6]


def onset_gate(z):
    """z: complex [gates] profile.  First gate where the deviation from the
    preceding plateau reaches half of its maximum (the echo step)."""
    n = len(z)
    dev = np.zeros(n)
    for g in range(6, n):
        dev[g] = abs(z[g] - z[g - 6:g - 3].mean())
    mx = dev.max()
    if mx <= 0:
        return None, 0.0
    g = int(np.argmax(dev >= 0.5 * mx))
    # linear interpolation between g-1 and g
    if g > 0 and dev[g] > dev[g - 1]:
        frac = (0.5 * mx - dev[g - 1]) / (dev[g] - dev[g - 1])
    else:
        frac = 1.0
    return g - 1 + frac, mx


def table_from_profiles(prof, chans):
    """prof: [nch, gates, mic] complex, A plan.  Returns (V_mm, t_on_us list)."""
    h = hdist()
    rd = np.sqrt(h ** 2 + SRC_Z ** 2)
    vs = []
    for k, i in enumerate(chans):
        for m in range(4):
            if rd[m, i] < 25:
                continue
            g, mx = onset_gate(prof[k, :, m])
            if g is None or mx < 4:
                continue
            t = (A_START + g) * 25.0
            re = (t - ONSET_LAT_US) * C_MM_US
            if re <= h[m, i]:
                continue
            vs.append((np.sqrt(re ** 2 - h[m, i] ** 2) - SRC_Z) / 2)
    if len(vs) < 4:
        return None
    return float(np.median(vs))


def b_plan(V):
    """Common gate plan for the per-channel scan."""
    h0 = 30.0
    t_on = np.sqrt(h0 ** 2 + (2 * V + SRC_Z) ** 2) / C_MM_US + ONSET_LAT_US
    start = int(round(t_on / 25.0)) - B_PRE
    return max(start, 6), t_on


def b_windows(start, t_on, V):
    t = (start + np.arange(B_GATES) * B_W) * 25.0          # gate start times
    base = (t >= t_on - 300) & (t + B_W * 25 <= t_on - 50)
    second = 2 * (2 * V + SRC_Z) / C_MM_US                 # 2nd round trip - 1st
    end = min(t_on + 400, t_on + 0.8 * second)
    echo = (t >= t_on + 100) & (t + B_W * 25 <= end)
    return base, echo


def solve(e, b, V):
    """e,b: complex [4,84] echo/base.  Returns (q rad, info)."""
    h = hdist()
    rd = np.sqrt(h ** 2 + SRC_Z ** 2)
    re = np.sqrt(h ** 2 + (2 * V + SRC_Z) ** 2)
    y = (e - b) * re * np.exp(-1j * K * re)
    mag = np.abs(e - b)
    live = mag.max(0) >= 0.15 * np.median(mag.max(0))
    w = (h < CONE_H_MM) & (rd > MIN_RD_MM) & (np.abs(b) < BASE_RATIO * np.abs(e)) & live[None, :]
    u = np.where(w, y / np.maximum(np.abs(y), 1e-12), 0)
    s = np.ones(84, complex)
    for _ in range(30):
        c = np.angle((u * np.conj(s)[None]).sum(1))
        v = u * np.exp(-1j * c)[:, None]
        s = v.sum(0)
    n = w.sum(0)
    ok = n > 0
    ph = np.where(ok, np.angle(s), 0)
    res = np.angle(v * np.exp(-1j * ph)[None])              # pair residual vs. the fit itself
    x, yy = ELEM_XY[:, 0], ELEM_XY[:, 1]
    G = np.c_[np.ones(84), x, yy, x * x + yy * yy]
    sw = np.sqrt(n)
    for _ in range(3):
        wt = sw * (np.abs(ph) < np.pi / 2)                  # ignore gross faults in the gauge
        sol = np.linalg.lstsq(G[ok] * wt[ok, None], ph[ok] * wt[ok], rcond=None)[0]
        ph = np.angle(np.exp(1j * (ph - G @ sol)))
        ph[~ok] = 0
    dof = max(int(w.sum()) - int(ok.sum()) - 4, 1)
    sd2 = float((w * res ** 2).sum() / dof)
    a = np.clip(1 - sd2 / np.maximum(n, 1) / np.maximum(ph ** 2, 1e-9), 0, 1)
    q = np.where(ok, -a * ph, 0)
    info = dict(pairs=int(w.sum()), covered=int(ok.sum()), sd_deg=np.degrees(np.sqrt(sd2)),
                raw_rms=np.degrees(np.sqrt((ph[ok] ** 2).mean())),
                q_rms=np.degrees(np.sqrt((q[ok] ** 2).mean())), live=live, n=n)
    return q, info


def from_scan(path, V=None):
    d = np.load(path)
    z = np.transpose(d["z"], (0, 2, 1))  # [ch, mic, gate] 25 us from start
    s0 = int(d["start"])
    chans = a_channels()
    # A: emulate profile of the A channels
    prof = np.stack([d["z"][i, A_START - s0:A_START - s0 + A_GATES, :] for i in chans])
    Vd = table_from_profiles(prof, chans)
    if V is None:
        V = Vd
    start, t_on = b_plan(V)
    # B: emulate 16 x 50 us gates from the 25 us scan
    t25 = (s0 + np.arange(z.shape[2])) * 25.0
    e = np.zeros((4, 84), complex)
    b = np.zeros((4, 84), complex)
    base, echo = b_windows(start, t_on, V)
    tb = (start + np.arange(B_GATES) * B_W) * 25.0
    for m in range(4):
        for i in range(84):
            gz = np.array([z[i, m, (t25 >= t) & (t25 < t + B_W * 25)].mean() for t in tb])
            b[m, i] = gz[base].mean()
            e[m, i] = gz[echo].mean()
    return Vd, V, start, t_on, e, b


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan", nargs="*", default=[])
    ap.add_argument("--save", default=None)
    a = ap.parse_args(argv)
    qs = []
    for p in a.scan:
        Vd, V, start, t_on, e, b = from_scan(p)
        q, info = solve(e, b, V)
        qs.append(q)
        print("%s: V_detect %.1f mm  B start %d (t_on %.0f us)  pairs %d covered %d  sd %.1f  raw %.1f  q %.1f deg  dead %s"
              % (p, Vd, start, t_on, info["pairs"], info["covered"], info["sd_deg"], info["raw_rms"],
                 info["q_rms"], list(np.where(~info["live"])[0])))
        print("   |q|>30deg:", [(i, round(float(np.degrees(q[i])))) for i in np.where(np.abs(q) > np.radians(30))[0]])
    for k in range(1, len(qs)):
        dd = np.angle(np.exp(1j * (qs[k] - qs[0])))
        print("repeat %d vs 0: rms %.1f deg" % (k, np.degrees(np.sqrt((dd ** 2).mean()))))
    if a.save and qs:
        np.save(a.save, qs[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
