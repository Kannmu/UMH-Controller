import numpy as np, glob, os
def load(p):
    d = np.load(p); return d["t"], d["v"], float(d["xinc"])
for p in sorted(glob.glob("Utiles/bench/dbg/data/*.npz")):
    t, v, x = load(p)
    ac = v - v.mean()
    w = np.hanning(len(ac))
    sp = np.abs(np.fft.rfft(ac*w))*2/w.sum()
    f = np.fft.rfftfreq(len(ac), d=x)

    peaks = []
    for f0 in (40000., 80000., 120000., 160000.):
        i = int(np.argmin(np.abs(f-f0)))
        peaks.append(round(float(sp[i]),2))
    print(os.path.basename(p), "x=%.3g n=%d pp=%.2f rms=%.2f" % (x, len(v), ac.max()-ac.min(), ac.std()), peaks)

