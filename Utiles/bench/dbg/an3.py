import numpy as np
for tag in ("am_const255", "d75_128"):
    d = np.load("Utiles/bench/dbg/data/%s.npz" % tag)
    vd = d["v1"] - d["v2"]
    x = float(d["xinc"])
    ac = vd - vd.mean()
    w = np.hanning(len(ac))
    sp = np.abs(np.fft.rfft(ac*w))*2.0/w.sum()
    f = np.fft.rfftfreq(len(ac), d=x)
    print("==", tag, "rms=%.2f" % ac.std())

    m = f > 2000
    idx = np.argsort(sp*m)[::-1][:14]
    for i in idx:
        if sp[i] > 0.05:
            print("   %8.0f Hz  %6.2f" % (f[i], sp[i]))

