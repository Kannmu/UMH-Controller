import numpy as np
for tag in ("avg_static_ch2", "avg_am_ch2"):
    d = np.load("Utiles/bench/dbg/data/%s.npz" % tag)
    v = d["v"] - d["v"].mean()
    x = float(d["xinc"])
    w = np.hanning(len(v))
    sp = np.abs(np.fft.rfft(v*w)) * 2.0 / w.sum()
    f = np.fft.rfftfreq(len(v), d=x)
    print("==", tag, "carrier %.3f" % sp[int(np.argmin(np.abs(f-40000)))])
    for f0 in (24000, 27000, 30000, 32000, 33000, 47000, 48000, 50000, 52000, 56000):
        i = int(np.argmin(np.abs(f-f0)))
        print("   %7.0f Hz  %8.4f" % (f[i], sp[i]))

