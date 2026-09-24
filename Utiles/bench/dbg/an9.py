import numpy as np
for tag in ("d84_128_ch2", "d84_128_ch1"):
    d = np.load("Utiles/bench/dbg/data/%s.npz" % tag)
    v = d["v"] - d["v"].mean()
    x = float(d["xinc"])
    w = np.hanning(len(v))
    sp = np.abs(np.fft.rfft(v*w)) * 2.0 / w.sum()
    f = np.fft.rfftfreq(len(v), d=x)
    print("==", tag)

    for f0 in (7870, 24100, 32000, 40000, 47900, 55800, 80000):
        i = int(np.argmin(np.abs(f-f0)))
        print("  %7.0f Hz %8.3f" % (f[i], sp[i]))
    idx = np.argsort(sp*(f>5000))[::-1][:6]
    for i in idx:
        print("   pk %7.0f Hz %8.3f" % (f[i], sp[i]))

