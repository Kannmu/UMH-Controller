import numpy as np
d = np.load("Utiles/bench/dbg/data/avg1024_ch2.npz")
v = d["v"] - d["v"].mean()
x = float(d["xinc"])
w = np.hanning(len(v))
sp = np.abs(np.fft.rfft(v*w)) * 2.0 / w.sum()
f = np.fft.rfftfreq(len(v), d=x)
i0 = int(np.argmin(np.abs(f-40000)))
print("bin", f[i0], "carrier %.4f" % sp[i0])
for k in range(i0-14, i0+15):
    print("  %8.0f Hz  %9.4f  (%.4f%%)" % (f[k], sp[k], 100*sp[k]/sp[i0]))

