import numpy as np
d = np.load("Utiles/bench/dbg/data/q_slow.npz")
v = d["v"]
x0 = float(d["xinc"])
dc = np.convolve(v, np.ones(25)/25.0, mode="same")
e = np.abs(v - dc)
# find falling edges: e goes high->low
hi = e > 12.0
edges = np.where(hi[1:] & ~hi[:-1])[0] + 1
print("falling edges at", edges[:8], "xinc", x0)
if len(edges) > 1:
    print("period samples", np.diff(edges)[:6])

if len(edges):
    k = edges[0]
    print("decay samples:", " ".join("%.1f" % q for q in e[k:k+14]))
    seg = e[k:k+12]
    ok = seg > 1.0
    if ok.sum() > 3:
        n = np.arange(ok.sum())
        slope = np.polyfit(n, np.log(seg[ok]), 1)[0]
        tau_s = -1.0/slope * x0
        f0 = 40900.0
        print("tau=%.1f us  Q=pi*f0*tau=%.1f  BW=%.0f Hz" % (tau_s*1e6, np.pi*f0*tau_s, 1.0/(np.pi*tau_s)))

