import numpy as np
d = np.load("Utiles/bench/dbg/data/q_slow.npz")
v = d["v"]
x = float(d["xinc"])
e = np.convolve(np.abs(v - v.mean()), np.ones(5)/5.0, mode="same")
print("xinc", x, "n", len(e))
print(" ".join("%.0f" % q for q in e[::20]))

