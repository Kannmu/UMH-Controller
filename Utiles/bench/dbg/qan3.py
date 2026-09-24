import numpy as np
from scipy.signal import hilbert
d = np.load("Utiles/bench/dbg/data/q_fast.npz")
v = d["v"].astype(float)
x = float(d["xinc"])
env = np.abs(hilbert(v - v.mean()))
print("xinc", x, "n", len(v))
print(" ".join("%.0f" % q for q in env[::20]))

