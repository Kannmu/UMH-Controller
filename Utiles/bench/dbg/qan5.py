import numpy as np
d = np.load("Utiles/bench/dbg/data/q_fast.npz")
v = d["v"].astype(float)
w = 8
env = np.array([v[i:i+w].max() - v[i:i+w].min() for i in range(len(v)-w)])
print(" ".join("%.0f" % q for q in env[490:560]))

