import numpy as np
d = np.load("Utiles/bench/dbg/data/q_fast.npz")
v = d["v"].astype(float)
x = float(d["xinc"])
w = 8
env = np.array([v[i:i+w].max() - v[i:i+w].min() for i in range(len(v)-w)])
print("xinc %.2g  env(100us):" % x)
print(" ".join("%.0f" % q for q in env[::20]))

