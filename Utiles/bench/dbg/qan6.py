import numpy as np
d = np.load("Utiles/bench/dbg/data/q2_b.npz")
v = d["v"].astype(float)
x = float(d["xinc"])
w = 8
env = np.array([v[i:i+w].max() - v[i:i+w].min() for i in range(len(v)-w)])
hi = env > 15
fall = np.where(hi[1:] & ~hi[:-1])[0] + 1
rise = np.where(~hi[1:] & hi[:-1])[0] + 1
print("x", x, "fall", fall[:6], "rise", rise[:6])
print("env100us", " ".join("%.0f" % q for q in env[::20]))

