import numpy as np
for tag in ("ch0_128_fast", "all_16_fast", "ch0_128_slow"):
    d = np.load("Utiles/bench/dbg/data/%s.npz" % tag)
    v = d["v"]; x = float(d["xinc"])
    print("==", tag, "xinc", x, "n", len(v), "min", round(float(v.min()),2), "max", round(float(v.max()),2))
    step = max(1, len(v)//24)
    print(" ".join("%.1f" % q for q in v[::step]))

