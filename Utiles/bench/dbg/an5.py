import numpy as np
for tag in ("avg_static_ch2", "avg_am_ch2"):
    d = np.load("Utiles/bench/dbg/data/%s.npz" % tag)
    v = d["v"]
    x = float(d["xinc"])
    t = np.arange(len(v)) * x
    A = np.c_[np.cos(2*np.pi*40000*t), np.sin(2*np.pi*40000*t), np.ones(len(t))]
    c, _, _, _ = np.linalg.lstsq(A, v, rcond=None)
    r = v - A.dot(c)
    print(tag, "xinc", x, "amp %.3f" % np.hypot(c[0], c[1]), "res_rms %.4f res_max %.4f" % (r.std(), np.abs(r).max()))

    ph = (np.mod(t, 5e-5)/5e-5*32).astype(int)
    h1 = [r[ph == k].mean() for k in range(16)]
    h2 = [r[ph == k+16].mean() for k in range(16)]
    if tag.endswith("static_ch2") or True:
        print("   half1", " ".join("%.3f" % q for q in h1))
        print("   half2", " ".join("%.3f" % q for q in h2))

