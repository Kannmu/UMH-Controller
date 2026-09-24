import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
for tag in ("m4_idle", "m4_all128", "m4_post"):
    r, x = wavfile.read("Utiles/bench/dbg/data/%s.wav" % tag)
    f, P = welch(np.asarray(x, float), fs=r, nperseg=32768)
    m = (f>2000)&(f<20000)

    m &= ~((f>7500)&(f<8300))
    m &= ~((f>15400)&(f<16200))
    db = 10*np.log10(np.maximum(P[m], 1e-20)) - 20*np.log10(32768.0)
    print("==", tag, "median %.1f p10 %.1f p90 %.1f" % (np.median(db), np.percentile(db,10), np.percentile(db,90)))

