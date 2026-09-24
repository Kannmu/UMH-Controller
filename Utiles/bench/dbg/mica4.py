import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
d = {}
for tag in ("mic_idle", "mic_tone", "mic_post"):
    r, x = wavfile.read("Utiles/bench/dbg/data/%s.wav" % tag)
    f, P = welch(np.asarray(x, dtype=float), fs=r, nperseg=32768)
    d[tag] = (f, P, r)
    print("==", tag, "rms %.1f" % np.asarray(x, dtype=float).std())

f, Pt, r = d["mic_tone"]
f, Pi, _ = d["mic_idle"]
f, Pp, _ = d["mic_post"]
def at(fq):
    i = int(np.argmin(np.abs(f-fq)))
    return 10*np.log10(max(Pt[i],1e-9)), 10*np.log10(max(0.5*(Pi[i]+Pp[i]),1e-9))
for fq in (500, 1000, 2000, 3000, 4000, 5000, 8000, 12000):
    a, b = at(fq)
    print("%6d Hz  tone %.1f dB  idle %.1f dB  diff %+.1f" % (fq, a, b, a-b))

