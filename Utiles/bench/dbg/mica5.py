import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
tags = ("m3_idle", "m3_const", "m3_post")
P = {}
for t in tags:
    r, x = wavfile.read("Utiles/bench/dbg/data/%s.wav" % t)
    f, p = welch(np.asarray(x, dtype=float), fs=r, nperseg=32768)
    P[t] = p
    print("==", t, "rms %.1f dBFS %.1f" % (np.asarray(x,dtype=float).std(), 20*np.log10(np.asarray(x,dtype=float).std()/32768.0)))

Poff = 0.5*(P["m3_idle"] + P["m3_post"])
bands = ((20,100),(100,500),(500,2000),(2000,5000),(5000,10000),(10000,20000))
for lo,hi in bands:
    m = (f>=lo)&(f<hi)
    a = np.sqrt(np.trapz(P["m3_const"][m], f[m]))
    b = np.sqrt(np.trapz(Poff[m], f[m]))
    print("%5d-%5d Hz  const %.1f dBFS  idle %.1f  diff %+.1f" % (lo, hi, 20*np.log10(max(a,1e-9)/32768.0), 20*np.log10(max(b,1e-9)/32768.0), 20*np.log10(max(a,1e-9)/max(b,1e-9))))

