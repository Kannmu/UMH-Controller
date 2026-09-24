import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
bands = ((20,100),(100,500),(500,2000),(2000,5000),(5000,10000),(10000,20000))
for tag in ("mic_off","mic_on75","mic_off2"):
    r,x = wavfile.read("Utiles/bench/dbg/data/%s.wav" % tag)
    x = np.asarray(x, dtype=float)
    f,P = welch(x, fs=r, nperseg=16384)
    print("==", tag, "rms=%.1f dBFS=%.1f" % (x.std(), 20*np.log10(max(x.std(),1e-9)/32768.0)))

    for lo,hi in bands:
        m = (f>=lo)&(f<hi)
        br = np.sqrt(np.trapz(P[m], f[m]))
        print("   %5d-%5d: %6.1f" % (lo, hi, 20*np.log10(max(br,1e-9)/32768.0)))

