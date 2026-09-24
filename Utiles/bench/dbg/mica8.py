import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
for tag in ("m5_off44", "m5_on44", "m5_on96", "m5_off96"):
    r, x = wavfile.read("Utiles/bench/dbg/data/%s.wav" % tag)
    f, P = welch(np.asarray(x, dtype=float), fs=r, nperseg=65536)
    m = (f>500)&(f<22000)
    fm, Pm = f[m], P[m]
    idx = np.argsort(Pm)[::-1][:4]
    print("==", tag, "fs", r, "rms %.1f" % np.asarray(x,dtype=float).std())

    for i in idx:
        print("   %8.0f Hz  %7.1f dBFS" % (fm[i], 10*np.log10(max(Pm[i],1e-12))-20*np.log10(32768.0)))

