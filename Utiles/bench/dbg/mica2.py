import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
def psd(path):
    r,x = wavfile.read(path)
    f,P = welch(np.asarray(x,dtype=float), fs=r, nperseg=65536)
    return f,P
f,P0 = psd("Utiles/bench/dbg/data/mic_off.wav")
f,P1 = psd("Utiles/bench/dbg/data/mic_on75.wav")
f,P2 = psd("Utiles/bench/dbg/data/mic_off2.wav")
Poff = 0.5*(P0+P2)

m = (f>10)&(f<5000)
ratio = 10*np.log10(np.maximum(P1[m],1e-9)/np.maximum(Poff[m],1e-9))
idx = np.argsort(ratio)[::-1][:25]
for i in idx:
    print("%8.1f Hz  %6.1f dB" % (f[m][i], ratio[i]))
print("overall 20-5000 dB:", round(10*np.log10(np.trapz(P1[m],f[m])/np.trapz(Poff[m],f[m])),2))

