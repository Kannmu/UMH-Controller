import subprocess, time, sys
sys.path.insert(0, "Utiles/bench")
import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
import umh_static as us
from umh_link import UmhLink

FF = r"d:\Software\ffmpeg-6.0-essentials_build\bin\ffmpeg.exe"
MIC = "audio=Microphone (Realtek High Definition Audio)"
def rec(path, secs):
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-f", "dshow", "-i", MIC, "-t", str(secs), "-ac", "1", "-ar", "48000", "-y", path], check=True)

def measure(tag, lev, secs=4):
    us.send_dense(l, [lev]*84, [0]*84)
    time.sleep(0.4)
    path = "Utiles/bench/dbg/data/%s.wav" % tag
    rec(path, secs)
    r, x = wavfile.read(path)
    f, P = welch(np.asarray(x, float), fs=r, nperseg=32768)

    db = 10*np.log10(np.maximum(P, 1e-20)) - 20*np.log10(32768.0)
    out = []
    for lo, hi in ((2000,5000),(5000,7500),(8300,15400)):
        m = (f>lo)&(f<hi)
        out.append(np.median(db[m]))
    print("%-8s 2-5k %.1f 5-7.5k %.1f 8-15k %.1f" % (tag, out[0], out[1], out[2]))

l = UmhLink("COM4")
try:
    us.stop_static(l); time.sleep(0.4)
    measure("L8", 8)
    measure("L32", 32)
    measure("L64", 64)
    measure("L128", 128)
finally:
    us.stop_static(l); l.close()

