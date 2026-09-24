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

def measure(tag, levels, phases, secs=4):
    us.send_dense(l, levels, phases)
    time.sleep(0.4)
    path = "Utiles/bench/dbg/data/%s.wav" % tag
    rec(path, secs)
    r, x = wavfile.read(path)
    f, P = welch(np.asarray(x, float), fs=r, nperseg=32768)
    m = (f>2000)&(f<20000)
    m &= ~((f>7500)&(f<8300))
    m &= ~((f>15400)&(f<16200))
    db = 10*np.log10(np.maximum(P[m], 1e-20)) - 20*np.log10(32768.0)
    print("%-8s median %.1f p10 %.1f" % (tag, np.median(db), np.percentile(db,10)))

l = UmhLink("COM4")
def lv(idx, lev=128):
    a = [0]*84
    for i in idx: a[i] = lev
    return a
rng = np.random.RandomState(7)
ph_rnd = [int(x) for x in rng.randint(0, 256, 84)]
ph0 = [0]*84

try:
    us.stop_static(l); time.sleep(0.4)
    measure("n_off", [0]*84, ph0)
    measure("n_1", lv([75]), ph0)
    measure("n_21", lv(range(21)), ph0)

    measure("n_42", lv(range(42)), ph0)
    measure("n_84", lv(range(84)), ph0)
    measure("n_84r", lv(range(84)), ph_rnd)
finally:
    us.stop_static(l); l.close()

