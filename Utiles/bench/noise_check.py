# UMH-7 audible hiss check: mic floor idle vs all-84 static carrier.
import subprocess, sys, time
sys.path.insert(0, "Utiles/bench")
import numpy as np
from scipy.io import wavfile
from scipy.signal import welch
import umh_static as us
from umh_link import UmhLink

FF = r"d:\Software\ffmpeg-6.0-essentials_build\bin\ffmpeg.exe"
MIC = "audio=Microphone (Realtek High Definition Audio)"
PORT = "COM4"
def rec(path, secs):
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-f", "dshow", "-i", MIC, "-t", str(secs), "-ac", "1", "-ar", "48000", "-y", path], check=True)

def floor_db(path):
    r, x = wavfile.read(path)
    f, P = welch(np.asarray(x, float), fs=r, nperseg=32768)
    m = (f>2000)&(f<20000)
    m &= ~((f>7500)&(f<8300))
    m &= ~((f>15400)&(f<16200))
    return 10*np.log10(np.maximum(P[m], 1e-20)) - 20*np.log10(32768.0)

def main():
    l = UmhLink(PORT)
    try:
        us.stop_static(l); time.sleep(0.5)
        rec("Utiles/bench/dbg/data/chk_off.wav", 4)
        a = np.median(floor_db("Utiles/bench/dbg/data/chk_off.wav"))

        us.send_dense(l, [128]*84, [0]*84); time.sleep(0.5)
        rec("Utiles/bench/dbg/data/chk_on.wav", 4)
        b = np.median(floor_db("Utiles/bench/dbg/data/chk_on.wav"))
        print("idle %.1f dBFS/Hz  all84 %.1f  delta %+.1f dB" % (a, b, b-a))
    finally:
        us.stop_static(l); l.close()
main()

