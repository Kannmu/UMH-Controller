import subprocess, time, sys
sys.path.insert(0, "Utiles/bench")
import numpy as np
import umh_static as us
from umh_link import UmhLink
FF = r"d:\Software\ffmpeg-6.0-essentials_build\bin\ffmpeg.exe"
MIC = "audio=Microphone (Realtek High Definition Audio)"

def rec(path, secs):
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-f", "dshow", "-i", MIC, "-t", str(secs), "-ac", "1", "-ar", "48000", "-y", path], check=True)
l = UmhLink("COM4")
try:
    us.stop_static(l); time.sleep(0.5)
    rec("Utiles/bench/dbg/data/mic_off.wav", 4)

    lv = [0]*84; lv[75] = 128
    us.send_dense(l, lv, [0]*84); time.sleep(0.5)
    rec("Utiles/bench/dbg/data/mic_on75.wav", 4)
    us.stop_static(l); time.sleep(0.5)
    rec("Utiles/bench/dbg/data/mic_off2.wav", 4)
finally:
    us.stop_static(l); l.close()

