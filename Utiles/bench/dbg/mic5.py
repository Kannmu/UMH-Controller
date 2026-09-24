import subprocess, time, sys
sys.path.insert(0, "Utiles/bench")
import umh_static as us
from umh_link import UmhLink
FF = r"d:\Software\ffmpeg-6.0-essentials_build\bin\ffmpeg.exe"
MIC = "audio=Microphone (Realtek High Definition Audio)"
def rec(path, secs, rate):
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-f", "dshow", "-i", MIC, "-t", str(secs), "-ac", "1", "-ar", str(rate), "-y", path], check=True)

l = UmhLink("COM4")
try:
    us.stop_static(l); time.sleep(0.4)
    rec("Utiles/bench/dbg/data/m5_off44.wav", 3, 44100)
    us.send_dense(l, [128]*84, [0]*84); time.sleep(0.4)
    rec("Utiles/bench/dbg/data/m5_on44.wav", 4, 44100)
    rec("Utiles/bench/dbg/data/m5_on96.wav", 4, 96000)
    rec("Utiles/bench/dbg/data/m5_off96.wav", 3, 96000)
finally:
    us.stop_static(l); l.close()

