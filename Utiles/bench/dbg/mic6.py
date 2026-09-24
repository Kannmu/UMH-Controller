import subprocess, time, sys
sys.path.insert(0, "Utiles/bench")
import umh_static as us
from umh_link import UmhLink
FF = r"d:\Software\ffmpeg-6.0-essentials_build\bin\ffmpeg.exe"
MIC = "audio=IP Camera Bridge Audio"
def rec(path, secs):
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-f", "dshow", "-i", MIC, "-t", str(secs), "-ac", "1", "-ar", "48000", "-y", path], check=True)

l = UmhLink("COM4")
try:
    us.stop_static(l); time.sleep(0.4)
    rec("Utiles/bench/dbg/data/cam_off2.wav", 3)
    us.send_dense(l, [128]*84, [0]*84); time.sleep(0.4)
    rec("Utiles/bench/dbg/data/cam_on.wav", 4)
    us.stop_static(l); time.sleep(0.4)
    rec("Utiles/bench/dbg/data/cam_off3.wav", 3)
finally:
    us.stop_static(l); l.close()

