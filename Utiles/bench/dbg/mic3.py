import subprocess, time, threading, sys
sys.path.insert(0, "Utiles/bench")
import numpy as np
import umh_static as us
from umh_link import UmhLink
FF = r"d:\Software\ffmpeg-6.0-essentials_build\bin\ffmpeg.exe"
MIC = "audio=Microphone (Realtek High Definition Audio)"
def rec(path, secs):
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-f", "dshow", "-i", MIC, "-t", str(secs), "-ac", "1", "-ar", "48000", "-y", path], check=True)

RATE = 20000
FREQ = 1000.0
DEPTH = 0
stop_flag = False
def feeder():
    n = 0
    seq = 0
    chunk = 128
    while not stop_flag:
        idx = np.arange(n, n+chunk)
        w = 128.0 + DEPTH*np.sin(2*np.pi*FREQ*idx/RATE)
        l.audio_data(bytes(np.clip(w, 0, 255).astype(np.uint8)), seq & 0xFFFFFFFF)
        n += chunk
        seq += 1
        time.sleep(chunk/float(RATE))

l = UmhLink("COM4")
try:
    us.stop_static(l); time.sleep(0.5)
    rec("Utiles/bench/dbg/data/m3_idle.wav", 4)
    l.audio_configure([(0,0,100000,255,0)], envelope_rate_hz=RATE, prebuffer=512, envelope_level=255)
    l.audio_start(); time.sleep(0.4)

    th = threading.Thread(target=feeder, daemon=True); th.start()
    rec("Utiles/bench/dbg/data/m3_const.wav", 5)
    print("status", l.get_audio_status())
    stop_flag = True; th.join(timeout=1.0)
    l.audio_stop(); time.sleep(0.4)
    rec("Utiles/bench/dbg/data/m3_post.wav", 3)
finally:
    stop_flag = True
    try:
        l.audio_stop()
    except Exception:
        pass
    us.stop_static(l); l.close()


