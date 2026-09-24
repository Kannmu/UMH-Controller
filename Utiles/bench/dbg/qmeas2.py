import sys, time, threading, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-3")
stop_flag = False

def feeder():
    seq = 0
    hi = True
    chunk = 30
    while not stop_flag:
        lev = 128 if hi else 0
        hi = not hi
        l.audio_data(bytes([lev])*chunk, seq & 0xFFFFFFFF)
        seq += 1
        time.sleep(0.0015)

def grab(tag):
    s.write(":WAVeform:SOURce CHAN2")
    s.write(":RUN"); time.sleep(0.6); s.write(":STOP"); time.sleep(0.2)
    s.write(":WAVeform:POINts 1200")
    pre = s.query(":WAVeform:PREamble?").strip().split(",")

    raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", container=np.ndarray)
    xinc, xorig, xref, yinc, yorig, yref = [float(q) for q in pre[4:10]]
    v = (np.asarray(raw, float) - yref) * yinc + yorig
    np.savez("Utiles/bench/dbg/data/%s.npz" % tag, v=v, xinc=xinc)
    print(tag, len(v), "pp %.1f" % (v.max()-v.min()))

try:
    us.stop_static(l); time.sleep(0.3)
    l.audio_configure([(0,0,100000,255,0)], envelope_rate_hz=20000, prebuffer=512, envelope_level=255)
    l.audio_start(); time.sleep(0.3)
    th = threading.Thread(target=feeder, daemon=True); th.start()
    time.sleep(0.6)
    grab("q2_a")

    s.write(":TIMebase:MAIN:SCALe 5e-4")
    grab("q2_b")
    stop_flag = True; th.join(timeout=1.0)
    l.audio_stop()
finally:
    stop_flag = True
    try:
        l.audio_stop()
    except Exception:
        pass
    us.stop_static(l); l.close(); s.close()


