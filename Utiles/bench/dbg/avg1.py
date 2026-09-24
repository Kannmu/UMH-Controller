import sys, time, threading, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")
s.write(":WAVeform:MODE NORMal")
s.write(":WAVeform:FORMat BYTE")
s.write(":TRIGger:SWEep AUTO")
s.write(":TRIGger:EDGE:SOURce CHANnel1")
s.write(":TRIGger:EDGE:LEVel 1.0")
s.write(":ACQuire:TYPE AVERages")
s.write(":ACQuire:AVERages 256")

def grab(src, tag, extra_wait=3.0):
    s.write(":WAVeform:SOURce " + src)
    s.write(":RUN"); time.sleep(extra_wait); s.write(":STOP"); time.sleep(0.3)
    s.write(":WAVeform:POINts 1200")
    pre = s.query(":WAVeform:PREamble?").strip().split(",")
    raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", container=np.ndarray)
    xinc, xorig, xref, yinc, yorig, yref = [float(q) for q in pre[4:10]]
    v = (np.asarray(raw, dtype=float) - yref) * yinc + yorig
    np.savez("Utiles/bench/dbg/data/%s.npz" % tag, v=v, xinc=xinc)
    print(tag, "n", len(v), "mean %.2f pp %.2f" % (v.mean(), v.max()-v.min()))

def grab(src, tag, extra_wait=3.0):
    s.write(":WAVeform:SOURce " + src)
    s.write(":RUN"); time.sleep(extra_wait); s.write(":STOP"); time.sleep(0.3)
    s.write(":WAVeform:POINts 1200")
    pre = s.query(":WAVeform:PREamble?").strip().split(",")
    raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", container=np.ndarray)

    xinc, xorig, xref, yinc, yorig, yref = [float(q) for q in pre[4:10]]
    v = (np.asarray(raw, dtype=float) - yref) * yinc + yorig
    np.savez("Utiles/bench/dbg/data/%s.npz" % tag, v=v, xinc=xinc)
    print(tag, "n", len(v), "mean %.2f pp %.2f" % (v.mean(), v.max()-v.min()))

stop_flag = False
def feeder():
    seq = 0
    data = bytes([255])*64
    while not stop_flag:
        l.audio_data(data, seq & 0xFFFFFFFF)
        seq += 1
        time.sleep(0.0032)

try:
    us.stop_static(l); time.sleep(0.3)
    lv = [0]*84; lv[75] = 128
    us.send_dense(l, lv, [0]*84); time.sleep(0.4)
    grab("CHAN2", "avg_static_ch2", 2.0)
    l.audio_configure([(0,0,100000,255,0)], envelope_rate_hz=20000, prebuffer=512, envelope_level=255)

    l.audio_start(); time.sleep(0.3)
    th = threading.Thread(target=feeder, daemon=True); th.start()
    time.sleep(0.5)
    grab("CHAN2", "avg_am_ch2", 2.0)
    stop_flag = True; th.join(timeout=1.0)

    l.audio_stop()
finally:
    stop_flag = True
    try:
        l.audio_stop()
    except Exception:
        pass
    us.stop_static(l); l.close(); s.close()

