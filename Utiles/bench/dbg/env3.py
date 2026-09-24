import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-3")
s.write(":WAVeform:MODE NORMal")
s.write(":WAVeform:FORMat BYTE")
s.write(":TRIGger:SWEep AUTO")

def grab(src="MATH"):
    s.write(":WAVeform:SOURce " + src)
    s.write(":RUN"); time.sleep(0.5); s.write(":STOP"); time.sleep(0.2)
    s.write(":WAVeform:POINts 1200")
    pre = s.query(":WAVeform:PREamble?").strip().split(",")
    raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", container=np.ndarray)
    xinc, xorig, xref, yinc, yorig, yref = [float(q) for q in pre[4:10]]
    v = (np.asarray(raw, dtype=float) - yref) * yinc + yorig
    return v

def report(tag, src="MATH", n=8):
    rs = []
    acc = None
    Xa = None
    for i in range(n):
        v = grab(src)
        e = np.abs(v - v.mean())
        e = e / max(e.mean(), 1e-9) - 1.0
        rs.append(e.std())
    return np.mean(rs)

try:
    us.stop_static(l); time.sleep(0.3)
    print("idle", report("idle", "MATH", 6))
    lv = [0]*84; lv[75] = 128
    us.send_dense(l, lv, [0]*84); time.sleep(0.4)
    print("ch75_128", report("ch75", "MATH", 6))

    us.send_dense(l, [64]*84, [0]*84); time.sleep(0.4)
    print("all64", report("all64", "MATH", 4))
    us.stop_static(l); time.sleep(0.3)
    print("idle2", report("idle2", "MATH", 6))
finally:
    us.stop_static(l); l.close(); s.close()

