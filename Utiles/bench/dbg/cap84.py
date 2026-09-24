import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")

def grab(src, tag):
    s.write(":WAVeform:SOURce " + src)
    s.write(":RUN"); time.sleep(0.3); s.write(":STOP"); time.sleep(0.15)
    s.write(":WAVeform:POINts 1200")
    pre = s.query(":WAVeform:PREamble?").strip().split(",")
    raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", container=np.ndarray)

    xinc, xorig, xref, yinc, yorig, yref = [float(q) for q in pre[4:10]]
    v = (np.asarray(raw, dtype=float) - yref) * yinc + yorig
    np.savez("Utiles/bench/dbg/data/%s.npz" % tag, v=v, xinc=xinc)
    print(tag, len(v), "pp %.1f" % (v.max()-v.min()))

try:
    us.stop_static(l); time.sleep(0.3)
    us.send_dense(l, [128]*84, [0]*84); time.sleep(0.5)
    grab("CHAN2", "d84_128_ch2")
    grab("CHAN1", "d84_128_ch1")
finally:
    us.stop_static(l); l.close(); s.close()

