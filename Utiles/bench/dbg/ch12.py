import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")
s.write(":MATH:SCALe 10")

def go(lev):
    lv = [0]*84
    lv[75] = lev
    us.send_dense(l, lv, [0]*84)
    time.sleep(0.4)
    for src in ("CHAN1", "CHAN2", "MATH"):
        s.write(":RUN"); time.sleep(0.12)
        d = rd(s, src, 1200)
        v = d["v"]
        print("lev=%3d %-6s min=%7.2f max=%7.2f mean=%6.2f pp=%6.1f" % (lev, src, v.min(), v.max(), v.mean(), np.percentile(v,99)-np.percentile(v,1)))

try:
    us.stop_static(l); time.sleep(0.3)
    for lev in (8, 32, 128, 192):
        go(lev)
finally:
    us.stop_static(l); l.close(); s.close()

