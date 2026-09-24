import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")

def go(lev):
    lv = [0]*84
    lv[75] = lev
    us.send_dense(l, lv, [0]*84)
    time.sleep(0.2)
    s.write(":RUN"); time.sleep(0.1)
    d = rd(s, "MATH", 1200)
    v = d["v"]
    f, sp = spec(v, d["xinc"])
    i40 = int(np.argmin(np.abs(f-40000)))
    exp = (2*lev/256.0-1)*12
    print("lev=%3d pp=%5.1f a40=%6.2f mean=%7.2f exp_mean=%6.2f" % (lev, np.percentile(v,99)-np.percentile(v,1), sp[i40], v.mean(), exp))

try:
    us.stop_static(l); time.sleep(0.3)
    for lev in (0, 8, 16, 32, 64, 96, 128, 192, 255):
        go(lev)
finally:
    us.stop_static(l); l.close(); s.close()

