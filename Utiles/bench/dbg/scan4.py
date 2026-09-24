import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")

def one(k, lev=128):
    lv = [0]*84
    lv[k] = lev
    us.send_dense(l, lv, [0]*84)
    time.sleep(0.18)
    s.write(":RUN"); time.sleep(0.08)
    d = rd(s, "MATH", 1200)
    v = d["v"]
    f, sp = spec(v, d["xinc"])
    i40 = int(np.argmin(np.abs(f-40000)))
    return float(np.percentile(v,99)-np.percentile(v,1)), float(sp[i40]), float(v.mean())

try:
    us.stop_static(l); time.sleep(0.3)
    for k in range(42,84):
        r = one(k)
        if r[0] > 15:
            print("HIT %02d pp=%.1f a40=%.2f mean=%.2f" % ((k,)+r))
    print("done0-41")
finally:
    us.stop_static(l); l.close(); s.close()


