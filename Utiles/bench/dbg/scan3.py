import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")

def one(k, lev=64):
    lv = [0]*84
    lv[k] = lev
    us.send_dense(l, lv, [0]*84)
    time.sleep(0.15)
    s.write(":RUN"); time.sleep(0.05)
    d = rd(s, "MATH", 1200)
    v = d["v"]
    f, sp = spec(v, d["xinc"])
    i40 = int(np.argmin(np.abs(f-40000)))
    pp = float(np.percentile(v,99)-np.percentile(v,1))
    return pp, float(sp[i40]), float(v.mean())

try:
    us.send_dense(l, [0]*84, [0]*84); time.sleep(0.3)
    s.write(":RUN"); time.sleep(0.1)
    d = rd(s, "MATH", 1200); print("zero std %.2f" % float(d["v"].std()))
    res = []
    for k in range(84):
        r = one(k)
        res.append((k,)+r)
        if r[0] > 20:
            print("HIT %02d pp=%.1f a40=%.2f mean=%.2f" % ((k,)+r))

    print("SUMMARY")
    for r in res:
        print("%02d %5.1f %6.2f %7.2f" % r)
finally:
    us.stop_static(l); l.close(); s.close()

