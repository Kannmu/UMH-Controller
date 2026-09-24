import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")
res = []

try:
    for k in range(84):
        us.send_static(l, k, 128)
        time.sleep(0.18)
        s.write(":RUN"); time.sleep(0.05)
        d = rd(s, "MATH", 1200)
        v = d["v"]
        f, sp = spec(v, d["xinc"])
        i40 = int(np.argmin(np.abs(f-40000)))
        pp = float(np.percentile(v,99) - np.percentile(v,1))
        res.append((k, pp, float(sp[i40]), float(v.mean())))
        if pp > 20.0:
            print("HIT", k, "pp=%.1f a40=%.2f mean=%.2f" % (pp, sp[i40], v.mean()))
    print("done")
    for r in res:
        print("%02d pp=%5.1f a40=%6.2f mean=%7.2f" % r)
finally:
    us.stop_static(l); l.close(); s.close()

