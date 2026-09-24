import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")

def go(tag, lv):
    us.send_dense(l, lv, [0]*84)
    time.sleep(0.25)
    s.write(":RUN"); time.sleep(0.08)
    d = rd(s, "MATH", 1200)
    v = d["v"]
    f, sp = spec(v, d["xinc"])
    i40 = int(np.argmin(np.abs(f-40000)))
    print(tag, "pp=%.1f a40=%.2f mean=%.2f" % (np.percentile(v,99)-np.percentile(v,1), sp[i40], v.mean()))

def mk(idxs, lev):
    return [lev if i in idxs else 0 for i in range(84)]
try:
    for r in range(2):
        go("all64_%d" % r, [64]*84); time.sleep(0.4)
        go("first42_%d" % r, mk(range(42),64)); time.sleep(0.4)
        go("all32_%d" % r, [32]*84); time.sleep(0.4)
finally:
    us.stop_static(l); l.close(); s.close()

