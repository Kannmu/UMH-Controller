import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")

def take(tag):
    s.write(":RUN"); time.sleep(0.35)
    d = rd(s, "MATH", 1200)
    v = d["v"]
    f, sp = spec(v, d["xinc"])
    i40 = int(np.argmin(np.abs(f-40000)))
    lo, hi = np.percentile(v, [1, 99])
    print("%s pp=%.2f pp199=%.2f mean=%.2f a40=%.2f" % (tag, v.max()-v.min(), hi-lo, v.mean(), sp[i40]))

try:
    us.stop_static(l); time.sleep(0.3); take("off")
    for lev in (0, 8, 32, 128):
        us.send_dense(l, [lev]*84, [0]*84); time.sleep(0.3); take("dense_%d" % lev)
finally:
    us.stop_static(l); l.close(); s.close()

