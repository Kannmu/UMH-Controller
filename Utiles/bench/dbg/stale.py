import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc, umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
def g(tag):
    s.write(":RUN"); time.sleep(0.25)
    d = sc.capture(s, "MATH", 4000)
    v = d["v"]
    print(tag, round(float(v.min()),2), round(float(v.max()),2), round(float(v.sum()),1))
    return v
try:

    us.stop_static(l); a = g("off1")
    us.send_static(l, 0, 16); b = g("ch00")
    us.stop_static(l); c = g("off2")
    us.send_static(l, 40, 16); e = g("ch40")
    print("b==c:", bool(np.allclose(b,c,atol=1e-9)), "b==e:", bool(np.allclose(b,e,atol=1e-9)))
finally:
    us.stop_static(l); l.close(); s.close()

