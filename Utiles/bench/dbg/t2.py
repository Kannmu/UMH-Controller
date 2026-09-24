import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")

def take(tag):
    s.write(":RUN"); time.sleep(0.4)
    d = rd(s, "MATH", 1200)
    v = d["v"]
    f, sp = spec(v, d["xinc"])
    i = np.argmax(sp[1:]) + 1
    print(tag, "n", d["n"], "pp %.2f mean %.2f fPk %.0f aPk %.2f" % (v.max()-v.min(), v.mean(), f[i], sp[i]))
    return d
print("srate", s.query(":ACQuire:SRATe?").strip(), "mdep", s.query(":ACQuire:MDEPth?").strip())

try:
    us.stop_static(l); time.sleep(0.3); take("off")
    us.send_static(l, 0, 16); time.sleep(0.3); take("ch0_16")
    us.send_static(l, 0, 128); time.sleep(0.3); take("ch0_128")
finally:
    us.stop_static(l); l.close(); s.close()

