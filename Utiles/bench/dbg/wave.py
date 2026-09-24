import sys, time, numpy as np, os
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd
import umh_static as us
from umh_link import UmhLink
os.makedirs("Utiles/bench/dbg/data", exist_ok=True)
s = sc.open_scope(); l = UmhLink("COM4")

def grab(tag, source="MATH"):
    s.write(":RUN"); time.sleep(0.4)
    d = rd(s, source, 1200)
    np.savez("Utiles/bench/dbg/data/%s.npz" % tag, t=d["t"], v=d["v"], xinc=d["xinc"])
    print(tag, "pp %.2f mean %.2f" % (d["v"].max()-d["v"].min(), d["v"].mean()))
def dense(level):
    us.send_dense(l, [level]*84, [0]*84)

try:
    s.write(":TIMebase:MAIN:SCALe 5e-5")
    us.stop_static(l); time.sleep(0.3); grab("off_slow")
    us.send_static(l,0,16); time.sleep(0.3); grab("ch0_16_slow")
    us.send_static(l,0,128); time.sleep(0.3); grab("ch0_128_slow")
    dense(16); time.sleep(0.3); grab("all_16_slow")

    us.stop_static(l); time.sleep(0.2)
    s.write(":TIMebase:MAIN:SCALe 2e-6")
    grab("off_fast")
    us.send_static(l,0,128); time.sleep(0.3); grab("ch0_128_fast")
    dense(16); time.sleep(0.3); grab("all_16_fast")
finally:
    us.stop_static(l); l.close(); s.close()

