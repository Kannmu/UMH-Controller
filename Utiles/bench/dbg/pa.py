import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")

def run(tag, lv):
    us.send_dense(l, lv, [0]*84)
    time.sleep(0.3)
    st = l.get_fpga_status()
    flags = int.from_bytes(st[6:8], "little")
    s.write(":RUN"); time.sleep(0.1)
    d = rd(s, "MATH", 1200)
    v = d["v"]
    f, sp = spec(v, d["xinc"])
    i40 = int(np.argmin(np.abs(f-40000)))
    print(tag, "pp=%.1f a40=%.2f mean=%.2f run=%d" % (np.percentile(v,99)-np.percentile(v,1), sp[i40], v.mean(), (flags>>4)&1))

def mk(idxs, lev):
    return [lev if i in idxs else 0 for i in range(84)]
try:
    run("all0", [0]*84)
    run("all64", [64]*84)
    run("first42", mk(range(42), 64))

    run("ch0", mk([0], 64))
    run("ch0-1", mk([0,1], 64))
    run("ch0-7", mk(range(8), 64))
    run("ch0-15", mk(range(16), 64))
finally:
    us.stop_static(l); l.close(); s.close()

