import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-3")
s.write(":MATH:SCALe 10")

def grab(src, tag):
    s.write(":RUN"); time.sleep(0.4)
    d = rd(s, src, 1200)
    v = d["v"]
    np.savez("Utiles/bench/dbg/data/%s.npz" % tag, t=d["t"], v=v, xinc=d["xinc"])
    ac = v - v.mean()
    w = np.hanning(len(ac))
    sp = np.abs(np.fft.rfft(ac*w))*2.0/w.sum()
    f = np.fft.rfftfreq(len(ac), d=d["xinc"])
    idx = np.argsort(sp)[::-1][:8]
    print(tag, "xinc=%.3g" % d["xinc"])
    for i in idx:
        print("   %9.1f Hz %8.2f" % (f[i], sp[i]))

try:
    us.stop_static(l); time.sleep(0.3)
    lv = [0]*84; lv[75] = 128
    us.send_dense(l, lv, [0]*84); time.sleep(0.4)
    grab("CHAN1", "slow_ch1_128")
    grab("CHAN2", "slow_ch2_128")
    grab("MATH", "slow_math_128")
    us.stop_static(l)
finally:
    us.stop_static(l); l.close(); s.close()

