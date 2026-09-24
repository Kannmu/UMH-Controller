import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")
s.write(":MATH:SCALe 10")

def harm(v, xinc):
    ac = v - v.mean()
    w = np.hanning(len(ac))
    sp = np.abs(np.fft.rfft(ac*w))*2.0/w.sum()
    f = np.fft.rfftfreq(len(ac), d=xinc)
    amps = []
    for f0 in (40000., 80000., 120000., 160000., 200000.):
        i = int(np.argmin(np.abs(f-f0)))
        amps.append(round(float(sp[i]),2))
    return amps

def go(lev):
    lv = [0]*84
    lv[75] = lev
    us.send_dense(l, lv, [0]*84)
    time.sleep(0.4)
    s.write(":RUN"); time.sleep(0.12)
    d = rd(s, "MATH", 1200)
    v = d["v"]
    amps = harm(v, d["xinc"])
    print("lev=%3d min=%7.2f max=%7.2f mean=%7.2f pp=%5.1f h=%s" % (lev, v.min(), v.max(), v.mean(), np.percentile(v,99)-np.percentile(v,1), amps))
    np.savez("Utiles/bench/dbg/data/ch75_lev%d.npz" % lev, t=d["t"], v=v, xinc=d["xinc"])

try:
    us.stop_static(l); time.sleep(0.3)
    for lev in (0, 8, 16, 32, 64, 96, 128, 160, 192, 255):
        go(lev)
finally:
    us.stop_static(l); l.close(); s.close()

