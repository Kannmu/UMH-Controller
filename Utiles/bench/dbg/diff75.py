import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")
s.write(":TRIGger:EDGE:SOURce CHANnel1")
s.write(":TRIGger:EDGE:LEVel 1.0")
s.write(":TRIGger:EDGE:SLOPe POSitive")
print("trig src", s.query(":TRIGger:EDGE:SOURce?").strip())

def harm(v, xinc):
    ac = v - v.mean()
    w = np.hanning(len(ac))
    sp = np.abs(np.fft.rfft(ac*w))*2.0/w.sum()
    f = np.fft.rfftfreq(len(ac), d=xinc)
    out = []
    for f0 in (40000., 80000., 120000., 160000.):
        i = int(np.argmin(np.abs(f-f0)))
        out.append(round(float(sp[i]),2))
    return out

def go(lev, settle=0.4):
    lv = [0]*84
    lv[75] = lev
    us.send_dense(l, lv, [0]*84)
    time.sleep(settle)
    s.write(":RUN"); time.sleep(0.12); d1 = rd(s, "CHAN1", 1200)
    s.write(":RUN"); time.sleep(0.12); d2 = rd(s, "CHAN2", 1200)

    v1 = d1["v"]; v2 = d2["v"]; vd = v1 - v2
    np.savez("Utiles/bench/dbg/data/d75_%d.npz" % lev, t=d1["t"], v1=v1, v2=v2, xinc=d1["xinc"])
    print("lev=%3d m1=%6.2f m2=%6.2f md=%6.2f pp=%5.1f h=%s" % (lev, v1.mean(), v2.mean(), vd.mean(), np.percentile(vd,99)-np.percentile(vd,1), harm(vd, d1["xinc"])))

try:
    us.stop_static(l); time.sleep(0.3)
    for lev in (8, 16, 32, 64, 96, 128, 160, 192, 255):
        go(lev)
finally:
    us.stop_static(l); l.close(); s.close()

