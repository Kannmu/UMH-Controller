import sys, time
sys.path.insert(0, "Utiles/bench")
import numpy as np
import scope_umh as sc
import umh_static as us
from umh_link import UmhLink


def band(v, xinc):
    ac = v - v.mean()
    w = np.hanning(len(ac))
    sp = np.abs(np.fft.rfft(ac*w))
    f = np.fft.rfftfreq(len(ac), d=xinc)
    m = np.logical_and(f>30e3, f<50e3)
    i = np.argmax(sp[m]) if m.any() else 0
    return (round(float(v.mean()),2), round(float(v.max()-v.min()),2), round(float(ac.std()),3), round(float(f[m][i]),1), round(float(sp[m].max()*2/w.sum()),2))


s = sc.open_scope()
l = UmhLink("COM4")
hits = []
try:
    s.write(":TRIGger:SWEep AUTO")
    s.write(":RUN")
    us.stop_static(l)
    time.sleep(0.3)
    d = sc.capture(s, "MATH", 4000)
    print("OFF yinc=%.4f" % d["yinc"], band(d["v"], d["xinc"]))

    for k in range(84):
        s.write(":RUN")
        us.send_static(l, k, 16)
        time.sleep(0.12)
        d = sc.capture(s, "MATH", 4000)
        r = band(d["v"], d["xinc"])
        us.stop_static(l)
        if r[1] > 2.0:
            print("ch%02d" % k, r)
            hits.append((k, r))
    print("scan done", hits)

finally:
    us.stop_static(l)
    l.close()
    s.close()

