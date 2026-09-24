import sys
import time
import numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
import umh_static as us
from umh_link import UmhLink

def harmonic_stats(v, xinc):
    ac = v - v.mean()
    w = np.hanning(len(ac))
    sp = np.fft.rfft(ac * w)
    f = np.fft.rfftfreq(len(ac), d=xinc)
    amp = np.abs(sp) * 2.0 / w.sum()
    vals = []
    for f0 in (40000.0, 80000.0, 120000.0, 160000.0):
        if f0 >= f[-1]:
            vals.append(0.0); continue
        idx = int(np.argmin(np.abs(f - f0)))
        vals.append(float(amp[idx]))
    thd = float(np.sqrt(np.sum(np.square(vals[1:]))) / vals[0]) if vals[0] > 0 else 0.0
    return vals, thd

def main():
    s = sc.open_scope()
    l = UmhLink("COM4")
    try:
        us.stop_static(l)
        s.write(":TIMebase:MAIN:SCALe 5e-5")
        for lev in (8, 16, 32, 64, 96, 128):
            s.write(":TRIGger:SWEep AUTO"); s.write(":RUN"); time.sleep(0.05)
            us.send_dense(l, [lev] * 84, [0] * 84)
            time.sleep(0.12)
            d = sc.capture(s, "MATH", 20000)
            v = d["v"]
            hm, thd = harmonic_stats(v, d["xinc"])
            print("lev", lev, "mean", round(float(v.mean()),2), "pp", round(float(v.max()-v.min()),2), "rms", round(float(v.std()),2), "h1", round(hm[0],2), "thd", round(thd,3))
            us.stop_static(l)
    finally:
        us.stop_static(l)
        l.close()
        s.close()

if __name__ == "__main__":
    main()
