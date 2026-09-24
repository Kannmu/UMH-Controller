import sys
import time
import numpy as np
sys.path.insert(0, "Utiles/bench")
from umh_link import UmhLink
import umh_static as us
import scope_umh as sc

def band_stats(v, xinc):
    ac = v - v.mean()
    w = np.hanning(len(ac))
    sp = np.fft.rfft(ac * w)
    f = np.fft.rfftfreq(len(ac), d=xinc)
    band = (f > 30000) & (f < 50000)
    idx = np.argmax(np.abs(sp[band])) if band.any() else 0
    fp = f[band][idx] if band.any() else 0.0
    ap = np.abs(sp[band]).max() * 2.0 / w.sum() if band.any() else 0.0
    return v.mean(), v.max() - v.min(), ac.std(), fp, ap

def main():
    s = sc.open_scope()
    l = UmhLink("COM4")
    try:
        s.write(":TRIGger:SWEep AUTO")
        us.stop_static(l)
        for k in range(84):
            s.write(":RUN")
            us.send_static(l, k, 128)
            time.sleep(0.06)
            d = sc.capture(s, "MATH", 8000)
            m, pp, rms, fp, ap = band_stats(d["v"], d["xinc"])
            print("%02d mean=%8.2f pp=%8.1f rms=%7.2f f=%8.0f a=%8.2f" % (k, m, pp, rms, fp, ap))
            us.stop_static(l)
    finally:
        us.stop_static(l)
        l.close()
        s.close()

if __name__ == "__main__":
    main()
