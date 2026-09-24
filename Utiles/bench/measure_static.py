import sys
import time
import numpy as np
sys.path.insert(0, "Utiles/bench")
from umh_link import UmhLink
import umh_static as us
import scope_umh as sc

def measure(channel, level, phase=0, settle=0.25, points=12000, sparse=False, dense=False):
    s = sc.open_scope()
    l = UmhLink("COM4")
    try:
        s.write(":TRIGger:SWEep AUTO")
        s.write(":RUN")
        (us.send_dense(l, [level if i == channel else 0 for i in range(84)], [phase]*84) if dense else (us.send_sparse if sparse else us.send_static)(l, channel, level, phase))
        time.sleep(settle)
        d = sc.capture(s, "MATH", points)
        v = d["v"]
        ac = v - v.mean()
        w = np.hanning(len(ac))
        sp = np.fft.rfft(ac * w)
        f = np.fft.rfftfreq(len(ac), d=d["xinc"])
        band = (f > 30000) & (f < 50000)
        pk = f[band][np.argmax(np.abs(sp[band]))] if band.any() else 0
        amp = np.abs(sp[band]).max() * 2.0 / w.sum() if band.any() else 0
        return dict(mean=v.mean(), pp=v.max()-v.min(), ac_rms=ac.std(), f_peak=pk, a_peak=amp, t=d["t"], v=v)
    finally:
        us.stop_static(l)
        l.close()
        s.close()
