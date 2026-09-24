import sys
import time
import numpy as np
sys.path.insert(0, "Utiles/bench")
from umh_link import UmhLink
import scope_umh as sc

def measure(level, rate=20000, prebuffer=512, packets=4, settle=0.05, points=12000):
    s = sc.open_scope()
    l = UmhLink("COM4")
    try:
        s.write(":TRIGger:SWEep AUTO")
        s.write(":RUN")
        l.audio_configure([(0, 0, 1000000, 0, 255)], rate, prebuffer, 255)
        l.audio_start()
        seq = 0
        for i in range(packets):
            l.audio_data(bytes([level]) * 512, seq)
            seq += 1
            time.sleep(0.005)
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
        st = l.get_audio_status()
        return dict(mean=v.mean(), pp=v.max()-v.min(), ac_rms=ac.std(), f_peak=pk, a_peak=amp, underrun=st.underrun_count, overrun=st.overrun_count, loss=st.packet_loss_count, rendered=st.rendered_samples, ppm=st.clock_correction_ppm, t=d["t"], v=v)
    finally:
        try:
            l.audio_stop()
        except Exception:
            pass
        l.close()
        s.close()
