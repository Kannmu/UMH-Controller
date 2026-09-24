import sys, time, threading, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd, spec
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-5")
stop_flag = False

def feeder(rate=20000, chunk=256, level=255):
    seq = 0
    n = 0
    t0 = time.monotonic()
    data = bytes([level])*chunk
    while not stop_flag:
        l.audio_data(data, seq & 0xFFFFFFFF)
        seq += 1
        n += chunk
        dt = t0 + n/float(rate) - time.monotonic()
        if dt > 0:
            time.sleep(dt)

def cap12(tag):
    s.write(":RUN"); time.sleep(0.12); d1 = rd(s, "CHAN1", 1200)
    s.write(":RUN"); time.sleep(0.12); d2 = rd(s, "CHAN2", 1200)
    vd = d1["v"] - d2["v"]
    np.savez("Utiles/bench/dbg/data/%s.npz" % tag, t=d1["t"], v1=d1["v"], v2=d2["v"], xinc=d1["xinc"])
    ac = vd - vd.mean()
    w = np.hanning(len(ac))
    sp = np.abs(np.fft.rfft(ac*w))*2.0/w.sum()
    f = np.fft.rfftfreq(len(ac), d=d1["xinc"])
    h = []
    for f0 in (40000., 80000., 120000., 160000.):
        i = int(np.argmin(np.abs(f-f0))); h.append(round(float(sp[i]),2))
    print(tag, "mean=%.2f pp=%.1f h=%s" % (vd.mean(), np.percentile(vd,99)-np.percentile(vd,1), h))
    return vd

def cap12(tag):
    s.write(":RUN"); time.sleep(0.12); d1 = rd(s, "CHAN1", 1200)
    s.write(":RUN"); time.sleep(0.12); d2 = rd(s, "CHAN2", 1200)
    vd = d1["v"] - d2["v"]
    np.savez("Utiles/bench/dbg/data/%s.npz" % tag, t=d1["t"], v1=d1["v"], v2=d2["v"], xinc=d1["xinc"])

    ac = vd - vd.mean()
    w = np.hanning(len(ac))
    sp = np.abs(np.fft.rfft(ac*w))*2.0/w.sum()
    f = np.fft.rfftfreq(len(ac), d=d1["xinc"])
    h = []
    for f0 in (40000., 80000., 120000., 160000.):
        i = int(np.argmin(np.abs(f-f0))); h.append(round(float(sp[i]),2))
    print(tag, "mean=%.2f pp=%.1f h=%s" % (vd.mean(), np.percentile(vd,99)-np.percentile(vd,1), h))

th = None
try:
    us.stop_static(l); time.sleep(0.3)
    cap12("am_off")
    l.audio_configure([(0, 0, 100000, 255, 0)], envelope_rate_hz=20000, prebuffer=512, envelope_level=255)
    l.audio_start(); time.sleep(0.3)

    th = threading.Thread(target=feeder, args=(20000, 256, 255), daemon=True)
    th.start(); time.sleep(0.8)
    cap12("am_const255")
    print("status", l.get_audio_status())
    stop_flag = True; th.join(timeout=1.0)
    l.audio_stop()
finally:
    stop_flag = True
    try:
        l.audio_stop()
    except Exception:
        pass
    try:
        us.stop_static(l)
    except Exception:
        pass
    l.close(); s.close()

