import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
from sutil import rd
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 5e-3")
s.write(":MATH:SCALe 10")
def records(n, src="MATH"):
    out = []
    for i in range(n):
        out.append(rd(s, src, 1200)["v"])
    return out

def report(tag, recs):
    rs = []
    acc = None
    for v in recs:
        e = np.abs(v - v.mean())
        e = e / max(e.mean(), 1e-9) - 1.0
        rs.append(e.std())
        w = np.hanning(len(e))
        X = np.abs(np.fft.rfft(e*w)) * 2.0 / w.sum()
        acc = X**2 if acc is None else acc + X**2
        f = np.fft.rfftfreq(len(e), d=5e-5)
    Xa = np.sqrt(acc/len(recs))
    print("==", tag, "am_rms=%.3f%%" % (100*np.mean(rs)))
    idx = np.argsort(Xa*(f>20))[::-1][:6]
    for i in idx:
        print("   %8.1f Hz amp=%.4f%%" % (f[i], 100*Xa[i]))
    return Xa, f

def report(tag, recs):
    rs = []
    acc = None
    for v in recs:
        e = np.abs(v - v.mean())
        e = e / max(e.mean(), 1e-9) - 1.0
        rs.append(e.std())

        w = np.hanning(len(e))
        X = np.abs(np.fft.rfft(e*w)) * 2.0 / w.sum()
        acc = X**2 if acc is None else acc + X**2
        f = np.fft.rfftfreq(len(e), d=5e-5)

    Xa = np.sqrt(acc/len(recs))
    print("==", tag, "am_rms=%.3f%%" % (100*np.mean(rs)))
    idx = np.argsort(Xa*(f>20))[::-1][:6]
    for i in idx:
        print("   %8.1f Hz amp=%.4f%%" % (f[i], 100*Xa[i]))

def state(tag):
    v = np.load("Utiles/bench/dbg/data/dummy.npz")["v"] if False else None
try:
    us.stop_static(l); time.sleep(0.3)
    report("idle1", records(8, "MATH"))
    lv = [0]*84; lv[75] = 128
    us.send_dense(l, lv, [0]*84); time.sleep(0.4)
    report("ch75_128", records(8))

    us.send_dense(l, [128]*84, [0]*84); time.sleep(0.4)
    report("all_128", records(3))
    us.stop_static(l); time.sleep(0.4)
    report("idle2", records(6))
finally:
    us.stop_static(l); l.close(); s.close()

