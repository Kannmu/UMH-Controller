import sys, time, numpy as np
from scipy.signal import hilbert
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope(); l = UmhLink("COM4")
s.write(":TIMebase:MAIN:SCALe 1e-3")
s.write(":WAVeform:MODE NORMal")
s.write(":WAVeform:FORMat BYTE")
s.write(":TRIGger:SWEep AUTO")

def grab(src="CHAN2"):
    s.write(":WAVeform:SOURce " + src)
    s.write(":RUN"); time.sleep(0.5); s.write(":STOP"); time.sleep(0.2)
    s.write(":WAVeform:POINts 1200")
    pre = s.query(":WAVeform:PREamble?").strip().split(",")
    raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", container=np.ndarray)
    xinc, xorig, xref, yinc, yorig, yref = [float(q) for q in pre[4:10]]
    v = (np.asarray(raw, dtype=float) - yref) * yinc + yorig
    return v, xinc

def report(tag, n=5):
    rs = []
    acc = None
    for i in range(n):
        v, xinc = grab("CHAN2")
        env = np.abs(hilbert(v - v.mean()))
        e = env / max(env.mean(), 1e-9) - 1.0
        rs.append(e.std())
    return np.mean(rs), xinc

def report2(tag, n=5):
    acc = None
    rs = []
    for i in range(n):
        v, xinc = grab("CHAN2")
        env = np.abs(hilbert(v - v.mean()))
        e = env / max(env.mean(), 1e-9) - 1.0
        rs.append(e.std())

        w = np.hanning(len(e))
        X = np.abs(np.fft.rfft(e*w)) * 2.0 / w.sum()
        acc = X**2 if acc is None else acc + X**2
        f = np.fft.rfftfreq(len(e), d=xinc)

    Xa = np.sqrt(acc/len(rs))
    print("==", tag, "am_rms=%.3f%% xinc=%.2g" % (100*np.mean(rs), xinc))
    idx = np.argsort(Xa*(f>50))[::-1][:8]
    for i in idx:
        print("   %8.1f Hz  %.4f%%" % (f[i], 100*Xa[i]))

try:
    us.stop_static(l); time.sleep(0.3)
    report2("idle", 4)
    lv = [0]*84; lv[75] = 128
    us.send_dense(l, lv, [0]*84); time.sleep(0.4)
    report2("ch75_128", 4)
    us.stop_static(l); time.sleep(0.3)
    report2("idle2", 4)
finally:
    us.stop_static(l); l.close(); s.close()

