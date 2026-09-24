import time
import numpy as np

def rd(s, src="MATH", n=1200, mode="NORMal", tries=8):
    s.write(":WAVeform:SOURce " + src)
    s.write(":WAVeform:MODE " + mode)
    s.write(":WAVeform:FORMat BYTE")
    s.write(":TRIGger:SWEep AUTO")
    s.write(":RUN"); time.sleep(0.5); s.write(":STOP"); time.sleep(0.2)
    for t in range(tries):
        s.write(":RUN")
        time.sleep(0.25)
        s.write(":STOP")
        time.sleep(0.15)
        s.write(":WAVeform:POINts %d" % n)

        pre = s.query(":WAVeform:PREamble?").strip().split(",")
        if len(pre) != 10:
            continue
        fmt, typ, npts, cnt = [int(float(x)) for x in pre[:4]]
        if npts < 16:
            continue
        xinc, xorig, xref, yinc, yorig, yref = [float(x) for x in pre[4:]]
        raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", is_big_endian=False, container=np.ndarray)
        if len(raw) != npts:
            continue

        v = (raw - yref) * yinc + yorig
        t = xorig + (np.arange(len(v)) - xref) * xinc
        return dict(t=t, v=v, xinc=xinc, yinc=yinc, n=npts)
    raise RuntimeError("scope read failed")


def spec(v, xinc):
    ac = v - v.mean()
    w = np.hanning(len(ac))
    sp = np.abs(np.fft.rfft(ac*w)) * 2.0 / w.sum()
    return np.fft.rfftfreq(len(ac), d=xinc), sp

