# RIGOL DS1104Z helpers for UMH ultrasound measurements.
import time
import numpy as np
import pyvisa
RES = "USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR"

_RM = None

def open_scope(timeout=5000):
    global _RM
    _RM = pyvisa.ResourceManager()
    s = _RM.open_resource(RES)
    s.timeout = timeout
    return s

def capture(s, src="MATH", points=12000, mode="NORMal"):
    s.write(":STOP")
    s.write(":WAVeform:SOURce " + src)
    s.write(":WAVeform:MODE " + mode)
    s.write(":WAVeform:FORMat BYTE")
    s.write(":WAVeform:POINts %d" % points)
    pre = s.query(":WAVeform:PREamble?").strip().split(",")
    if len(pre) != 10:
        raise RuntimeError("bad preamble " + str(pre))
    fmt, typ, npts, cnt = [int(float(v)) for v in pre[:4]]
    xinc, xorig, xref, yinc, yorig, yref = [float(v) for v in pre[4:]]
    raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", is_big_endian=False, container=np.ndarray)
    raw = np.asarray(raw, dtype=np.float64)
    v = (raw - yref) * yinc + yorig
    t = xorig + (np.arange(len(v)) - xref) * xinc
    return dict(t=t, v=v, xinc=xinc, yinc=yinc, points=npts)

def arm_single(s, source="CHANnel1", level=1.0, slope="POSitive"):
    s.write(":TRIGger:MODE EDGE")
    s.write(":TRIGger:EDGE:SOURce " + source)
    s.write(":TRIGger:EDGE:SLOPe " + slope)
    s.write(":TRIGger:EDGE:LEVel %.3f" % level)
    s.write(":TRIGger:SWEep SINGle")
    s.write(":SINGle")

def wait_stop(s, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if s.query(":TRIGger:STATus?").strip().upper().startswith("STOP"):
                return True
        except Exception:
            pass
        time.sleep(0.01)
    return False

def wait_arm(s, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            st = s.query(":TRIGger:STATus?").strip().upper()
            if st.startswith("WAIT") or st.startswith("RUN") or st.startswith("AUTO"):
                return True
        except Exception:
            pass
        time.sleep(0.01)
    return False
