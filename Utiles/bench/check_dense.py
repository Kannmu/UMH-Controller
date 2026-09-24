import sys
import time
lev = int(sys.argv[1]) if len(sys.argv) > 1 else 64
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc
import umh_static as us
from umh_link import UmhLink
s = sc.open_scope()
l = UmhLink("COM4")
try:
    s.write(":TRIGger:EDGE:SOURce CHANnel1")
    s.write(":TRIGger:EDGE:SLOPe POSitive")
    s.write(":TRIGger:EDGE:LEVel 1")
    s.write(":TRIGger:SWEep SINGle")
    s.write(":SINGle")
    us.send_dense(l, [lev]*84, [0]*84)
    time.sleep(0.3)
    print("fpga", l.get_fpga_status().hex())
    print("trig", s.query(":TRIGger:STATus?").strip())
    for src in ["CHANnel1", "CHANnel2", "MATH"]:
        v = sc.capture(s, src, 12000)["v"]
        print(src, "mean", round(float(v.mean()),2), "pp", round(float((v.max()-v.min())),2), "rms", round(float(v.std()),2))
finally:
    us.stop_static(l)
    l.close()
    s.close()
