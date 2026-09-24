import sys, time, numpy as np
sys.path.insert(0, "Utiles/bench")
import scope_umh as sc, umh_static as us
from umh_link import UmhLink
def stat(v):
    return "n=%d min=%.2f max=%.2f mean=%.2f pp=%.2f" % (len(v), v.min(), v.max(), v.mean(), v.max()-v.min())

def cap(s, tag):
    sc.arm_single(s, "CHANnel1", 2.0)
    ok = sc.wait_stop(s, 1.5)
    d = sc.capture(s, "MATH", 4000)
    print(tag, "trig", ok, stat(d["v"]))
    return d["v"]
s = sc.open_scope(); l = UmhLink("COM4")
try:
    for i in range(2):
        a = cap(s, "off%d" % i); time.sleep(0.2)

    us.stop_static(l); time.sleep(0.3)
    for i in range(2):
        a = cap(s, "offz%d" % i)
    us.send_static(l, 0, 16); time.sleep(0.3)
    for i in range(2):
        b = cap(s, "ch0g16_%d" % i)
    us.send_static(l, 0, 128); time.sleep(0.3)
    c = cap(s, "ch0g128")
finally:
    us.stop_static(l); l.close(); s.close()

