import sys, time
sys.path.insert(0, "Utiles/bench")
import umh_static as us
from umh_link import UmhLink
l = UmhLink("COM4")
def sysstat():
    b = l.request(3, b"", 4)
    flags = int.from_bytes(b[0:4], "little")
    ring = int.from_bytes(b[4:8], "little")
    return flags, ring >> 16, ring & 0xFFFF
def fpga():
    st = l.get_fpga_status()
    return int.from_bytes(st[6:8], "little"), int.from_bytes(st[8:12], "little"), int.from_bytes(st[12:16], "little")

try:
    us.stop_static(l); time.sleep(0.3)
    print("pre", sysstat(), fpga())
    us.send_dense(l, [128]*84, [0]*84)
    for i in range(14):
        time.sleep(0.4)
        print("t%d" % i, sysstat(), fpga())
finally:
    us.stop_static(l); l.close()

