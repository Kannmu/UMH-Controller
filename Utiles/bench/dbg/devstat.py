import sys
sys.path.insert(0, "Utiles/bench")
from umh_link import UmhLink
import umh_static as us
l = UmhLink("COM4")
try:
    print("profile:", l.get_profile())
    print("fpga:", l.get_fpga_status().hex())
    print("err:", l.get_error_counters().hex())
finally:
    us.stop_static(l)
    l.close()

