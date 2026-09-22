import sys, time
sys.path.insert(0, r"D:\Data\OneDrive\Projects\UMH\Software\UMH Controller\Utiles\bench")
from umh_link import UmhLink

link = UmhLink("COM4")
try:
    p = link.get_profile()
    print("profile:", p)
    print("fpga:", link.get_fpga_status().hex())
    print("motion:", link.get_motion_status())
    pts = [(int(12_000*__import__('math').cos(i/64*6.283)), int(12_000*__import__('math').sin(i/64*6.283)), 70_000, 200, i % 16) for i in range(64)]
    link.motion_config(0, 0x0C, 800, 100, 2000, 100000, 200, 0, 2000, 128, 16, [(255,0,0)]*16)
    link.motion_upload(pts)
    link.motion_start()
    time.sleep(1.5)
    st = link.get_motion_status()
    print("status:", st)
    link.motion_stop()
    time.sleep(0.1)
    print("after stop:", link.get_motion_status())
finally:
    link.close()
