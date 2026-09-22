import sys, time
sys.path.insert(0, r"D:\Data\OneDrive\Projects\UMH\Software\UMH Controller\Utiles\bench")
from umh_link import UmhLink, make_path, MOTION_FLAG_RGB, MOTION_FLAG_LOOP
link = UmhLink("COM4")
try:
    for rate in (200, 500, 1000):
        link.motion_stop(); link.clear_plan()
        link.motion_config(0, MOTION_FLAG_RGB|MOTION_FLAG_LOOP, rate, 200, 3000, 65535, 200, 0, 2000, 128, 16, [(255,0,0)]*16)
        link.motion_upload(make_path("circle", 96))
        link.motion_start()
        time.sleep(0.3)
        a = link.get_motion_status(); t0 = time.monotonic()
        time.sleep(5.0)
        b = link.get_motion_status(); t1 = time.monotonic()
        print(f"rate={rate}: real={(b.frames-a.frames)/(t1-t0):.1f} fps, device_fps={b.fps_x100/100:.1f}, missed_delta={b.missed_deadlines-a.missed_deadlines}, frames_delta={b.frames-a.frames}")
        link.motion_stop(); time.sleep(0.05)
finally:
    link.close()
