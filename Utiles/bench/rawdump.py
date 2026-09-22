import sys, time, struct
sys.path.insert(0, r"D:\Data\OneDrive\Projects\UMH\Software\UMH Controller\Utiles\bench")
from umh_link import UmhLink, MSG_MOTION_STATUS
link = UmhLink("COM4")
try:
    body = link.request(MSG_MOTION_STATUS, b"", MSG_MOTION_STATUS)
    print("len", len(body)); print(body.hex())
    for off in range(0, len(body)-3, 2):
        print(off, struct.unpack_from("<H", body, off)[0])
finally:
    link.close()
