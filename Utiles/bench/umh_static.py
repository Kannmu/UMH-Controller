import struct
import time
import sys
sys.path.insert(0, "Utiles/bench")
from umh_link import UmhLink, MSG_BLOCK_BEGIN, MSG_BLOCK_DATA, MSG_BLOCK_END, MSG_SET_PLAN, MSG_START_PLAN, MSG_STOP_PLAN, MSG_CLEAR_PLAN

def block_payload(channel, level, phase=0):
    hdr = struct.pack("<IIQQIIHH", 1, 1000000, 0, 1000000, 1, 0, 1, 0)
    desc = struct.pack("<HBBBBHBBHBBHHI", 0, 1, 1, 1, 0, 3, 1, 0, 1, 8, 0, channel, 0, 0)
    rec = b"\x00" + struct.pack("<H", 0) + b"\x01" + struct.pack("<BB", phase, level)
    return hdr + desc, rec

def plan_payload(block_id=1):
    return struct.pack("<IBBBBIIQII", block_id, 0, 0, 0, 0, 1, 1, 0, 1, 0)

def send_static(link, channel, level, phase=0):
    body, rec = block_payload(channel, level, phase)
    link.request(MSG_BLOCK_BEGIN, body, stream_sequence=0)
    link.request(MSG_BLOCK_DATA, rec, stream_sequence=1)
    link.request(MSG_BLOCK_END, b"", stream_sequence=2)
    link.request(MSG_SET_PLAN, plan_payload())
    link.request(MSG_START_PLAN, b"")

def stop_static(link):
    try:
        link.request(MSG_STOP_PLAN, b"", timeout=2.0)
    except Exception:
        pass
    try:
        link.request(MSG_CLEAR_PLAN, b"", timeout=2.0)
    except Exception:
        pass

def send_sparse(link, channel, level, phase=0):
    hdr = struct.pack("<IIQQIIHH", 1, 1000000, 0, 1000000, 1, 0, 1, 0)
    desc = struct.pack("<HBBBBHBBHBBHHI", 0, 1, 1, 3, 0, 3, 3, 0, 1, 8, 0, 0, 0, 0)
    rec = b"\x00" + struct.pack("<H", 0) + b"\x01" + b"\x04" + struct.pack("<HBB", channel, phase, level)
    link.request(MSG_BLOCK_BEGIN, hdr + desc, stream_sequence=0)
    link.request(MSG_BLOCK_DATA, rec, stream_sequence=1)
    link.request(MSG_BLOCK_END, b"", stream_sequence=2)
    link.request(MSG_SET_PLAN, plan_payload())
    link.request(MSG_START_PLAN, b"")

def send_dense(link, levels, phases):
    hdr = struct.pack("<IIQQIIHH", 1, 1000000, 0, 1000000, 1, 0, 1, 0)
    desc = struct.pack("<HBBBBHBBHBBHHI", 0, 1, 1, 2, 0, 3, 0, 0, 0, 8, 0, 0, 0, 0)
    payload = b"".join(struct.pack("<BB", int(phases[i]), int(levels[i])) for i in range(84))
    rec = b"\x00" + struct.pack("<H", 0) + b"\x01" + payload
    link.request(MSG_BLOCK_BEGIN, hdr + desc, stream_sequence=0)
    link.request(MSG_BLOCK_DATA, rec, stream_sequence=1)
    link.request(MSG_BLOCK_END, b"", stream_sequence=2)
    link.request(MSG_SET_PLAN, plan_payload())
    link.request(MSG_START_PLAN, b"")
