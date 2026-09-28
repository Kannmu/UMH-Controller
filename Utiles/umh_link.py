"""Minimal UMH v7 USB-CDC host helper for bench tests.

Emits exact *static* ultrasound states through the ordinary protocol path
(BLOCK_BEGIN/DATA/END + SET_PLAN(ONCE) + START_PLAN).  A ONCE plan leaves the
last committed frame active and the render task never resubmits it, so the
FPGA holds a bit-exact static pattern.

Examples:
  python Utiles/umh_link.py off
  python Utiles/umh_link.py all --level 128 --phase 0
  python Utiles/umh_link.py single --channel 41 --level 128
  python Utiles/umh_link.py point --x 0 --y 0 --z 100000 --level 255
  python Utiles/umh_link.py status
"""
import argparse
import struct
import sys
import time

import serial

SYNC0, SYNC1, VER, HDR = 0x55, 0xAA, 7, 16
MSG = dict(GET_STATUS=0x03, BLOCK_BEGIN=0x10, BLOCK_DATA=0x11, BLOCK_END=0x12,
           BLOCK_CANCEL=0x13, SET_PLAN=0x20, START_PLAN=0x21, STOP_PLAN=0x22,
           CLEAR_PLAN=0x23, FPGA_STATUS=0x30, ERROR_COUNTERS=0x60,
           ACK=0x70, NACK=0x71)
FLAG_ACK_REQUIRED = 0x01
CHANNELS = 84


class UmhLink:
    def __init__(self, port="COM4", timeout=1.0):
        self.ser = serial.Serial(port, 115200, timeout=timeout)
        self.tid = int(time.time()) & 0xFFFF
        self.seq = 1
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _frame(self, mtype, payload=b"", seq=0):
        self.tid = (self.tid + 1) & 0xFFFFFFFF
        hdr = struct.pack("<BBBBBBHII", SYNC0, SYNC1, VER, mtype, FLAG_ACK_REQUIRED,
                          HDR, len(payload), self.tid, seq)
        return hdr + payload, self.tid

    def _read_response(self, tid, timeout=2.0):
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            buf += self.ser.read(self.ser.in_waiting or 1)
            while len(buf) >= HDR:
                i = buf.find(bytes([SYNC0, SYNC1]))
                if i < 0:
                    buf = b""
                    break
                buf = buf[i:]
                if len(buf) < HDR:
                    break
                _, _, ver, mtype, flags, hl, plen, rtid, rseq = struct.unpack("<BBBBBBHII", buf[:HDR])
                if len(buf) < HDR + plen:
                    break
                payload = buf[HDR:HDR + plen]
                buf = buf[HDR + plen:]
                if rtid == tid:
                    return mtype, payload
        raise TimeoutError("no response for tid %d" % tid)

    def request(self, name, payload=b"", seq=0, check=True):
        data, tid = self._frame(MSG[name], payload, seq)
        self.ser.write(data)
        mtype, resp = self._read_response(tid)
        if check and mtype == MSG["NACK"]:
            raise RuntimeError("%s NACK status=%d" % (name, resp[0] if resp else -1))
        return mtype, resp

    # ------------------------------------------------------------------
    def stop(self):
        self.request("STOP_PLAN", check=False)
        self.request("CLEAR_PLAN", check=False)

    def _send_block(self, track, record_payload):
        header = struct.pack("<IIQQIIHH", 1, 1_000_000, 0, 1000, 1, 0, 1, 0)
        seq = self.seq
        self.request("BLOCK_BEGIN", header + track, seq=seq)
        record = bytes([0x00]) + struct.pack("<HB", 0, 0x01) + record_payload
        self.request("BLOCK_DATA", record, seq=seq + 1)
        self.request("BLOCK_END", b"", seq=seq + 2)
        self.seq += 3
        plan = struct.pack("<IBBBBIIQII", 1, 0, 0, 0, 0, 1, 1, 0, 0, 0)
        self.request("SET_PLAN", plan)
        self.request("START_PLAN")

    def emit_channels(self, phases, levels):
        assert len(phases) == CHANNELS and len(levels) == CHANNELS
        # track_id, payload_type=CHANNEL_STATE, value U8, DENSE, HOLD,
        # component PHASE|LEVEL, target ALL, unit 0, count 84, qbits 8,
        # attr 0, start 0, stride 2, reserved
        track = struct.pack("<HBBBBHBBHBBHHI", 0, 1, 1, 2, 0, 0x3, 0, 0, CHANNELS,
                            8, 0, 0, 2, 0)
        payload = bytes(b for pl in zip(phases, levels) for b in pl)
        self.stop()
        self._send_block(track, payload)

    def emit_point(self, x_um, y_um, z_um, level=255, phase=0):
        track = struct.pack("<HBBBBHBBHBBHHI", 0, 2, 4, 1, 0, 0x18, 0, 0, 0,
                            0, 0, 0, 15, 0)
        payload = struct.pack("<iiiBBB", x_um, y_um, z_um, level, phase, 0)
        self.stop()
        self._send_block(track, payload)

    def status(self):
        _, resp = self.request("GET_STATUS")
        flags, counts = struct.unpack("<II", resp[:8])
        return flags, counts


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["off", "all", "single", "point", "status"])
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--level", type=int, default=128)
    ap.add_argument("--phase", type=int, default=0)
    ap.add_argument("--channel", type=int, default=0)
    ap.add_argument("--x", type=int, default=0)
    ap.add_argument("--y", type=int, default=0)
    ap.add_argument("--z", type=int, default=100000)
    a = ap.parse_args(argv)
    link = UmhLink(a.port)
    try:
        if a.cmd == "off":
            link.stop()
        elif a.cmd == "all":
            link.emit_channels([a.phase] * CHANNELS, [a.level] * CHANNELS)
        elif a.cmd == "single":
            lv = [0] * CHANNELS
            lv[a.channel] = a.level
            link.emit_channels([a.phase] * CHANNELS, lv)
        elif a.cmd == "point":
            link.emit_point(a.x, a.y, a.z, a.level, a.phase)
        flags, counts = link.status()
        print("status flags=0x%08X frames=%d free=%d" % (flags, counts >> 16, counts & 0xFFFF))
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
