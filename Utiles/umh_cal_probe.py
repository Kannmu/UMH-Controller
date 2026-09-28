"""Bench client for UMH_MSG_CAL_PROBE (0x86): multi-gate microphone I/Q profile.

The device drives an arbitrary 84-channel phase/level pattern for burst_us,
samples gate_count gates (gate_width x 25 us, gate_step x 25 us apart, first
gate gate_start x 25 us after the FPGA pattern swap), optionally subtracts the
phase-inverted pattern, and returns float32 [gate][mic][I,Q].

    python Utiles/umh_cal_probe.py --channel 41 --level 128 --gates 64 --step 1 --width 1
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

import numpy as np
import serial

SYNC0, SYNC1, VER, HDR = 0x55, 0xAA, 7, 16
MSG_CAL_PROBE = 0x86
MSG_NACK = 0x71
CHANNELS, MICS = 84, 4
_HEADER = struct.Struct("<BBBBBBHII")


class ProbeError(RuntimeError):
    pass


class UmhProbe:
    def __init__(self, port: str = "COM4"):
        self.ser = serial.Serial(port, 2_000_000, timeout=0.05)
        self.ser.reset_input_buffer()
        self.tid = (int(time.time()) & 0xFFFF) << 8
        self.buf = bytearray()

    def close(self) -> None:
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _send(self, mtype: int, payload: bytes) -> int:
        self.tid = (self.tid + 1) & 0xFFFFFFFF
        self.ser.write(_HEADER.pack(SYNC0, SYNC1, VER, mtype, 1, HDR, len(payload),
                                    self.tid, 0) + payload)
        return self.tid

    def _recv(self, tid: int, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                self.buf.extend(chunk)
            while len(self.buf) >= HDR:
                if self.buf[0] != SYNC0 or self.buf[1] != SYNC1 or self.buf[5] != HDR:
                    del self.buf[0]
                    continue
                _, _, _, mtype, flags, _, plen, rtid, rseq = _HEADER.unpack_from(self.buf, 0)
                if len(self.buf) < HDR + plen:
                    break
                payload = bytes(self.buf[HDR:HDR + plen])
                del self.buf[:HDR + plen]
                if rtid == tid:
                    return mtype, flags, rseq, payload
        raise TimeoutError("no CAL_PROBE reply")

    def probe(self, phase, level, gate_count=64, gate_start=0, gate_step=1,
              gate_width=1, burst_us=None, settle_us=20000, repeats=1, diff=True,
              timeout=None) -> tuple[np.ndarray, int]:
        """Return (complex array [gate, mic], saturated_blocks)."""
        phase = np.asarray(phase, dtype=np.uint8).reshape(CHANNELS)
        level = np.asarray(level, dtype=np.uint8).reshape(CHANNELS)
        if burst_us is None:
            burst_us = (gate_start + (gate_count - 1) * gate_step + gate_width) * 25 + 2500
        payload = struct.pack("<BBHHHHBB4x", gate_count, gate_width, gate_start, gate_step,
                              int(burst_us), int(settle_us), repeats, 1 if diff else 0)
        payload += phase.tobytes() + level.tobytes()
        if timeout is None:
            per = (burst_us + settle_us) * 1e-6 + gate_count * 2e-4 + 0.01
            timeout = 2.0 + repeats * (2 if diff else 1) * per * 2
        tid = self._send(MSG_CAL_PROBE, payload)
        mtype, flags, rseq, data = self._recv(tid, timeout)
        if mtype == MSG_NACK:
            raise ProbeError("CAL_PROBE NACK status=%d" % (data[0] if data else -1))
        rc = struct.unpack("<h", struct.pack("<H", rseq & 0xFFFF))[0]
        if rc != 0 or not data:
            raise ProbeError("CAL_PROBE failed rc=%d" % rc)
        v = np.frombuffer(data, dtype="<f4").reshape(gate_count, MICS, 2)
        return v[..., 0] + 1j * v[..., 1], rseq >> 16

    def single(self, channel: int, level: int = 128, phase: int = 0, **kw):
        ph = np.zeros(CHANNELS, np.uint8)
        lv = np.zeros(CHANNELS, np.uint8)
        ph[channel] = phase
        lv[channel] = level
        return self.probe(ph, lv, **kw)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--channel", type=int, default=41)
    ap.add_argument("--level", type=int, default=128)
    ap.add_argument("--gates", type=int, default=64)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--step", type=int, default=1)
    ap.add_argument("--width", type=int, default=1)
    ap.add_argument("--burst", type=int, default=None)
    ap.add_argument("--settle", type=int, default=20000)
    ap.add_argument("--repeats", type=int, default=4)
    ap.add_argument("--nodiff", action="store_true")
    ap.add_argument("--save", default=None)
    a = ap.parse_args(argv)
    with UmhProbe(a.port) as p:
        t0 = time.time()
        z, sat = p.single(a.channel, a.level, gate_count=a.gates, gate_start=a.start,
                          gate_step=a.step, gate_width=a.width, burst_us=a.burst,
                          settle_us=a.settle, repeats=a.repeats, diff=not a.nodiff)
        dt = time.time() - t0
    print("channel %d level %d  %.2f s  saturated=%d" % (a.channel, a.level, dt, sat))
    for g in range(z.shape[0]):
        t_us = (a.start + g * a.step) * 25
        cells = "  ".join("%7.1f %6.1f" % (abs(z[g, m]), np.degrees(np.angle(z[g, m])))
                          for m in range(MICS))
        print("%5d us  %s" % (t_us, cells))
    if a.save:
        np.save(a.save, z)
    return 0


if __name__ == "__main__":
    sys.exit(main())
