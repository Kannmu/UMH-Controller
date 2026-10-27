#!/usr/bin/env python3
"""Modulated-aperture streaming bench for the UMH controller.

Speaks the v7 CDC protocol directly over the COM port so the envelope pacing
can be swept without the browser in the loop, and reports the device's own
counters before/after each run.

Wire contract (Core/Inc/umh_protocol.h, aperture_mod.h):
  16-byte header: 55 AA 07 type flags hlen=16 u16 payload u32 txn u32 seq
  MOD_CONFIGURE (0x90): 20-byte umh_mod_config_wire_t (+ optional fan-out)
  MOD_START     (0x91): empty, ACK
  MOD_FEED      (0x92): raw 8-bit envelope samples, no ACK
  MOD_STOP      (0x93): empty, ACK
  MOD_STATUS    (0x94): empty -> 32-byte umh_mod_status_wire_t
"""

from __future__ import annotations

import argparse
import math
import struct
import subprocess
import sys
import time
from dataclasses import dataclass

import serial

GDB = r"D:\SOFTWARE\GCC-ARM-NONE-EABI-10.3-2021.10\BIN\arm-none-eabi-gdb.exe"
GDB_HOST = ":3333"

SYNC0 = 0x55
SYNC1 = 0xAA
VERSION = 7
HEADER = 16

MSG_GET_PROFILE = 0x01
MSG_PROFILE = 0x02
MSG_ACK = 0x70
MSG_NACK = 0x71
MSG_MOD_CONFIGURE = 0x90
MSG_MOD_START = 0x91
MSG_MOD_FEED = 0x92
MSG_MOD_STOP = 0x93
MSG_MOD_STATUS = 0x94

FLAG_ACK_REQUIRED = 0x01
FLAG_RESPONSE = 0x08

MOD_STATUS_FMT = "<BBHHHIIIIiI"
MOD_STATUS_SIZE = struct.calcsize(MOD_STATUS_FMT)


@dataclass
class ModStatus:
    state: int
    flags: int
    ring_fill: int
    ring_capacity: int
    prebuffer: int
    underrun: int
    overrun: int
    packet_loss: int
    rendered: int
    ppm: int
    max_service_us: int

    @classmethod
    def unpack(cls, payload: bytes) -> "ModStatus":
        if len(payload) != MOD_STATUS_SIZE:
            raise ValueError(f"mod status is {len(payload)} bytes, want {MOD_STATUS_SIZE}")
        return cls(*struct.unpack(MOD_STATUS_FMT, payload))

    def __str__(self) -> str:
        return (
            f"state={self.state} flags=0x{self.flags:02x} fill={self.ring_fill}/{self.ring_capacity} "
            f"pre={self.prebuffer} under={self.underrun} over={self.overrun} "
            f"lost={self.packet_loss} rendered={self.rendered} ppm={self.ppm} "
            f"maxsvc={self.max_service_us}us"
        )


class Device:
    def __init__(self, port: str, baud: int = 2_000_000) -> None:
        self.port = self._open(port, baud)
        self.rx = bytearray()
        self.txn = 1
        self.pending: dict[int, tuple[int, bytes]] = {}
        self.feed_seq = 0
        self.writes = 0
        self.bytes_written = 0

    @staticmethod
    def _open(port: str, baud: int) -> serial.Serial:
        """Open COMx, resetting the MCU once if Windows wedged the port."""
        try:
            return serial.Serial(port, baud, timeout=0, write_timeout=0.5)
        except serial.SerialException:
            subprocess.run(
                [GDB, "-batch", "-nx", "-ex", f"target extended-remote {GDB_HOST}",
                 "-ex", "monitor reset run", "-ex", "detach"],
                capture_output=True, timeout=30)
            time.sleep(2.5)
            return serial.Serial(port, baud, timeout=0, write_timeout=0.5)

    def close(self) -> None:
        try:
            self.port.close()
        except Exception:
            pass

    # ---------------------------------------------------------------- raw io
    def _encode(self, mtype: int, payload: bytes, txn: int, seq: int, flags: int) -> bytes:
        return (
            struct.pack("<BBBBBBHII", SYNC0, SYNC1, VERSION, mtype, flags, HEADER,
                        len(payload), txn, seq)
            + payload
        )

    def write(self, data: bytes) -> None:
        self.port.write(data)
        self.writes += 1
        self.bytes_written += len(data)

    def _pump(self, timeout: float) -> None:
        """Read available bytes and decode frames until timeout expires."""
        deadline = time.perf_counter() + timeout
        while True:
            chunk = self.port.read(4096)
            if chunk:
                self.rx.extend(chunk)
                self._parse()
            if time.perf_counter() >= deadline:
                return
            time.sleep(0.0002)

    def _parse(self) -> None:
        buf = self.rx
        while True:
            pos = buf.find(b"\x55\xaa\x07")
            if pos < 0:
                if len(buf) > 2:
                    del buf[: len(buf) - 2]
                return
            if pos:
                del buf[:pos]
            if len(buf) < HEADER:
                return
            payload_len = buf[6] | (buf[7] << 8)
            total = HEADER + payload_len
            if len(buf) < total:
                return
            txn = struct.unpack_from("<I", buf, 8)[0]
            mtype = buf[3]
            payload = bytes(buf[HEADER:total])
            del buf[:total]
            self.pending[txn] = (mtype, payload)

    def request(self, mtype: int, payload: bytes = b"", timeout: float = 0.5) -> tuple[int, bytes]:
        txn = self.txn
        self.txn += 1
        self.write(self._encode(mtype, payload, txn, 0, FLAG_ACK_REQUIRED))
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            self._pump(0.001)
            if txn in self.pending:
                return self.pending.pop(txn)
        raise TimeoutError(f"no reply to 0x{mtype:02x} within {timeout}s")

    def command(self, mtype: int, payload: bytes = b"", timeout: float = 0.5) -> None:
        reply_type, reply = self.request(mtype, payload, timeout)
        if reply_type == MSG_NACK:
            raise RuntimeError(f"device NACK 0x{mtype:02x} status={reply[0] if reply else -1}")

    def feed(self, samples: bytes, lead_inflight: int | None = None) -> None:
        """MOD_FEED carries no ACK; sequence drives the device's loss counter."""
        self.write(self._encode(MSG_MOD_FEED, samples, 0, self.feed_seq, 0))
        self.feed_seq += 1

    def status(self) -> ModStatus:
        mtype, payload = self.request(MSG_MOD_STATUS, b"", 0.5)
        if mtype == MSG_NACK:
            raise RuntimeError("MOD_STATUS NACK")
        return ModStatus.unpack(payload)


# ------------------------------------------------------------------ envelope
def make_envelope(rate: int, seconds: float, tone_hz: float, depth: float) -> bytes:
    """Square-law AM envelope: level = 128 * (1 - depth + depth * sin)."""
    n = int(rate * seconds)
    out = bytearray(n)
    w = 2.0 * math.pi * tone_hz / rate
    for i in range(n):
        s = 0.5 + 0.5 * math.sin(w * i)
        out[i] = max(0, min(255, int(round(255.0 * (1.0 - depth + depth * s)))))
    return bytes(out)


def load_wav_envelope(path: str, rate: int) -> bytes:
    import wave

    with wave.open(path, "rb") as wf:
        channels = wf.getnchannels()
        width = wf.getsampwidth()
        src_rate = wf.getframerate()
        frames = wf.readframes(wf.getnframes())
    if width != 2:
        raise ValueError("only 16-bit PCM wav is supported")
    count = len(frames) // 2
    pcm = struct.unpack("<" + "h" * count, frames[: count * 2])
    mono = [0.0] * (count // channels)
    for i in range(len(mono)):
        mono[i] = sum(pcm[i * channels + j] for j in range(channels)) / (channels * 32768.0)
    # linear resample to the envelope rate
    out = bytearray(int(len(mono) * rate / src_rate))
    step = src_rate / rate
    for i in range(len(out)):
        x = i * step
        i0 = int(x)
        i1 = min(i0 + 1, len(mono) - 1)
        frac = x - i0
        v = mono[i0] * (1.0 - frac) + mono[i1] * frac
        out[i] = max(0, min(255, int(round(128.0 + v * 127.0))))
    return bytes(out)


# ------------------------------------------------------------------- streaming
def run(args: argparse.Namespace) -> int:
    dev = Device(args.port)
    try:
        profile_type, profile = dev.request(MSG_GET_PROFILE, b"", 2.0)
        if profile_type != MSG_PROFILE:
            raise RuntimeError(f"unexpected profile reply 0x{profile_type:02x}")

        dev.command(MSG_MOD_CONFIGURE, struct.pack(
            "<iiiBBHHH", args.x_um, args.y_um, args.z_um, 0, args.level,
            args.rate, args.prebuffer, 1 if args.planewave else 0))
        # Read the zeroed counters before the stream starts: the device begins
        # consuming as soon as the prebuffer lands, so any request after that
        # steals runway the host application would also not have.
        base = dev.status()
        dev.command(MSG_MOD_START)

        env = (load_wav_envelope(args.wav, args.rate) if args.wav
               else make_envelope(args.rate, args.seconds + 1.0, args.tone, args.depth))
        env_pos = 0
        packet = args.packet
        lead = args.lead

        t0 = time.perf_counter()
        start_feed_seq = dev.feed_seq
        sent = 0
        dev.feed(bytes(args.prebuffer))   # prime: the device starts eating here
        last_report = t0
        peak_fill = 0
        min_fill = 1 << 30
        fill_sum = 0
        fill_n = 0
        rows: list[tuple[float, int, int, int, int]] = []

        def send_block() -> bool:
            nonlocal env_pos, sent, env
            if env_pos + packet > len(env):
                if args.wav:
                    env_pos = 0
                else:
                    env += env[: max(packet * 4, len(env))]
            dev.feed(env[env_pos: env_pos + packet])
            env_pos += packet
            sent += packet
            return True

        while True:
            now = time.perf_counter()
            elapsed = now - t0
            if elapsed >= args.seconds:
                break
            if args.mode == "fill":
                st = dev.status()
                fill_sum += st.ring_fill
                fill_n += 1
                peak_fill = max(peak_fill, st.ring_fill)
                min_fill = min(min_fill, st.ring_fill)
                rows.append((elapsed, st.ring_fill, st.underrun, st.overrun, st.packet_loss))
                if args.verbose:
                    print(f"t={elapsed:6.2f}s {st}")
                if st.ring_fill + packet <= args.fill_target:
                    send_block()
                time.sleep(0.002)
                continue
            allowed = int(elapsed * args.rate) + lead
            if args.mode == "bounded":
                allowed = min(allowed, sent + args.burst)
            while sent + packet <= allowed:
                send_block()
            # wake when the next packet becomes due under the active policy
            next_due = t0 + (sent + packet) / args.rate
            delay = next_due - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                time.sleep(0)
            if args.status_hz > 0 and now - last_report >= 1.0 / args.status_hz:
                last_report = now
                st = dev.status()
                peak_fill = max(peak_fill, st.ring_fill)
                min_fill = min(min_fill, st.ring_fill)
                fill_sum += st.ring_fill
                fill_n += 1
                rows.append((elapsed, st.ring_fill, st.underrun, st.overrun, st.packet_loss))
                if args.verbose:
                    print(f"t={elapsed:6.2f}s {st}")

        wall = time.perf_counter() - t0
        final = dev.status()
        dev.command(MSG_MOD_STOP)
        time.sleep(0.2)

        print(f"final    {final}")
        print(f"sent={sent} samples in {wall:.2f}s "
              f"({sent / wall:.0f} samp/s), packets={dev.feed_seq - start_feed_seq}, "
              f"bytes={dev.bytes_written}")
        print(f"overrun +{final.overrun - base.overrun}, underrun +{final.underrun - base.underrun}, "
              f"lost +{final.packet_loss - base.packet_loss}, "
              f"rendered +{final.rendered - base.rendered}")
        if fill_n:
            print(f"ring fill: min={min_fill} mean={fill_sum / fill_n:.0f} peak={peak_fill} "
                  f"(capacity {final.ring_capacity}, prebuffer {final.prebuffer})")
        if args.csv:
            with open(args.csv, "w", encoding="utf-8") as fh:
                fh.write("t_s,fill,underrun,overrun,lost\n")
                for row in rows:
                    fh.write(f"{row[0]:.3f},{row[1]},{row[2]},{row[3]},{row[4]}\n")
        ok = (final.overrun == base.overrun) and (final.underrun == base.underrun)
        return 0 if ok else 2
    finally:
        dev.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--rate", type=int, default=20000)
    ap.add_argument("--prebuffer", type=int, default=512)
    ap.add_argument("--level", type=int, default=255)
    ap.add_argument("--packet", type=int, default=256)
    ap.add_argument("--lead", type=int, default=256, help="samples the pacer may run ahead")
    ap.add_argument("--burst", type=int, default=2048,
                    help="bounded mode: max samples a single catch-up may queue")
    ap.add_argument("--fill-target", type=int, default=1024,
                    help="fill mode: device ring fill to hold")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--tone", type=float, default=440.0)
    ap.add_argument("--depth", type=float, default=0.48)
    ap.add_argument("--wav", default="")
    ap.add_argument("--planewave", action="store_true", default=True)
    ap.add_argument("--x-um", type=int, default=0)
    ap.add_argument("--y-um", type=int, default=0)
    ap.add_argument("--z-um", type=int, default=250000)
    ap.add_argument("--status-hz", type=float, default=10.0)
    ap.add_argument("--mode", choices=("wall", "bounded", "fill"), default="wall")
    ap.add_argument("--csv", default="")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    try:
        return run(args)
    except Exception as exc:  # noqa: BLE001 - bench tool
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
