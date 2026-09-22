"""Minimal UMH v7 serial client used by the local benchmark tools.

Only the control-path messages needed for performance measurements are
implemented here; the same wire format is shared with UMH Demo and the device
firmware (Docs/UMH_v7_Protocol.md).
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

import serial

SYNC0 = 0x55
SYNC1 = 0xAA
PROTOCOL_VERSION = 7
HEADER_SIZE = 16
MAX_PAYLOAD = 2048

MSG_GET_PROFILE = 0x01
MSG_PROFILE = 0x02
MSG_GET_STATUS = 0x03
MSG_STATUS = 0x04
MSG_BLOCK_BEGIN = 0x10
MSG_BLOCK_DATA = 0x11
MSG_BLOCK_END = 0x12
MSG_BLOCK_CANCEL = 0x13
MSG_SET_PLAN = 0x20
MSG_START_PLAN = 0x21
MSG_STOP_PLAN = 0x22
MSG_CLEAR_PLAN = 0x23
MSG_FPGA_STATUS = 0x30
MSG_ERROR_COUNTERS = 0x60
MSG_SET_DEMO = 0x61
MSG_MOTION_UPLOAD = 0x62
MSG_MOTION_CONFIG = 0x63
MSG_MOTION_START = 0x64
MSG_MOTION_TARGET = 0x65
MSG_MOTION_STOP = 0x66
MSG_MOTION_STATUS = 0x67
MSG_ACK = 0x70
MSG_NACK = 0x71
MSG_CAL_RESULT = 0x81
MSG_CAL_START = 0x82
MSG_AUDIO_CONFIGURE = 0x90
MSG_AUDIO_START = 0x91
MSG_AUDIO_DATA = 0x92
MSG_AUDIO_STOP = 0x93
MSG_AUDIO_STATUS = 0x94

FLAG_ACK_REQUIRED = 0x01
FLAG_FIRST = 0x02
FLAG_LAST = 0x04
FLAG_RESPONSE = 0x08
FLAG_ERROR = 0x10

STATUS_OK = 0

MOTION_FLAG_PAUSED = 1 << 0
MOTION_FLAG_DIRECT = 1 << 1
MOTION_FLAG_RGB = 1 << 2
MOTION_FLAG_LOOP = 1 << 3
MOTION_FLAG_LINEAR = 1 << 4
MOTION_FLAG_TRAP_PATTERN = 1 << 5
MOTION_FLAG_STEP = 1 << 6

UMH_MOTION_MAX_POINTS = 255


class DeviceError(RuntimeError):
    def __init__(self, message_type: int, status: int):
        super().__init__(f"device rejected 0x{message_type:02x}: status {status}")
        self.message_type = message_type
        self.status = status


@dataclass
class Profile:
    model: str
    firmware: str
    protocol: str
    serial: str
    channel_count: int
    rgb_count: int
    microphone_count: int
    max_frame_rate: int
    timebase_hz: int
    carrier_hz: int
    sound_speed: int
    capability_flags: int

    @property
    def has_motion(self) -> bool:
        return bool(self.capability_flags & (1 << 9))

    @property
    def has_focused_am(self) -> bool:
        return bool(self.capability_flags & (1 << 8))

    @property
    def has_focused_am_multi(self) -> bool:
        return bool(self.capability_flags & (1 << 10))


@dataclass
class MotionStatus:
    state: int
    flags: int
    output_rate_hz: int
    path_points: int
    loop_ms: int
    x_um: int
    y_um: int
    z_um: int
    level: int
    trap_mode: int
    service_max_us: int
    service_avg_us: int
    render_max_us: int
    render_avg_us: int
    submit_max_us: int
    submit_avg_us: int
    frames: int
    missed_deadlines: int
    frame_errors: int
    fps_x100: int


@dataclass
class AudioStatus:
    state: int
    flags: int
    ring_fill: int
    ring_capacity: int
    prebuffer: int
    underrun_count: int
    overrun_count: int
    packet_loss_count: int
    rendered_samples: int
    clock_correction_ppm: int
    max_service_us: int


def _u16(value: int) -> bytes:
    return struct.pack("<H", value & 0xFFFF)


def _i16(value: int) -> bytes:
    return struct.pack("<h", max(-32768, min(32767, value)))


def _i32(value: int) -> bytes:
    return struct.pack("<i", int(value))


def _u32(value: int) -> bytes:
    return struct.pack("<I", value & 0xFFFFFFFF)


class UmhLink:
    def __init__(self, port: str, baudrate: int = 2_000_000, timeout: float = 0.5):
        self.port = serial.Serial(port, baudrate=baudrate, timeout=timeout, write_timeout=1.0)
        self.buffer = bytearray()
        self.transaction_id = 1
        self.unsolicited = []
        self.profile: Profile | None = None
        # drop whatever the device streamed before we attached
        self.port.reset_input_buffer()

    def close(self) -> None:
        try:
            self.port.close()
        except Exception:
            pass

    # ---------------------------------------------------------------- framing
    def _encode(self, message_type: int, payload: bytes, transaction_id: int,
                stream_sequence: int = 0, flags: int = FLAG_ACK_REQUIRED) -> bytes:
        header = struct.pack(
            "<BBB BBB H I I",
            SYNC0, SYNC1, PROTOCOL_VERSION, message_type, flags, HEADER_SIZE,
            len(payload), transaction_id, stream_sequence,
        )
        return header + payload

    def _read_frame(self, timeout: float):
        deadline = time.monotonic() + timeout
        while True:
            frame = self._extract_frame()
            if frame is not None:
                return frame
            if time.monotonic() > deadline:
                return None
            chunk = self.port.read(max(1, min(4096, self.port.in_waiting or 1)))
            if chunk:
                self.buffer.extend(chunk)

    def _extract_frame(self):
        buf = self.buffer
        while len(buf) >= HEADER_SIZE:
            if buf[0] != SYNC0 or buf[1] != SYNC1:
                del buf[0]
                continue
            payload_len = struct.unpack_from("<H", buf, 6)[0]
            if payload_len > MAX_PAYLOAD:
                del buf[0]
                continue
            if len(buf) < HEADER_SIZE + payload_len:
                return None
            frame = bytes(buf[: HEADER_SIZE + payload_len])
            del buf[: HEADER_SIZE + payload_len]
            return frame
        return None

    def request(self, message_type: int, payload: bytes = b"", expected: int | None = None,
                timeout: float = 2.0) -> bytes:
        if expected is None:
            expected = MSG_ACK
        self.transaction_id = (self.transaction_id + 1) & 0xFFFFFFFF
        transaction = self.transaction_id
        self.port.write(self._encode(message_type, payload, transaction))
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"message 0x{message_type:02x} timed out")
            frame = self._read_frame(remaining)
            if frame is None:
                raise TimeoutError(f"message 0x{message_type:02x} timed out")
            _, _, _, frame_type, flags, _, plen, rx_transaction, _ = struct.unpack_from("<BBB BBB H I I", frame, 0)
            body = frame[HEADER_SIZE: HEADER_SIZE + plen]
            if rx_transaction != transaction:
                self.unsolicited.append((frame_type, body))
                continue
            if frame_type == MSG_NACK or (flags & FLAG_ERROR):
                status = body[0] if body else -1
                raise DeviceError(frame_type, status)
            if frame_type != expected:
                raise RuntimeError(f"expected response 0x{expected:02x}, got 0x{frame_type:02x}")
            return body

    def send_no_reply(self, message_type: int, payload: bytes, stream_sequence: int = 0) -> None:
        self.transaction_id = (self.transaction_id + 1) & 0xFFFFFFFF
        self.port.write(self._encode(message_type, payload, self.transaction_id,
                                     stream_sequence, flags=0))

    # --------------------------------------------------------------- commands
    def get_profile(self) -> Profile:
        body = self.request(MSG_GET_PROFILE, b"", MSG_PROFILE)
        if len(body) != 1118:
            raise RuntimeError(f"profile length {len(body)}")
        capability_flags = struct.unpack_from("<I", body, 106)[0]
        self.profile = Profile(
            model=body[0:16].split(b"\0")[0].decode("ascii", "replace"),
            firmware=body[16:32].split(b"\0")[0].decode("ascii", "replace"),
            protocol=body[32:40].split(b"\0")[0].decode("ascii", "replace"),
            serial=body[40:64].split(b"\0")[0].decode("ascii", "replace"),
            channel_count=struct.unpack_from("<H", body, 64)[0],
            rgb_count=struct.unpack_from("<H", body, 66)[0],
            microphone_count=struct.unpack_from("<H", body, 68)[0],
            max_frame_rate=struct.unpack_from("<H", body, 74)[0],
            timebase_hz=struct.unpack_from("<I", body, 76)[0],
            carrier_hz=struct.unpack_from("<I", body, 80)[0],
            sound_speed=struct.unpack_from("<I", body, 84)[0],
            capability_flags=capability_flags,
        )
        return self.profile

    def get_fpga_status(self) -> bytes:
        return self.request(MSG_FPGA_STATUS, b"", MSG_FPGA_STATUS)

    def get_error_counters(self) -> bytes:
        return self.request(MSG_ERROR_COUNTERS, b"", MSG_ACK)

    def motion_upload(self, points: Sequence[tuple[int, int, int, int, int]]) -> None:
        payload = bytearray()
        payload += struct.pack("<BBH4B", 1, 0, len(points), 0, 0, 0, 0)
        for x_um, y_um, z_um, level, palette in points:
            payload += struct.pack("<hhhBB", max(-32768, min(32767, round(x_um / 10))),
                                   max(-32768, min(32767, round(y_um / 10))),
                                   max(-32768, min(32767, round(z_um / 10))),
                                   level & 0xFF, palette & 0xFF)
        self.request(MSG_MOTION_UPLOAD, bytes(payload))

    def motion_config(self, mode: int, flags: int, output_rate_hz: int, loop_ms: int,
                      max_speed_mm_s: int, max_accel_mm_s2: int, level: int,
                      trap_mode: int = 0, trap_radius_um: int = 2000,
                      trap_phase_span: int = 128, palette_count: int = 1,
                      palette: Iterable[tuple[int, int, int]] | None = None,
                      z_offset_um: int = 0, palette_spin_x10: int = 0,
                      path_spin_mrad_s: int = 0) -> None:
        payload = bytearray(69)
        payload[0] = mode
        payload[1] = flags
        struct.pack_into("<H", payload, 2, max(50, min(2000, output_rate_hz)))
        struct.pack_into("<H", payload, 4, max(2, min(60000, loop_ms)))
        struct.pack_into("<H", payload, 6, max(0, min(10000, max_speed_mm_s)))
        struct.pack_into("<H", payload, 8, max(0, min(500000, max_accel_mm_s2)))
        struct.pack_into("<h", payload, 10, max(-32768, min(32767, round(z_offset_um / 10))))
        payload[12] = level & 0xFF
        payload[13] = trap_mode & 0xFF
        struct.pack_into("<H", payload, 14, max(0, min(32767, round(trap_radius_um / 10))))
        payload[16] = trap_phase_span & 0xFF
        count = max(0, min(16, palette_count))
        payload[17] = count
        colors = list(palette or [(255, 255, 255)])
        for index in range(16):
            color = colors[index % len(colors)]
            payload[18 + index * 3] = color[0] & 0xFF
            payload[19 + index * 3] = color[1] & 0xFF
            payload[20 + index * 3] = color[2] & 0xFF
        payload[66] = max(0, min(250, palette_spin_x10))
        struct.pack_into("<h", payload, 67, max(-32768, min(32767, path_spin_mrad_s)))
        self.request(MSG_MOTION_CONFIG, bytes(payload))

    def motion_start(self) -> None:
        self.request(MSG_MOTION_START, b"")

    def motion_target(self, x_um: int, y_um: int, z_um: int, level: int, palette: int = 0) -> None:
        payload = struct.pack("<hhhBB", max(-32768, min(32767, round(x_um / 10))),
                              max(-32768, min(32767, round(y_um / 10))),
                              max(-32768, min(32767, round(z_um / 10))),
                              level & 0xFF, palette & 0xFF)
        self.request(MSG_MOTION_TARGET, payload)

    def motion_stop(self) -> None:
        self.request(MSG_MOTION_STOP, b"", timeout=3.0)

    def get_motion_status(self) -> MotionStatus:
        body = self.request(MSG_MOTION_STATUS, b"", MSG_MOTION_STATUS)
        if len(body) < 52:
            raise RuntimeError(f"motion status length {len(body)}")
        return MotionStatus(
            state=body[0], flags=body[1],
            output_rate_hz=struct.unpack_from("<H", body, 2)[0],
            path_points=struct.unpack_from("<H", body, 4)[0],
            loop_ms=struct.unpack_from("<H", body, 6)[0],
            x_um=struct.unpack_from("<i", body, 8)[0],
            y_um=struct.unpack_from("<i", body, 12)[0],
            z_um=struct.unpack_from("<i", body, 16)[0],
            level=body[20], trap_mode=body[21],
            service_max_us=struct.unpack_from("<H", body, 24)[0],
            service_avg_us=struct.unpack_from("<H", body, 26)[0],
            render_max_us=struct.unpack_from("<H", body, 28)[0],
            render_avg_us=struct.unpack_from("<H", body, 30)[0],
            submit_max_us=struct.unpack_from("<H", body, 32)[0],
            submit_avg_us=struct.unpack_from("<H", body, 34)[0],
            frames=struct.unpack_from('<I', body, 36)[0],
            missed_deadlines=struct.unpack_from('<I', body, 40)[0],
            frame_errors=struct.unpack_from('<I', body, 44)[0],
            fps_x100=struct.unpack_from('<I', body, 48)[0],
        )

    def audio_configure(self, focus: Sequence[tuple[int, int, int, int, int]],
                        envelope_rate_hz: int = 20000, prebuffer: int = 512,
                        envelope_level: int = 255) -> None:
        payload = bytearray()
        base = focus[0]
        payload += _i32(base[0]) + _i32(base[1]) + _i32(base[2])
        payload += bytes([base[3] & 0xFF, max(0, min(255, envelope_level)) & 0xFF])
        payload += _u16(envelope_rate_hz) + _u16(prebuffer) + _u16(0)
        extra = list(focus[1:8])
        if extra:
            payload.append(len(extra))
            for point in extra:
                payload += _i32(point[0]) + _i32(point[1]) + _i32(point[2])
                payload += bytes([point[3] & 0xFF, point[4] & 0xFF])
        self.request(MSG_AUDIO_CONFIGURE, bytes(payload))

    def audio_start(self) -> None:
        self.request(MSG_AUDIO_START, b"")

    def audio_stop(self) -> None:
        self.request(MSG_AUDIO_STOP, b"", timeout=3.0)

    def audio_data(self, levels: bytes, sequence: int) -> None:
        self.send_no_reply(MSG_AUDIO_DATA, levels, sequence)

    def get_audio_status(self) -> AudioStatus:
        body = self.request(MSG_AUDIO_STATUS, b"", MSG_AUDIO_STATUS)
        return AudioStatus(
            state=body[0], flags=body[1],
            ring_fill=struct.unpack_from("<H", body, 2)[0],
            ring_capacity=struct.unpack_from("<H", body, 4)[0],
            prebuffer=struct.unpack_from("<H", body, 6)[0],
            underrun_count=struct.unpack_from("<I", body, 8)[0],
            overrun_count=struct.unpack_from("<I", body, 12)[0],
            packet_loss_count=struct.unpack_from("<I", body, 16)[0],
            rendered_samples=struct.unpack_from("<I", body, 20)[0],
            clock_correction_ppm=struct.unpack_from("<i", body, 24)[0],
            max_service_us=struct.unpack_from("<I", body, 28)[0],
        )

    def set_demo(self, demo_id: int) -> None:
        self.request(MSG_SET_DEMO, bytes([demo_id]))

    def stop_plan(self) -> None:
        self.request(MSG_STOP_PLAN, b"")

    def clear_plan(self) -> None:
        self.request(MSG_CLEAR_PLAN, b"")


def make_path(kind: str, points: int = 120, size_mm: float = 12.0,
              height_mm: float = 70.0, z_spread_mm: float = 10.0):
    """Small reference path generator matching UMH Demo shape semantics."""
    import math

    out = []
    for index in range(points):
        t = index / max(1, points)
        if kind == "circle":
            x, y, z = size_mm * math.cos(2 * math.pi * t), size_mm * math.sin(2 * math.pi * t), height_mm
        elif kind == "helix":
            x = size_mm * math.cos(2 * math.pi * 3 * t)
            y = size_mm * math.sin(2 * math.pi * 3 * t)
            z = height_mm - z_spread_mm / 2 + z_spread_mm * t
        elif kind == "lissajous":
            x = size_mm * math.sin(2 * math.pi * 1.5 * t + math.pi / 4)
            y = size_mm * math.sin(2 * math.pi * t)
            z = height_mm + z_spread_mm * 0.5 * math.sin(4 * math.pi * t)
        elif kind == "sphere":
            polar = math.pi * t
            radial = size_mm * math.sin(polar)
            x = radial * math.cos(2 * math.pi * 3 * t)
            y = radial * math.sin(2 * math.pi * 3 * t)
            z = height_mm + size_mm * math.cos(polar)
        else:
            raise ValueError(kind)
        out.append((round(x * 1000), round(y * 1000), round(z * 1000), 200, (index * 15) // max(1, points)))
    return out


