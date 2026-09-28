"""Read the EEPROM profile, clear every per-channel phase correction and commit.

gain[], enabled[], RGB and calibration metadata are kept unchanged.  The
record CRC/commit words are recomputed by the firmware on commit.

    python Utiles/umh_eeprom_phase_zero.py [--port COM4] [--dry-run]
"""
from __future__ import annotations

import argparse
import struct
import sys

from umh_cal_probe import UmhProbe

MSG_EEPROM_READ, MSG_EEPROM_WRITE, MSG_EEPROM_COMMIT = 0x40, 0x41, 0x42
MSG_ACK, MSG_NACK = 0x70, 0x71
CHANNELS = 84
PHASE_OFFSET = 12  # magic u32, version u16, payload_length u16, generation u32


def request(p: UmhProbe, mtype: int, payload: bytes = b"", timeout: float = 3.0):
    tid = p._send(mtype, payload)
    return p._recv(tid, timeout)


def read_record(p: UmhProbe) -> bytes:
    mtype, _, _, data = request(p, MSG_EEPROM_READ)
    if mtype != MSG_EEPROM_READ:
        raise RuntimeError("EEPROM_READ failed, type 0x%02x" % mtype)
    return data


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    with UmhProbe(a.port) as p:
        rec = bytearray(read_record(p))
        phase = struct.unpack_from("<%dH" % CHANNELS, rec, PHASE_OFFSET)
        nz = sum(1 for v in phase if v >> 8)
        print("record %d bytes, non-zero phase bytes: %d" % (len(rec), nz))
        struct.pack_into("<%dH" % CHANNELS, rec, PHASE_OFFSET, *([0] * CHANNELS))
        if a.dry_run:
            return 0
        for mtype, payload in ((MSG_EEPROM_WRITE, bytes(rec)), (MSG_EEPROM_COMMIT, b"")):
            rt, _, _, data = request(p, mtype, payload)
            if rt != MSG_ACK:
                raise RuntimeError("0x%02x rejected: type 0x%02x status %s" % (mtype, rt, data[:1].hex()))
        back = read_record(p)
        left = sum(1 for v in struct.unpack_from("<%dH" % CHANNELS, back, PHASE_OFFSET) if v)
        same = back[PHASE_OFFSET + 2 * CHANNELS:PHASE_OFFSET + 3 * CHANNELS] == \
            bytes(rec[PHASE_OFFSET + 2 * CHANNELS:PHASE_OFFSET + 3 * CHANNELS])
        print("committed: non-zero phase words %d, gain unchanged %s" % (left, same))
        return 0 if left == 0 and same else 1


if __name__ == "__main__":
    sys.exit(main())
