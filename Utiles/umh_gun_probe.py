"""Gun-trigger state probe: dump the live umh_gun_trigger_t over SWD.

The PA9 shot latch lives in one static struct, so halting the core and reading
that RAM block is enough to see exactly which envelope state the device is in,
what level it is emitting, and whether the render task is still submitting.

This tool only observes.  To exercise the trigger without a physical switch
(synthetic press, re-press during the fade, release-deadline watchdog) use
Utiles/gun_probe.tcl, which drives the whole sequence inside one OpenOCD
session.

Usage:
  python Utiles/umh_gun_probe.py                    # one dump
  python Utiles/umh_gun_probe.py --watch 40 --interval 0.25
  python Utiles/umh_gun_probe.py --watch 60 --interval 0.1 --csv run.csv
"""
import argparse
import os
import struct
import subprocess
import sys
import tempfile
import time

OPENOCD = os.environ.get("OPENOCD", r"D:\Software\OpenOCD\bin\openocd.exe")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, "openocd.cfg")
DUMP_TCL = os.path.join(ROOT, "Utiles", "dump_ram.tcl")

GUN_ADDR = 0x20005DA0

ENV_STATE = {0: "IDLE", 1: "RAMP_UP", 2: "FULL", 3: "RAMP_DOWN"}

# umh_gun_trigger_t (arm-none-eabi-gdb "ptype /o umh_gun_trigger_t")
#   +0   pressed, engaged, refresh_pending, raw, stable_samples  (5 x u8)
#   +8   fires u32, deferred u32
#   +16  last_submit_ms u32, ramp_start_ms u32, sequence u32
#   +28  envelope_state, current_level
#   +30  base_level[84]
#   +114 base_ready, envelope_sent, reserved u16
#   +118 umh_output_frame_t (232 B), channels at +20, rgb at +188
FRAME_OFF = 118
BASE_LEVEL_OFF = 30
BASE_READY_OFF = 114
ENVELOPE_SENT_OFF = 115
CHAN_OFF = FRAME_OFF + 20
RGB_OFF = FRAME_OFF + 188
DUMP_LEN = 320


def _openocd(script, timeout=90):
    result = subprocess.run(
        [OPENOCD, "-f", CFG, "-f", DUMP_TCL, "-c", script],
        capture_output=True, text=True, timeout=timeout,
    )
    return result.stdout + result.stderr


def dump(addr, length, path):
    if os.path.exists(path):
        os.remove(path)
    target = path.replace("\\", "/")
    out = _openocd(f"umh_dump {hex(addr)} {length} {target}")
    if not os.path.exists(path):
        raise RuntimeError(f"openocd dump failed:\n{out}")
    with open(path, "rb") as handle:
        return handle.read()


def decode(raw):
    pressed, engaged, refresh_pending, rawpin, stable = struct.unpack_from("<5B", raw, 0)
    fires, deferred = struct.unpack_from("<II", raw, 8)
    last_submit, ramp_start, sequence = struct.unpack_from("<III", raw, 16)
    env_state, cur_level = struct.unpack_from("<2B", raw, 28)
    base_ready = raw[BASE_READY_OFF]
    env_sent = raw[ENVELOPE_SENT_OFF]
    base0 = raw[BASE_LEVEL_OFF]
    chan = [raw[CHAN_OFF + 2 * i + 1] for i in range(84)]
    return dict(
        pressed=pressed, engaged=engaged, pending=refresh_pending, rawpin=rawpin,
        stable=stable, fires=fires, deferred=deferred,
        last_submit=last_submit, ramp_start=ramp_start, sequence=sequence,
        env=env_state, level=cur_level, base_ready=base_ready, sent=env_sent,
        base0=base0, nchan=sum(1 for v in chan if v > 0), vmax=max(chan),
        rgb_r=raw[RGB_OFF], rgb_g=raw[RGB_OFF + 1], rgb_b=raw[RGB_OFF + 2],
    )


HDR = (f"{'time':>8} {'press':>5} {'eng':>3} {'bl':>2} {'env':>9} {'lvl':>4} "
       f"{'sent':>4} {'base':>4} {'fires':>6} {'defer':>5} {'seq':>6} {'n>0':>4} "
       f"{'vmax':>4} {'rgbR':>4} {'ramp_age':>8}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--watch", type=int, default=0)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--csv")
    args = parser.parse_args()

    samples = 1 if args.watch == 0 else args.watch
    tmp = os.path.join(tempfile.gettempdir(), "umh_gun_probe.bin")
    rows = []
    print(HDR)
    for _ in range(samples):
        state = decode(dump(GUN_ADDR, DUMP_LEN, tmp))
        now = time.strftime("%H:%M:%S")
        rows.append((now, state))
        print(f"{now:>8} {state['pressed']:>5} {state['engaged']:>3} "
              f"{state['base_ready']:>2} {ENV_STATE.get(state['env'], state['env']):>9} "
              f"{state['level']:>4} {state['sent']:>4} {state['base0']:>4} "
              f"{state['fires']:>6} "
              f"{state['deferred']:>5} {state['sequence']:>6} {state['nchan']:>4} "
              f"{state['vmax']:>4} {state['rgb_r']:>4} {state['ramp_start']:>8}")
        if samples > 1:
            time.sleep(args.interval)

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as handle:
            handle.write("time," + ",".join(rows[0][1].keys()) + "\n")
            for stamp, state in rows:
                handle.write(stamp + "," + ",".join(str(v) for v in state.values()) + "\n")
        print(f"wrote {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
