"""Focused-AM audio bench: stream a known envelope, record the carrier, demodulate.

Bypasses every host DSP stage.  The script speaks the V7 audio protocol
(0x90..0x94) directly, paces a synthetic envelope at exactly the configured
rate against the wall clock, records the PC microphone at 96 kHz with ffmpeg
and recovers the emitted envelope |z(t)| by complex demodulation of the
40 kHz carrier.  The recovered envelope is compared with what was sent:

  * tone frequency (pitch / device rate check, ppm)
  * harmonic distortion and noise floor of the recovered envelope
  * device status (ring, underruns, clock correction)

  python Utiles/umh_audio_bench.py --case tone1k --secs 5
  python Utiles/umh_audio_bench.py --case dc64,tone1k,tone3k --secs 4 --save x.npz
"""
import argparse
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np
import serial

FS = 96000
MIC = "Microphone (Realtek High Definition Audio)"
RATE = 20000
PKT = 256
PREBUF = 1536


class V7Audio:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.ser.reset_input_buffer()
        self.tid = 100
        self.lock = threading.Lock()
        self.frames = []
        self.cv = threading.Condition()
        self.alive = True
        self.rx = threading.Thread(target=self._reader, daemon=True)
        self.rx.start()

    def _reader(self):
        buf = b""
        while self.alive:
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                return
            while len(buf) >= 16:
                i = buf.find(b"\x55\xaa")
                if i < 0:
                    buf = buf[-1:]
                    break
                buf = buf[i:]
                if len(buf) < 16:
                    break
                mtype, flags = buf[3], buf[4]
                plen, tid = struct.unpack("<HI", buf[6:12])
                if len(buf) < 16 + plen:
                    break
                with self.cv:
                    self.frames.append((tid, mtype, flags, buf[16:16 + plen]))
                    self.cv.notify_all()
                buf = buf[16 + plen:]

    def request(self, mtype, payload=b"", timeout=2.0):
        self.tid += 1
        tid = self.tid
        hdr = struct.pack("<BBBBBBHII", 0x55, 0xAA, 7, mtype, 1, 16, len(payload), tid, 0)
        with self.lock:
            self.ser.write(hdr + payload)
        deadline = time.time() + timeout
        with self.cv:
            while time.time() < deadline:
                for f in self.frames:
                    if f[0] == tid:
                        self.frames.remove(f)
                        return f
                self.cv.wait(0.05)
        raise TimeoutError("no response to 0x%02X" % mtype)

    def send_audio(self, data, seq):
        hdr = struct.pack("<BBBBBBHII", 0x55, 0xAA, 7, 0x92, 0, 16, len(data), 0, seq)
        with self.lock:
            self.ser.write(hdr + data)

    def status(self):
        _, mtype, _, p = self.request(0x94)
        (st, fl, fill, cap, pre, und, ovr, loss, rend, ppm, svc) = struct.unpack("<BBHHHIIIIiI", p)
        return dict(state=st, fill=fill, und=und, ovr=ovr, loss=loss, rendered=rend,
                    ppm=ppm, svc=svc)

    def close(self):
        self.alive = False
        time.sleep(0.1)
        self.ser.close()


def make_case(name, secs):
    n = int(secs * RATE)
    t = np.arange(n) / RATE
    if name.startswith("dc"):
        env = np.full(n, float(name[2:]))
    elif name.startswith("tone"):
        f = float(name[4:].replace("k", "")) * (1000 if "k" in name else 1)
        env = 64 + 40 * np.sin(2 * np.pi * f * t)
    elif name == "sweep":
        f0, f1 = 200.0, 8000.0
        k = np.log(f1 / f0) / secs
        env = 64 + 40 * np.sin(2 * np.pi * f0 * (np.exp(k * t) - 1) / k)
    else:
        raise ValueError(name)
    return np.clip(np.round(env), 0, 128).astype(np.uint8)


def stream(dev, env, stat_log):
    """Pace the envelope against the wall clock, keeping PREBUF samples ahead."""
    seq = 0
    sent = 0
    t0 = time.perf_counter()
    last_stat = t0
    # Initial prebuffer burst.
    while sent < PREBUF + PKT and sent < len(env):
        dev.send_audio(env[sent:sent + PKT].tobytes(), seq)
        seq += 1
        sent += PKT
    t0 = time.perf_counter()
    while sent < len(env):
        target = PREBUF + (time.perf_counter() - t0) * RATE
        if sent < target:
            dev.send_audio(env[sent:sent + PKT].tobytes(), seq)
            seq += 1
            sent += PKT
        else:
            time.sleep(0.002)
        now = time.perf_counter()
        if now - last_stat > 0.5:
            last_stat = now
            try:
                s = dev.status()
                s["t"] = now - t0
                stat_log.append(s)
            except TimeoutError as e:
                stat_log.append({"t": now - t0, "error": str(e)})


def record_start(secs, path):
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "dshow",
           "-sample_rate", str(FS), "-sample_size", "16", "-channels", "1",
           "-audio_buffer_size", "50", "-i", "audio=" + MIC, "-t", str(secs),
           "-f", "s16le", "-acodec", "pcm_s16le", path]
    return subprocess.Popen(cmd, stdin=subprocess.DEVNULL)


def lowpass_taps(cut, fs, n=801):
    t = np.arange(n) - (n - 1) / 2
    h = np.sinc(2 * cut / fs * t) * np.blackman(n)
    return h / h.sum()


def demod(x):
    n = 1 << 20 if len(x) >= 1 << 20 else 1 << int(np.log2(len(x)))
    X = np.abs(np.fft.rfft(x[:n] * np.hanning(n)))
    f = np.fft.rfftfreq(n, 1 / FS)
    m = (f > 39000) & (f < 41000)
    fc = f[m][np.argmax(X[m])]
    t = np.arange(len(x)) / FS
    z = np.convolve(x * np.exp(-2j * np.pi * fc * t), lowpass_taps(9000, FS), mode="same")
    return fc, np.abs(z)[2000:-2000]


def spectrum(v, fs):
    v = v - v.mean()
    n = len(v)
    w = np.blackman(n)
    X = np.abs(np.fft.rfft(v * w)) / (np.sum(w) / 2)
    return np.fft.rfftfreq(n, 1 / fs), X


def analyse(name, x, env):
    rms_clip = np.mean(np.abs(x) > 32000)
    fc, a = demod(x)
    f, A = spectrum(a, FS)
    res = {"name": name, "fc": fc, "clip": rms_clip, "mean": a.mean()}
    print("\n=== %s  fc=%.2f Hz  clip=%.3f%%  env mean=%.0f" % (name, fc, rms_clip * 100, a.mean()))
    if name.startswith("tone"):
        ft = float(name[4:].replace("k", "")) * (1000 if "k" in name else 1)
        m = (f > ft * 0.97) & (f < ft * 1.03)
        i = np.argmax(np.where(m, A, 0))
        # parabolic peak interpolation on log magnitude
        la, lb, lc = np.log(A[i - 1:i + 2] + 1e-12)
        d = 0.5 * (la - lc) / (la - 2 * lb + lc)
        fpk = f[i] + d * (f[1] - f[0])
        fund = A[i]
        harm = []
        for k in range(2, 6):
            mk = (f > k * ft * 0.97) & (f < k * ft * 1.03)
            harm.append(A[mk].max() if mk.any() and k * ft < 9000 else 0)
        # noise: everything in 100..8000 except +-3 % windows around fundamental/harmonics
        nm = (f > 100) & (f < 8000)
        for k in range(1, 6):
            nm &= ~((f > k * ft * 0.97) & (f < k * ft * 1.03))
        noise = np.sqrt(np.sum(A[nm] ** 2) / 2)  # rms from amplitude spectrum bins
        res.update(fpk=fpk, fund=fund, harm=harm, noise=noise)
        print("  tone: sent %.1f Hz  recovered %.2f Hz  (%+.0f ppm)" % (ft, fpk, (fpk / ft - 1) * 1e6))
        print("  fundamental %.1f  |  H2..H5 %s dBc" % (
            fund, " ".join("%.1f" % (20 * np.log10(h / fund + 1e-12)) for h in harm)))
        print("  in-band noise (100..8000 Hz, excl. harmonics) %.1f dBc" % (20 * np.log10(noise / (fund / np.sqrt(2)) + 1e-12)))
    else:
        nm = (f > 100) & (f < 8000)
        noise = np.sqrt(np.sum(A[nm] ** 2) / 2)
        res.update(noise=noise)
        print("  envelope noise 100..8000 Hz: %.2f  (%.1f dB re mean)" % (noise, 20 * np.log10(noise / a.mean() + 1e-12)))
    for lo, hi in [(100, 1000), (1000, 3000), (3000, 6000), (6000, 9000)]:
        mb = (f >= lo) & (f < hi)
        print("   band %5d..%5d  %.1f dB" % (lo, hi, 20 * np.log10(np.sqrt(np.sum(A[mb] ** 2) / 2) + 1e-12)))
    return res, a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--case", default="dc64,tone1k")
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--z", type=int, default=1_000_000)
    ap.add_argument("--x", type=int, default=0, help="focus x offset (um), steers SPL off the mic")
    ap.add_argument("--level", type=int, default=255, help="config level 0..255 (envelope ceiling)")
    ap.add_argument("--save", default="")
    a = ap.parse_args()
    dev = V7Audio(a.port)
    tmp = tempfile.mkdtemp(prefix="umhaudio_")
    saved = {}
    try:
        for case in a.case.split(","):
            try:
                dev.request(0x22)
                dev.request(0x23)
            except TimeoutError:
                pass
            cfg = struct.pack("<iiiBBHHH", a.x, 0, a.z, 0, a.level, RATE, PREBUF, 0)
            r = dev.request(0x90, cfg)
            if r[1] != 0x70:
                print("configure rejected", r)
                return 1
            dev.request(0x91)
            env = make_case(case, a.secs + 1.5)
            path = os.path.join(tmp, case + ".raw")
            stats = []
            th = threading.Thread(target=stream, args=(dev, env, stats))
            th.start()
            time.sleep(0.6)
            rec = record_start(a.secs, path)
            rec.wait()
            th.join()
            time.sleep(0.2)
            s_end = dev.status()
            dev.request(0x93)
            time.sleep(0.2)
            x = np.fromfile(path, dtype="<i2").astype(np.float64)[int(0.3 * FS):]
            res, env_rec = analyse(case, x, env)
            errs = [s for s in stats if "error" in s]
            ok = [s for s in stats if "error" not in s]
            if ok:
                print("  device: fill %s  ppm %s  und %d ovr %d loss %d svc %d us  status timeouts %d" % (
                    [s["fill"] for s in ok[::2]], [s["ppm"] for s in ok[::2]],
                    s_end["und"], s_end["ovr"], s_end["loss"], s_end["svc"], len(errs)))
                if len(ok) >= 2:
                    dr = (ok[-1]["rendered"] - ok[0]["rendered"]) / (ok[-1]["t"] - ok[0]["t"])
                    print("  device tick rate %.1f Hz (%+.0f ppm)" % (dr, (dr / RATE - 1) * 1e6))
            saved[case] = x.astype(np.int16)
    finally:
        try:
            dev.request(0x93)
        except Exception:
            pass
        dev.close()
        for fn in os.listdir(tmp):
            os.remove(os.path.join(tmp, fn))
        os.rmdir(tmp)
    if a.save:
        np.savez_compressed(a.save, **saved)
    return 0


if __name__ == "__main__":
    sys.exit(main())
