"""Acoustic purity probe: PC microphone at 96 kHz vs a static UMH output.

Each case is recorded with ffmpeg (dshow, raw 16-bit mono 96 kHz), then
analysed with a Welch PSD.  The linear quantity that tells whether the
emitted carrier itself is noisy is the *noise skirt* around 40 kHz,
expressed in dBc/Hz relative to the carrier (no mic nonlinearity needed).
The audio band (200 Hz..16 kHz) is reported against the TX-off reference;
excess there can be real demodulated sound or the mic's own
nonlinearity, so it is interpreted together with the skirt.

  python Utiles/umh_noise_probe.py --cases off,all128,single41_128 --secs 4
"""
import argparse
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from umh_link import UmhLink, CHANNELS  # noqa: E402

FS = 96000
MIC = "Microphone (Realtek High Definition Audio)"


def record(secs, path):
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "dshow",
           "-sample_rate", str(FS), "-sample_size", "16", "-channels", "1",
           "-audio_buffer_size", "50", "-i", "audio=" + MIC, "-t", str(secs),
           "-f", "s16le", "-acodec", "pcm_s16le", path]
    subprocess.run(cmd, check=True, stdin=subprocess.DEVNULL)
    x = np.fromfile(path, dtype="<i2").astype(np.float64)
    return x[int(0.3 * FS):]  # drop device start-up


def welch(x, nfft=16384):
    win = np.hanning(nfft)
    step = nfft // 2
    segs = [x[i:i + nfft] for i in range(0, len(x) - nfft, step)]
    acc = np.zeros(nfft // 2 + 1)
    for s in segs:
        s = s - s.mean()
        acc += np.abs(np.fft.rfft(s * win)) ** 2
    psd = acc / len(segs) / (FS * np.sum(win ** 2))  # one-sided scale below
    psd[1:-1] *= 2
    f = np.fft.rfftfreq(nfft, 1 / FS)
    return f, psd


def band(f, psd, lo, hi):
    m = (f >= lo) & (f < hi)
    return np.sum(psd[m]) * (f[1] - f[0])


def db(v):
    return 10 * np.log10(max(v, 1e-30))


def analyse(name, x, ref=None):
    f, p = welch(x)
    fc_idx = np.argmax(np.where((f > 38000) & (f < 42000), p, 0))
    fc = f[fc_idx]
    carrier = band(f, p, fc - 30, fc + 30)
    clip = np.mean(np.abs(x) > 32000)
    out = {"name": name, "x": x, "rms": np.sqrt(np.mean(x ** 2)), "clip": clip,
           "fc": fc, "carrier_db": db(carrier), "f": f, "p": p}
    # noise skirt: bands offset from the carrier (both sides), dBc/Hz
    skirt = {}
    for lo, hi in [(100, 300), (300, 1000), (1000, 3000), (3000, 8000)]:
        pw = band(f, p, fc + lo, fc + hi) + band(f, p, fc - hi, fc - lo)
        skirt[(lo, hi)] = pw
    out["skirt"] = skirt
    audio = {}
    for lo, hi in [(200, 1000), (1000, 4000), (4000, 8000), (8000, 16000), (16000, 30000)]:
        audio[(lo, hi)] = band(f, p, lo, hi)
    out["audio"] = audio
    return out


def report(res, ref):
    print("\n=== %s  rms=%.0f  clip=%.4f%%  fc=%.1f Hz  carrier=%.1f dB" % (
        res["name"], res["rms"], res["clip"] * 100, res["fc"], res["carrier_db"]))
    car = 10 ** (res["carrier_db"] / 10)
    print("  skirt (both sides)  |  dBc/Hz   | excess over OFF")
    for k, v in res["skirt"].items():
        bw = 2 * (k[1] - k[0])
        r = ref["skirt"][k] if ref else 0
        print("   +-%5d..%5d Hz   | %7.1f   | %+6.1f dB" % (
            k[0], k[1], db(v / bw) - db(car), db(v) - db(r) if ref else 0))
    print("  audio band         |  dB (abs) | excess over OFF")
    for k, v in res["audio"].items():
        r = ref["audio"][k] if ref else 0
        print("   %5d..%5d Hz     | %7.1f   | %+6.1f dB" % (
            k[0], k[1], db(v), db(v) - db(r) if ref else 0))


def apply_case(link, case):
    if case == "off":
        link.stop()
        return
    kind, _, rest = case.partition("_")
    if kind == "all":
        link.emit_channels([0] * CHANNELS, [int(rest)] * CHANNELS)
    elif kind.startswith("single"):
        ch = int(kind[6:])
        lv = [0] * CHANNELS
        lv[ch] = int(rest)
        link.emit_channels([0] * CHANNELS, lv)
    elif kind == "point":
        link.emit_point(0, 0, int(rest.split("_")[0]) if rest else 100000, 255)
    else:
        raise ValueError(case)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", default="off,all_128")
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--save", default="")
    a = ap.parse_args()
    link = UmhLink("COM4")
    tmp = tempfile.mkdtemp(prefix="umhprobe_")
    results = {}
    try:
        for case in a.cases.split(","):
            apply_case(link, case)
            time.sleep(0.5)
            x = record(a.secs, os.path.join(tmp, case + ".raw"))
            results[case] = analyse(case, x)
        link.stop()
    finally:
        link.close()
    ref = results.get("off")
    for r in results.values():
        report(r, ref)
    if a.save:
        np.savez_compressed(a.save, **{k: v["x"].astype(np.int16) for k, v in results.items()})
    for fn in os.listdir(tmp):
        os.remove(os.path.join(tmp, fn))
    os.rmdir(tmp)


if __name__ == "__main__":
    main()
