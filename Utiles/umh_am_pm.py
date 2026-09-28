"""Split a recorded 40 kHz carrier into AM and PM noise spectra.

Input: .npz written by umh_noise_probe.py --save (int16 96 kHz captures).
For every case the carrier is mixed to complex baseband, low-pass filtered
(+-8 kHz) and decimated to 24 kHz.  Then
  a(t)   = |z|/mean|z| - 1         fractional amplitude noise (AM)
  phi(t) = unwrap(arg z) detrended phase noise in rad (PM)
PSDs are single-sided, in dB re 1/Hz (== dBc/Hz for small modulation).
The "floor" column is the TX-off capture pushed through the same
demodulator, normalised to the case's carrier: additive (mic/room) noise
appears at that level in *both* AM and PM, so anything above it is real.

  python Utiles/umh_am_pm.py capture.npz
"""
import sys

import numpy as np

FS = 96000
DEC = 4
FSB = FS // DEC
BANDS = [(20, 100), (100, 300), (300, 1000), (1000, 3000), (3000, 8000)]


def lowpass_taps(cut, fs, n=801):
    t = np.arange(n) - (n - 1) / 2
    h = np.sinc(2 * cut / fs * t) * np.blackman(n)
    return h / h.sum()


H = lowpass_taps(8000, FS)


def baseband(x, fc):
    t = np.arange(len(x)) / FS
    z = x * np.exp(-2j * np.pi * fc * t)
    z = np.convolve(z, H, mode="valid")[::DEC]
    return z


def refine_fc(x):
    n = 1 << 20 if len(x) >= 1 << 20 else 1 << int(np.log2(len(x)))
    X = np.abs(np.fft.rfft(x[:n] * np.hanning(n)))
    f = np.fft.rfftfreq(n, 1 / FS)
    m = (f > 38000) & (f < 42000)
    fc = f[m][np.argmax(X[m])]
    z = baseband(x, fc)
    ph = np.unwrap(np.angle(z))
    k = np.polyfit(np.arange(len(ph)) / FSB, ph, 1)[0]
    return fc + k / (2 * np.pi)


def psd(v, nfft=4096):
    win = np.hanning(nfft)
    segs = [v[i:i + nfft] for i in range(0, len(v) - nfft, nfft // 2)]
    acc = np.zeros(nfft // 2 + 1)
    for s in segs:
        s = s - s.mean()
        acc += np.abs(np.fft.rfft(s * win)) ** 2
    p = acc / len(segs) / (FSB * np.sum(win ** 2))
    p[1:-1] *= 2
    return np.fft.rfftfreq(nfft, 1 / FSB), p


def band_db(f, p, lo, hi):
    m = (f >= lo) & (f < hi)
    return 10 * np.log10(np.mean(p[m]) + 1e-30)


def main(path):
    d = np.load(path)
    off = d["off"].astype(float) if "off" in d.files else None
    for k in d.files:
        if k == "off":
            continue
        x = d[k].astype(float)
        fc = refine_fc(x)
        z = baseband(x, fc)[200:-200]
        mag = np.abs(z)
        a = mag / mag.mean() - 1
        ph = np.unwrap(np.angle(z))
        n = np.arange(len(ph))
        ph = ph - np.polyval(np.polyfit(n, ph, 2), n)
        fa, pa = psd(a)
        fp, pp = psd(ph)
        floor = None
        if off is not None:
            zo = baseband(off, fc)[200:-200]
            ff, pf = psd(zo.real / mag.mean())
            floor = pf  # additive noise per quadrature, rel. carrier
        print("\n=== %-14s fc=%.3f Hz  carrier=%.1f dBFS  AMrms=%.2e  PMrms=%.2e rad" % (
            k, fc, 20 * np.log10(mag.mean() / 32768 * np.sqrt(0.5) + 1e-30),
            np.std(a), np.std(ph)))
        print("   band Hz      AM dBc/Hz   PM dBrad2/Hz   floor dBc/Hz   AM-floor")
        for lo, hi in BANDS:
            am = band_db(fa, pa, lo, hi)
            pm = band_db(fp, pp, lo, hi)
            fl = band_db(ff, floor, lo, hi) if floor is not None else float("nan")
            print("   %5d-%-5d   %8.1f     %8.1f       %8.1f      %+6.1f" % (lo, hi, am, pm, fl, am - fl))


if __name__ == "__main__":
    main(sys.argv[1])
