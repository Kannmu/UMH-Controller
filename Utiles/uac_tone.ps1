# Render a test tone (or a WAV file) to a named WASAPI endpoint.
# Usage: pwsh -File Utiles/uac_tone.ps1 -Match 'UMH' -Seconds 3 -Freq 440
#        pwsh -File Utiles/uac_tone.ps1 -Match 'UMH' -Seconds 10 -Wav 'C:\Windows\Media\Speech On.wav'
param(
  [string]$Match = 'UMH',
  [int]$Seconds = 3,
  [double]$Freq = 440.0,
  [double]$Amplitude = 0.5,
  [string]$Wav = ''
)
$ErrorActionPreference = 'Stop'

$cs = @'
using System;
using System.IO;
using System.Runtime.InteropServices;

public static class WasapiTone
{
    [ComImport, Guid("BCDE0395-E52F-467C-8E3D-C4579291692E")]
    private class MMDeviceEnumerator { }

    [ComImport, Guid("A95664D2-9614-4F35-A746-DE8DB63617E6"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    private interface IMMDeviceEnumerator
    {
        int EnumAudioEndpoints(int dataFlow, int stateMask, out IMMDeviceCollection devices);
        int GetDefaultAudioEndpoint(int dataFlow, int role, out IMMDevice device);
        int GetDevice(string id, out IMMDevice device);
    }

    [ComImport, Guid("0BD7A1BE-7A1A-44DB-8397-CC5392387B5E"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    private interface IMMDeviceCollection
    {
        int GetCount(out int count);
        int Item(int index, out IMMDevice device);
    }

    [ComImport, Guid("D666063F-1587-4E43-81F1-B948E807363F"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    private interface IMMDevice
    {
        int Activate(ref Guid id, int clsCtx, IntPtr activationParams, [MarshalAs(UnmanagedType.IUnknown)] out object iface);
        int OpenPropertyStore(int access, out IPropertyStore properties);
        int GetId([MarshalAs(UnmanagedType.LPWStr)] out string id);
        int GetState(out int state);
    }

    [ComImport, Guid("886D8EEB-8CF2-4446-8D02-CDBA1DBDCF99"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    private interface IPropertyStore
    {
        int GetCount(out int count);
        int GetAt(int index, out PROPERTYKEY key);
        int GetValue(ref PROPERTYKEY key, out PROPVARIANT value);
        int SetValue(ref PROPERTYKEY key, ref PROPVARIANT value);
        int Commit();
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct PROPERTYKEY { public Guid fmtid; public int pid; }

    [StructLayout(LayoutKind.Explicit)]
    private struct PROPVARIANT
    {
        [FieldOffset(0)] public short vt;
        [FieldOffset(8)] public IntPtr pointerValue;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct WAVEFORMATEX
    {
        public ushort wFormatTag;
        public ushort nChannels;
        public uint nSamplesPerSec;
        public uint nAvgBytesPerSec;
        public ushort nBlockAlign;
        public ushort wBitsPerSample;
        public ushort cbSize;
    }

    [ComImport, Guid("1CB9AD4C-DBFA-4c32-B178-C2F568A703B2"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    private interface IAudioClient
    {
        int Initialize(int shareMode, int streamFlags, long bufferDuration, long periodicity, IntPtr format, IntPtr sessionGuid);
        int GetBufferSize(out uint frames);
        int GetStreamLatency(out long latency);
        int GetCurrentPadding(out uint padding);
        int IsFormatSupported(int shareMode, IntPtr format, out IntPtr closest);
        int GetMixFormat(out IntPtr format);
        int GetDevicePeriod(out long defaultPeriod, out long minimumPeriod);
        int Start();
        int Stop();
        int Reset();
        int SetEventHandle(IntPtr handle);
        int GetService(ref Guid id, [MarshalAs(UnmanagedType.IUnknown)] out object iface);
    }

    [ComImport, Guid("F294ACFC-3146-4483-A7BF-ADDCA7C260E2"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    private interface IAudioRenderClient
    {
        int GetBuffer(uint frames, out IntPtr data);
        int ReleaseBuffer(uint frames, int flags);
    }

    private static readonly Guid IID_IAudioClient = new Guid("1CB9AD4C-DBFA-4c32-B178-C2F568A703B2");
    private static readonly Guid IID_IAudioRenderClient = new Guid("F294ACFC-3146-4483-A7BF-ADDCA7C260E2");
    private static readonly PROPERTYKEY PKEY_Device_FriendlyName =
        new PROPERTYKEY { fmtid = new Guid("a45c254e-df1c-4efd-8020-67d146a850e0"), pid = 14 };

    /* Load a 16-bit PCM WAV, mix to mono, resample to 48 kHz and normalise to
     * the requested peak.  Linear interpolation is enough for a bench signal:
     * the point is to drive the device with real programme material, not to
     * measure the resampler. */
    private static float[] LoadWav48kMono(string path, double amplitude)
    {
        using (var br = new BinaryReader(File.OpenRead(path)))
        {
            if (new string(br.ReadChars(4)) != "RIFF") throw new InvalidDataException("not RIFF");
            br.ReadInt32();
            if (new string(br.ReadChars(4)) != "WAVE") throw new InvalidDataException("not WAVE");
            int channels = 1, rate = 48000, bits = 16;
            long dataOffset = 0; int dataLength = 0;
            while (br.BaseStream.Position + 8 <= br.BaseStream.Length)
            {
                string id = new string(br.ReadChars(4));
                int length = br.ReadInt32();
                long next = br.BaseStream.Position + length + (length & 1);
                if (id == "fmt ")
                {
                    br.ReadUInt16(); channels = br.ReadUInt16(); rate = br.ReadInt32();
                    br.ReadInt32(); br.ReadUInt16(); bits = br.ReadUInt16();
                }
                else if (id == "data")
                {
                    dataOffset = br.BaseStream.Position; dataLength = length;
                }
                br.BaseStream.Position = next;
            }
            if (dataOffset == 0 || dataLength == 0 || bits != 16 || channels < 1)
                throw new InvalidDataException("unsupported WAV");
            br.BaseStream.Position = dataOffset;
            int frames = dataLength / (2 * channels);
            var mono = new float[frames];
            double peak = 0.0;
            for (int i = 0; i < frames; i++)
            {
                double sum = 0.0;
                for (int c = 0; c < channels; c++) sum += br.ReadInt16() / 32768.0;
                mono[i] = (float)(sum / channels);
                double a = Math.Abs(mono[i]);
                if (a > peak) peak = a;
            }
            int outFrames = (int)((long)frames * 48000 / rate);
            var output = new float[outFrames];
            double scale = peak > 1e-9 ? amplitude / peak : 0.0;
            for (int i = 0; i < outFrames; i++)
            {
                double src = (double)i * rate / 48000.0;
                int i0 = (int)src;
                double frac = src - i0;
                int i1 = i0 + 1 < frames ? i0 + 1 : i0;
                double v = mono[i0] * (1.0 - frac) + mono[i1] * frac;
                output[i] = (float)(v * scale);
            }
            return output;
        }
    }

    public static string Play(string match, int seconds, double freq, double amplitude,
                              string wavPath, out string detail)
    {
        var enumerator = (IMMDeviceEnumerator)new MMDeviceEnumerator();
        IMMDeviceCollection collection;
        enumerator.EnumAudioEndpoints(0 /*eRender*/, 1 /*ACTIVE*/, out collection);
        int count;
        collection.GetCount(out count);
        IMMDevice target = null;
        string targetId = null, targetName = null;
        for (int i = 0; i < count; i++)
        {
            IMMDevice dev;
            collection.Item(i, out dev);
            IPropertyStore store;
            dev.OpenPropertyStore(0, out store);
            PROPVARIANT value;
            var key = PKEY_Device_FriendlyName;
            store.GetValue(ref key, out value);
            string name = Marshal.PtrToStringUni(value.pointerValue);
            detail = "endpoint " + i + ": " + name;
            if (name != null && name.ToLowerInvariant().Contains(match.ToLowerInvariant()))
            {
                target = dev;
                targetName = name;
                dev.GetId(out targetId);
                break;
            }
        }
        if (target == null) { detail = "no endpoint matching '" + match + "'"; return null; }

        object clientObj;
        Guid iid = IID_IAudioClient;
        target.Activate(ref iid, 23 /*CLSCTX_ALL*/, IntPtr.Zero, out clientObj);
        var client = (IAudioClient)clientObj;

        IntPtr pFormat;
        client.GetMixFormat(out pFormat);
        var fmt = (WAVEFORMATEX)Marshal.PtrToStructure(pFormat, typeof(WAVEFORMATEX));
        detail = string.Format("{0} | mix {1} ch @ {2} Hz, {3} bits, tag=0x{4:X}", targetName,
                               fmt.nChannels, fmt.nSamplesPerSec, fmt.wBitsPerSample, fmt.wFormatTag);

        int hr = client.Initialize(0 /*SHARED*/, 0, 2000000, 0, pFormat, IntPtr.Zero);
        if (hr != 0) { detail += " | Initialize hr=0x" + hr.ToString("X8"); return null; }

        uint bufferFrames;
        client.GetBufferSize(out bufferFrames);
        object renderObj;
        Guid rid = IID_IAudioRenderClient;
        client.GetService(ref rid, out renderObj);
        var render = (IAudioRenderClient)renderObj;

        int channels = fmt.nChannels;
        int blockAlign = fmt.nBlockAlign;
        bool isFloat = fmt.wFormatTag == 3 || fmt.wBitsPerSample == 32;
        float[] program = string.IsNullOrEmpty(wavPath) ? null : LoadWav48kMono(wavPath, amplitude);
        double phase = 0.0;
        double step = 2.0 * Math.PI * freq / fmt.nSamplesPerSec;

        client.Start();
        var sw = System.Diagnostics.Stopwatch.StartNew();
        ulong framesWritten = 0;
        while (sw.Elapsed.TotalSeconds < seconds)
        {
            uint padding;
            client.GetCurrentPadding(out padding);
            uint avail = bufferFrames - padding;
            if (avail == 0) { System.Threading.Thread.Sleep(2); continue; }
            IntPtr data;
            hr = render.GetBuffer(avail, out data);
            if (hr != 0) break;
            byte[] buf = new byte[avail * blockAlign];
            for (int f = 0; f < avail; f++)
            {
                double s;
                if (program != null)
                {
                    ulong index = (framesWritten + (ulong)f) % (ulong)program.Length;
                    s = program[(int)index];
                }
                else
                {
                    s = Math.Sin(phase) * amplitude;
                    phase += step;
                }
                for (int c = 0; c < channels; c++)
                {
                    int off = f * blockAlign + c * (blockAlign / channels);
                    if (isFloat)
                        BitConverter.GetBytes((float)s).CopyTo(buf, off);
                    else
                        BitConverter.GetBytes((short)(s * 32767.0)).CopyTo(buf, off);
                }
            }
            Marshal.Copy(buf, 0, data, buf.Length);
            render.ReleaseBuffer(avail, 0);
            framesWritten += avail;
            System.Threading.Thread.Sleep(4);
        }
        client.Stop();
        detail += " | wrote " + framesWritten + " frames in " + seconds + " s";
        return targetId;
    }
}
'@

Add-Type -TypeDefinition $cs -Language CSharp | Out-Null
$detail = ''
$id = [WasapiTone]::Play($Match, $Seconds, $Freq, $Amplitude, $Wav, [ref]$detail)
Write-Output "detail: $detail"
if ($null -eq $id) { exit 1 }
Write-Output "device-id: $id"
