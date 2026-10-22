# Set the capture-endpoint volume of a microphone by friendly-name match.
# Usage: pwsh -File Utiles/mic_volume.ps1 -Match 'Realtek' -Level 1.0
param(
  [string]$Match = 'Realtek',
  [double]$Level = 1.0
)
$ErrorActionPreference = 'Stop'
$cs = @'
using System;
using System.Runtime.InteropServices;

[ComImport, Guid("BCDE0395-E52F-467C-8E3D-C4579291692E")]
public class MMDeviceEnumerator { }

[ComImport, Guid("A95664D2-9614-4F35-A746-DE8DB63617E6"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IMMDeviceEnumerator
{
    int EnumAudioEndpoints(int dataFlow, int stateMask, out IMMDeviceCollection devices);
    int GetDefaultAudioEndpoint(int dataFlow, int role, out IMMDevice endpoint);
    int GetDevice(string id, out IMMDevice device);
    int RegisterEndpointNotificationCallback(IntPtr client);
    int UnregisterEndpointNotificationCallback(IntPtr client);
}

[ComImport, Guid("0BD7A1BE-7A1A-44DB-8397-CC5392387B5E"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IMMDeviceCollection
{
    int GetCount(out int count);
    int Item(int index, out IMMDevice device);
}

[ComImport, Guid("D666063F-1587-4E43-81F1-B948E807363F"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IMMDevice
{
    int Activate(ref Guid iid, int clsCtx, IntPtr activationParams, [MarshalAs(UnmanagedType.IUnknown)] out object iface);
    int OpenPropertyStore(int access, out IPropertyStore properties);
    int GetId([MarshalAs(UnmanagedType.LPWStr)] out string id);
    int GetState(out int state);
}

[ComImport, Guid("886D8EEB-8CF2-4446-8D02-CDBA1DBDCF99"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IPropertyStore
{
    int GetCount(out int count);
    int GetAt(int index, out PROPERTYKEY key);
    int GetValue(ref PROPERTYKEY key, out PROPVARIANT value);
    int SetValue(ref PROPERTYKEY key, ref PROPVARIANT value);
    int Commit();
}

[StructLayout(LayoutKind.Sequential)]
public struct PROPERTYKEY { public Guid fmtid; public int pid; }

[StructLayout(LayoutKind.Explicit)]
public struct PROPVARIANT
{
    [FieldOffset(0)] public short vt;
    [FieldOffset(8)] public IntPtr pointerValue;
}

[ComImport, Guid("5CDF2C82-841E-4546-9722-0CF74078229A"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IAudioEndpointVolume
{
    int RegisterControlChangeNotify(IntPtr notify);
    int UnregisterControlChangeNotify(IntPtr notify);
    int GetChannelCount(out int count);
    int SetMasterVolumeLevel(float levelDb, ref Guid eventContext);
    int SetMasterVolumeLevelScalar(float level, ref Guid eventContext);
    int GetMasterVolumeLevel(out float levelDb);
    int GetMasterVolumeLevelScalar(out float level);
    int SetChannelVolumeLevel(int channel, float levelDb, ref Guid eventContext);
    int SetChannelVolumeLevelScalar(int channel, float level, ref Guid eventContext);
    int GetChannelVolumeLevel(int channel, out float levelDb);
    int GetChannelVolumeLevelScalar(int channel, out float level);
    int SetMute(bool mute, ref Guid eventContext);
    int GetMute(out bool mute);
}

public static class MicVolume
{
    private static readonly PROPERTYKEY PKEY_Device_FriendlyName =
        new PROPERTYKEY { fmtid = new Guid("a45c254e-df1c-4efd-8020-67d146a850e0"), pid = 14 };

    public static string Set(string match, double level)
    {
        var enumerator = (IMMDeviceEnumerator)new MMDeviceEnumerator();
        IMMDeviceCollection collection;
        enumerator.EnumAudioEndpoints(1 /*eCapture*/, 1 /*ACTIVE*/, out collection);
        int count;
        collection.GetCount(out count);
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
            if (name == null || !name.ToLowerInvariant().Contains(match.ToLowerInvariant())) continue;
            object volumeObj;
            var iid = new Guid("5CDF2C82-841E-4546-9722-0CF74078229A");
            dev.Activate(ref iid, 23 /*CLSCTX_ALL*/, IntPtr.Zero, out volumeObj);
            var volume = (IAudioEndpointVolume)volumeObj;
            var context = Guid.Empty;
            volume.SetMasterVolumeLevelScalar((float)Math.Clamp(level, 0.0, 1.0), ref context);
            float now;
            volume.GetMasterVolumeLevelScalar(out now);
            return name + " -> " + now.ToString("0.000");
        }
        return "no capture endpoint matching '" + match + "'";
    }
}
'@
Add-Type -TypeDefinition $cs -Language CSharp | Out-Null
Write-Output ([MicVolume]::Set($Match, $Level))
