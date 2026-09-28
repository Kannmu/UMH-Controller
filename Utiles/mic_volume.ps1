# Get/set capture endpoint volume by friendly-name substring (Core Audio).
# usage: pwsh Utiles/mic_volume.ps1 -Name "Realtek" [-Level 0.05]
param([string]$Name = "Realtek", [double]$Level = -1)
Add-Type -Language CSharp @"
using System; using System.Runtime.InteropServices;
[Guid("5CDF2C82-841E-4546-9722-0CF74078229A"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IAudioEndpointVolume {
  int RegisterControlChangeNotify(IntPtr p); int UnregisterControlChangeNotify(IntPtr p);
  int GetChannelCount(out uint c); int SetMasterVolumeLevel(float l, Guid g);
  int SetMasterVolumeLevelScalar(float l, ref Guid g); int GetMasterVolumeLevel(out float l);
  int GetMasterVolumeLevelScalar(out float l);
}
[Guid("D666063F-1587-4E43-81F1-B948E807363F"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IMMDevice {
  int Activate(ref Guid iid, int ctx, IntPtr p, [MarshalAs(UnmanagedType.IUnknown)] out object o);
  int OpenPropertyStore(int access, out IPropertyStore ps); int GetId([MarshalAs(UnmanagedType.LPWStr)] out string id);
  int GetState(out int s);
}
[Guid("0BD7A1BE-7A1A-44DB-8397-CC5392387B5E"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IMMDeviceCollection { int GetCount(out uint n); int Item(uint i, out IMMDevice d); }
[Guid("A95664D2-9614-4F35-A746-DE8DB63617E6"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IMMDeviceEnumerator { int EnumAudioEndpoints(int flow, int mask, out IMMDeviceCollection c); }
[Guid("886d8eeb-8cf2-4446-8d02-cdba1dbdcf99"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
public interface IPropertyStore { int GetCount(out uint c); int GetAt(uint i, out PropKey k); int GetValue(ref PropKey k, out PropVariant v); }
[StructLayout(LayoutKind.Sequential)] public struct PropKey { public Guid fmtid; public int pid; }
[StructLayout(LayoutKind.Explicit)] public struct PropVariant { [FieldOffset(0)] public short vt; [FieldOffset(8)] public IntPtr p; }
[ComImport, Guid("BCDE0395-E52F-467C-8E3D-C4579291692E")] public class MMDeviceEnumeratorCo {}
public static class MicVol {
  public static string Run(string name, float level) {
    var en = (IMMDeviceEnumerator)new MMDeviceEnumeratorCo(); IMMDeviceCollection col;
    en.EnumAudioEndpoints(1, 1, out col); uint n; col.GetCount(out n); string outp = "";
    for (uint i = 0; i < n; i++) {
      IMMDevice d; col.Item(i, out d); IPropertyStore ps; d.OpenPropertyStore(0, out ps);
      var key = new PropKey { fmtid = new Guid("a45c254e-df1c-4efd-8020-67d146a850e0"), pid = 14 };
      PropVariant v; ps.GetValue(ref key, out v); string fn = Marshal.PtrToStringUni(v.p);
      if (fn == null || fn.IndexOf(name, StringComparison.OrdinalIgnoreCase) < 0) continue;
      var iid = typeof(IAudioEndpointVolume).GUID; object o; d.Activate(ref iid, 23, IntPtr.Zero, out o);
      var ev = (IAudioEndpointVolume)o; float cur, db; ev.GetMasterVolumeLevelScalar(out cur); ev.GetMasterVolumeLevel(out db);
      outp += fn + " scalar=" + cur + " dB=" + db;
      if (level >= 0) { Guid g = Guid.Empty; ev.SetMasterVolumeLevelScalar(level, ref g); ev.GetMasterVolumeLevel(out db); outp += " -> " + level + " (" + db + " dB)"; }
      outp += "\n";
    }
    return outp;
  }
}
"@
[MicVol]::Run($Name, [float]$Level)
