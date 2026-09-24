# Raw USBTMC client using the IVI ausbtmc.sys device interface.
import time, win32con, win32event, win32file, pywintypes
PATH = chr(92)*2 + chr(63) + chr(92) + "USB#VID_1AB1&PID_04CE#DS1ZF253901234#{a9fdbb24-128a-11d5-9961-00108335e361}"
def open_dev():
    return win32file.CreateFile(PATH, win32con.GENERIC_READ|win32con.GENERIC_WRITE, 3, None, win32con.OPEN_EXISTING, win32con.FILE_FLAG_OVERLAPPED, None)

def _cancel(h):
    try:
        win32file.CancelIo(h)
    except Exception:
        pass

def write_dev(h, data, timeout=2.0):
    ov = pywintypes.OVERLAPPED()
    ov.hEvent = win32event.CreateEvent(None, True, False, None)
    err, n = win32file.WriteFile(h, data, ov)
    if err == 997:
        rc = win32event.WaitForSingleObject(ov.hEvent, int(timeout*1000))
        if rc != 0:
            _cancel(h); return None
        n = win32file.GetOverlappedResult(h, ov, False)
    win32file.CloseHandle(ov.hEvent)
    return n


def read_dev(h, size=4096, timeout=2.0):
    buf = win32file.AllocateReadBuffer(size)
    ov = pywintypes.OVERLAPPED()
    ov.hEvent = win32event.CreateEvent(None, True, False, None)
    err, data = win32file.ReadFile(h, buf, ov)
    if err == 997:
        rc = win32event.WaitForSingleObject(ov.hEvent, int(timeout*1000))
        if rc != 0:
            _cancel(h); win32file.CloseHandle(ov.hEvent); return None
        n = win32file.GetOverlappedResult(h, ov, False)
    else:
        n = len(data) if data is not None else 0
    win32file.CloseHandle(ov.hEvent)
    return bytes(buf[:n])

if __name__ == "__main__":
    h = open_dev(); print("handle", h)
    print("write bytes", write_dev(h, b"*IDN?\n"))
    time.sleep(0.05)
    print("read", read_dev(h, 4096, 1.5))
    win32file.CloseHandle(h)
