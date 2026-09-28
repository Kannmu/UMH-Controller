"""Tiny OpenOCD Tcl-RPC client (port 6666).  Memory reads do not halt the core."""
import socket

class Ocd:
    def __init__(self, host="127.0.0.1", port=6666):
        self.s = socket.create_connection((host, port))

    def cmd(self, c):
        self.s.sendall(c.encode() + b"\x1a")
        buf = b""
        while not buf.endswith(b"\x1a"):
            buf += self.s.recv(65536)
        return buf[:-1].decode(errors="replace")

    def read32(self, addr, n=1):
        r = self.cmd("read_memory 0x%x 32 %d" % (addr, n))
        return [int(v, 16) for v in r.split()]

    def read16(self, addr, n=1):
        r = self.cmd("read_memory 0x%x 16 %d" % (addr, n))
        return [int(v, 16) for v in r.split()]
