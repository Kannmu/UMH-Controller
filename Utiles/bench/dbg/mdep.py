import pyvisa, time
rm = pyvisa.ResourceManager()
s = rm.open_resource("USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR")
s.timeout = 5000
for val in ("12000", "120000", "1200000", "12000000", "12M", "AUTO"):
    try:
        s.write(":ACQuire:MDEPth " + val)
        s.query("*OPC?")
        print(val, "->", s.query(":ACQuire:MDEPth?").strip())
    except Exception as e:
        print(val, "ERR", type(e).__name__)
        try: s.read()
        except Exception: pass
s.close()

