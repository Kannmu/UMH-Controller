import pyvisa, time, numpy as np
rm = pyvisa.ResourceManager()
s = rm.open_resource("USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR")
s.timeout = 8000
s.write(":TIMebase:MAIN:SCALe 5e-3")
s.write(":TRIGger:SWEep AUTO")
s.write(":RUN"); time.sleep(0.8); s.write(":STOP"); time.sleep(0.3)
s.write(":WAVeform:SOURce CHAN1")
s.write(":WAVeform:MODE NORMal")
s.write(":WAVeform:FORMat BYTE")
s.write(":WAVeform:POINts 1200")

st = s.query(":TRIGger:STATus?").strip()
print("status", st)
pre = s.query(":WAVeform:PREamble?").strip().split(",")
print("pre", pre)
raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", container=np.ndarray)
print("len", len(raw))
v = (np.asarray(raw, dtype=float) - float(pre[9])) * float(pre[8]) + float(pre[9-1])

print("n", len(v), "min %.2f max %.2f std %.3f" % (v.min(), v.max(), v.std()))
print(np.round(v[:20], 2))
s.close()

