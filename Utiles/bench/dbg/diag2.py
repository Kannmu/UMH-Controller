import pyvisa, time, numpy as np
rm = pyvisa.ResourceManager()
s = rm.open_resource("USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR")
s.timeout = 8000
s.write(":TIMebase:MAIN:SCALe 5e-3")
s.write(":TRIGger:SWEep AUTO")
s.write(":WAVeform:SOURce MATH")
s.write(":WAVeform:MODE NORMal")
s.write(":WAVeform:FORMat BYTE")

for i in range(5):
    s.write(":RUN"); time.sleep(0.35); s.write(":STOP"); time.sleep(0.2)
    s.write(":WAVeform:POINts 1200")
    pre = s.query(":WAVeform:PREamble?").strip().split(",")
    npts = int(float(pre[2]))
    raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", container=np.ndarray)
    print(i, "npts", npts, "len", len(raw), "xinc", pre[4], "yinc", pre[7])
s.close()

