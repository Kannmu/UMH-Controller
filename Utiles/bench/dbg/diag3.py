import pyvisa, time, numpy as np
rm = pyvisa.ResourceManager()
s = rm.open_resource("USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR")
s.timeout = 8000
s.write(":TIMebase:MAIN:SCALe 5e-3")
s.write(":TRIGger:SWEep AUTO")
s.write(":WAVeform:MODE NORMal")
s.write(":WAVeform:FORMat BYTE")
print("mathdisp", s.query(":MATH:DISPlay?").strip())

for src in ("MATH", "CHAN1"):
    s.write(":WAVeform:SOURce " + src)
    for t in range(2):
        s.write(":RUN"); time.sleep(0.5); s.write(":STOP"); time.sleep(0.2)
        s.write(":WAVeform:POINts 1200")
        pre = s.query(":WAVeform:PREamble?").strip().split(",")
        raw = s.query_binary_values(":WAVeform:DATA?", datatype="B", container=np.ndarray)
        print(src, t, "npts", pre[2], "len", len(raw))
s.close()

