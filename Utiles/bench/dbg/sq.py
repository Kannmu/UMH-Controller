import pyvisa
rm = pyvisa.ResourceManager()
s = rm.open_resource("USB0::0x1AB1::0x04CE::DS1ZF253901234::INSTR")
s.timeout = 3000
print("IDN:", s.query("*IDN?").strip())

for ch in (1,2):
    print("CH%d probe=%s coup=%s scale=%s off=%s bw=%s" % (ch, s.query(":CHANnel%d:PROBe?"%ch).strip(), s.query(":CHANnel%d:COUPling?"%ch).strip(), s.query(":CHANnel%d:SCALe?"%ch).strip(), s.query(":CHANnel%d:OFFSet?"%ch).strip(), s.query(":CHANnel%d:BWLimit?"%ch).strip()))

print("MATH disp=%s op=%s src1=%s src2=%s scale=%s" % (s.query(":MATH:DISPlay?").strip(), s.query(":MATH:OPERator?").strip(), s.query(":MATH:SOURce1?").strip(), s.query(":MATH:SOURce2?").strip(), s.query(":MATH:SCALe?").strip()))

print("TB scale=%s off=%s" % (s.query(":TIMebase:MAIN:SCALe?").strip(), s.query(":TIMebase:MAIN:OFFSet?").strip()))
print("ACQ type=%s mdep=%s" % (s.query(":ACQuire:TYPE?").strip(), s.query(":ACQuire:MDEPth?").strip()))
print("TRIG st=%s sweep=%s src=%s lev=%s" % (s.query(":TRIGger:STATus?").strip(), s.query(":TRIGger:SWEep?").strip(), s.query(":TRIGger:EDGE:SOURce?").strip(), s.query(":TRIGger:EDGE:LEVel?").strip()))
s.close()

