# Diamond batch flow for the UMH MachXO2 project.
# Synthesis is run first so the LSE-only installation can validate RTL and
# produce its resource report before map/PAR are attempted.
prj_project open "D:/Data/OneDrive/Projects/UMH/Software/UMH Controller/FPGA/UMH_7.ldf"
prj_run Synthesis -impl UMH_7_1
prj_project close
