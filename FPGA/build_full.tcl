prj_project open "D:/Data/OneDrive/Projects/UMH/Software/UMH Controller/FPGA/UMH_7.ldf"
prj_run Synthesis -impl UMH_7_1
prj_run Map -impl UMH_7_1
prj_run PAR -impl UMH_7_1
prj_run Export -impl UMH_7_1
prj_project close
