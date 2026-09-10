if {[catch {prj_project open "D:/Data/OneDrive/Projects/UMH/Software/UMH Controller/FPGA/UMH_7.ldf"} err]} {
    puts "ERROR: project open failed: $err"
    exit 1
}
foreach step {Synthesis Map PAR Export} {
    if {[catch {prj_run $step -impl UMH_7_1} err]} {
        puts "ERROR: $step failed: $err"
        catch {prj_project close}
        exit 1
    }
}
prj_project close
puts "UMH_7_1 Diamond build completed"
exit 0
