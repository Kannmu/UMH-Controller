set project_file [file normalize [file join [pwd] UMH_7.ldf]]
if {[catch {prj_project open $project_file} err]} {
    puts "ERROR: project open failed: $err"
    exit 1
}
foreach step {Synthesis Map PAR} {
    if {[catch {prj_run $step -impl UMH_7_1} err]} {
        puts "ERROR: $step failed: $err"
        catch {prj_project close}
        exit 1
    }
}
if {[catch {prj_run Export -impl UMH_7_1 -task Bitgen} err]} {
    puts "ERROR: Bitgen failed: $err"
    catch {prj_project close}
    exit 1
}
if {[catch {prj_run Export -impl UMH_7_1 -task Jedecgen} err]} {
    puts "ERROR: Jedecgen failed: $err"
    catch {prj_project close}
    exit 1
}
prj_project close
puts "UMH_7_1 Diamond build completed"
exit 0
