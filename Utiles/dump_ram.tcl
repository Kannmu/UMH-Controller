# Dump one RAM block from the running UMH controller.
#   openocd -f openocd.cfg -f Utiles/dump_ram.tcl -c "umh_dump <addr> <len> <path>"
#
# The ST-Link halt request is asynchronous: `halt` returns before the core has
# actually stopped, and a dump issued in that window fails with "not halted".
# Wait for the debug logic to confirm the state before reading.

proc umh_dump {addr len path} {
    init
    halt
    set n 0
    while {[stm32g4x.cpu curstate] ne "halted" && $n < 400} {
        sleep 5
        incr n
    }
    if {[stm32g4x.cpu curstate] ne "halted"} {
        echo "UMH_DUMP: target did not halt"
        shutdown
        return -1
    }
    if {[catch {dump_image $path $addr $len} err]} {
        echo "UMH_DUMP: $err"
        resume
        shutdown
        return -1
    }
    echo "UMH_DUMP: ok $path"
    resume
    shutdown
    return 0
}
