# Bench probe for the PA9 "ultrasound gun" trigger.
#
#   openocd -f openocd.cfg -f Utiles/gun_probe.tcl -c "umh_gun_state" -c "shutdown"
#   openocd -f openocd.cfg -f Utiles/gun_probe.tcl -c "umh_gun_inject 800 1000 50" -c "shutdown"
#   openocd -f openocd.cfg -f Utiles/gun_probe.tcl -c "umh_gun_retrigger 400 60 400 600 50" -c "shutdown"
#   openocd -f openocd.cfg -f Utiles/gun_probe.tcl -c "umh_gun_watchdog 60" -c "shutdown"
#
# umh_gun_state    : one gun_trigger + fpga_link + system_status snapshot.
# umh_gun_inject   : hold a synthetic press for hold_ms, release, then sample
#                    every step_ms for observe_ms.
# umh_gun_retrigger: press / release / press again, to exercise a shot that is
#                    re-armed while the fade-out is still running.
# umh_gun_watchdog : run a release, then break the render task's fade service
#                    (clear base_ready and back-date the ramp) so only the
#                    protocol task's release deadline can mute the array.
#
# The synthetic press writes pressed=1 and stable_samples=0 into the statics.
# The input task re-reads PA9 every 10 ms and only moves the debounced level
# after UMH_GUN_DEBOUNCE_SAMPLES (5) consecutive disagreeing samples, so
# rewriting both bytes faster than ~50 ms of target run time keeps the shot
# latched without a physical switch.  That is the only way to exercise the
# release path (the protocol task's edge handling and the render task's fade)
# with the ST-Link as the only stimulus.  Everything is read back over SWD, so
# the test proves the logic path; the ultrasound supply can stay off.
#
# Offsets come from the DWARF of the debug ELF (arm-none-eabi-gdb
# "ptype /o umh_gun_trigger_t"); gun_trigger, fpga_link and uwTick keep these
# addresses in both build trees.

set UMH_GUN    0x20005da0
set UMH_LINK   0x20004450
set UMH_STATUS 0x2001447c
set UMH_TICK   0x20014418

proc umh_halted {} {
    set n 0
    while {[stm32g4x.cpu curstate] ne "halted" && $n < 400} {
        sleep 5
        incr n
    }
    return [expr {[stm32g4x.cpu curstate] eq "halted"}]
}

proc umh_word {addr} {
    return [expr {[mrw $addr] & 0xFFFFFFFF}]
}

proc umh_byte {addr} {
    return [expr {[mrw [expr {$addr & ~3}]] >> (8 * ($addr & 3)) & 0xFF}]
}

# One snapshot line.  tag is printed first so a time series stays readable.
proc umh_gun_line {tag} {
    global UMH_GUN UMH_LINK UMH_STATUS UMH_TICK
    set g $UMH_GUN
    set w0  [umh_word $g]
    set w7  [umh_word [expr {$g + 28}]]
    set w28 [umh_word [expr {$g + 112}]]
    set tick [umh_word $UMH_TICK]
    set ramp [umh_word [expr {$g + 20}]]
    set age [expr {($tick - $ramp) & 0xFFFFFFFF}]
    set flags [umh_word $UMH_STATUS]
    set link_flags [expr {([umh_word [expr {$UMH_LINK + 528}]] >> 16) & 0xFFFF}]
    set link_run [expr {[umh_word [expr {$UMH_LINK + 540}]] & 0xFF}]
    echo [format "%s tick=%u press=%u eng=%u pend=%u raw=%u stab=%u fires=%u defer=%u seq=%u env=%u lvl=%u base=%u ready=%u sent=%u ch0=%u ch83=%u rgbR=%u ramp_age=%u linkrun=%u linkfl=0x%04x sysfl=0x%08x" \
        $tag $tick \
        [expr {$w0 & 0xFF}] [expr {($w0 >> 8) & 0xFF}] [expr {($w0 >> 16) & 0xFF}] \
        [expr {($w0 >> 24) & 0xFF}] [umh_byte [expr {$g + 4}]] \
        [umh_word [expr {$g + 8}]] [umh_word [expr {$g + 12}]] [umh_word [expr {$g + 24}]] \
        [expr {$w7 & 0xFF}] [expr {($w7 >> 8) & 0xFF}] [expr {($w7 >> 16) & 0xFF}] \
        [expr {($w28 >> 16) & 0xFF}] [expr {($w28 >> 24) & 0xFF}] \
        [expr {([umh_word [expr {$g + 136}]] >> 24) & 0xFF}] \
        [expr {([umh_word [expr {$g + 304}]] >> 8) & 0xFF}] \
        [expr {([umh_word [expr {$g + 304}]] >> 16) & 0xFF}] \
        $age $link_run $link_flags $flags]
}

proc umh_gun_open {} {
    init
    halt
    if {![umh_halted]} {
        echo "UMH_GUN: target did not halt"
        shutdown
        return -1
    }
    return 0
}

# Keep the synthetic press latched for hold_ms of host time.  Halt, rewrite,
# resume: each round trip leaves the target running for a few milliseconds, so
# the input task never collects its five disagreeing samples.
proc umh_gun_hold {hold_ms} {
    global UMH_GUN
    set t0 [clock milliseconds]
    set n 0
    while {[expr {[clock milliseconds] - $t0}] < $hold_ms} {
        halt
        if {![umh_halted]} { break }
        mwb $UMH_GUN 1
        mwb [expr {$UMH_GUN + 4}] 0
        resume
        sleep 5
        incr n
    }
    return $n
}

proc umh_gun_observe {observe_ms step_ms} {
    set steps [expr {$observe_ms / $step_ms}]
    for {set i 0} {$i < $steps} {incr i} {
        sleep $step_ms
        halt
        if {![umh_halted]} { break }
        umh_gun_line [format "T+%d" [expr {($i + 1) * $step_ms}]]
        resume
    }
}

proc umh_gun_state {} {
    if {[umh_gun_open] != 0} { return -1 }
    umh_gun_line "STATE"
    resume
    return 0
}

proc umh_gun_inject {hold_ms observe_ms step_ms} {
    if {[umh_gun_open] != 0} { return -1 }
    umh_gun_line "BEFORE"
    resume
    set n [umh_gun_hold $hold_ms]
    echo "UMH_GUN: press rewritten $n times over $hold_ms ms of host time"
    halt
    umh_gun_line "DURING"
    resume
    # Stop rewriting: the input task now sees PA9 high and drops the debounced
    # level after its five samples, which is the release edge under test.
    umh_gun_observe $observe_ms $step_ms
    shutdown
    return 0
}

proc umh_gun_retrigger {hold1_ms gap_ms hold2_ms observe_ms step_ms} {
    if {[umh_gun_open] != 0} { return -1 }
    umh_gun_line "BEFORE"
    resume
    set n [umh_gun_hold $hold1_ms]
    echo "UMH_GUN: first press rewritten $n times"
    sleep $gap_ms
    halt
    umh_gun_line "GAP"
    resume
    set n [umh_gun_hold $hold2_ms]
    echo "UMH_GUN: second press rewritten $n times"
    halt
    umh_gun_line "SECOND"
    resume
    umh_gun_observe $observe_ms $step_ms
    shutdown
    return 0
}

# Break the render task's fade service on purpose: with base_ready cleared the
# refresh returns immediately, so only gun_trigger_release()'s deadline can
# mute the array.  ramp_start_ms is back-dated so the deadline is already past.
proc umh_gun_watchdog {gap_ms} {
    global UMH_GUN UMH_TICK
    if {[umh_gun_open] != 0} { return -1 }
    umh_gun_line "BEFORE"
    resume
    set n [umh_gun_hold 800]
    echo "UMH_GUN: press rewritten $n times"
    sleep $gap_ms
    halt
    umh_gun_line "RELEASED"
    mwb [expr {$UMH_GUN + 114}] 0
    mww [expr {$UMH_GUN + 20}] [expr {([umh_word $UMH_TICK] - 400) & 0xFFFFFFFF}]
    echo "UMH_GUN: base_ready cleared and ramp_start back-dated by 400 ms"
    resume
    umh_gun_observe 600 50
    shutdown
    return 0
}
