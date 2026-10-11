# Connect GDB to a running OpenOCD (localhost:3333) and report target state.
#   openocd -f openocd.cfg &
#   arm-none-eabi-gdb -q -batch -x Utiles/gdb_state.gdb build/UMH_Controller.elf
set pagination off
set confirm off
set print pretty off
target extended-remote localhost:3333
monitor reset halt
printf "\n==== halted ====\n"
info registers pc sp lr xpsr
printf "\n==== CFSR/HFSR ====\n"
printf "CFSR=0x%08x HFSR=0x%08x DFSR=0x%08x\n", *(unsigned int*)0xE000ED28, *(unsigned int*)0xE000ED2C, *(unsigned int*)0xE000ED30
printf "RCC_CSR=0x%08x (reset flags)\n", *(unsigned int*)0x40021024
printf "\n==== backtrace ====\n"
bt
printf "\n==== uwTick ====\n"
print uwTick
printf "\n==== task list symbols ====\n"
print uxCurrentNumberOfTasks
print pxCurrentTCB
printf "\n==== gun_trigger ====\n"
print gun_trigger
detach
quit
