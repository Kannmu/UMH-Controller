# The core appears to reboot before application_init completes. Walk the stack
# by hand and check the reboot evidence.
set pagination off
set confirm off
target extended-remote localhost:3333
monitor reset halt
printf "\n==== right after reset ====\n"
info registers pc sp msp psp lr
printf "RCC_CSR=0x%08x  (reset cause flags)\n", *(unsigned int*)0x40021024
printf "SCB AIRCR=0x%08x\n", *(unsigned int*)0xE000ED0C
printf "RCC_CR=0x%08x RCC_CFGR=0x%08x\n", *(unsigned int*)0x40021000, *(unsigned int*)0x40021008
printf "IWDG SR=0x%08x PR=0x%08x RLR=0x%08x\n", *(unsigned int*)0x40003000, *(unsigned int*)0x40003004, *(unsigned int*)0x40003008
printf "WWDG CR=0x%08x CFR=0x%08x\n", *(unsigned int*)0x40002C00, *(unsigned int*)0x40002C04
printf "\n==== step to main ====\n"
break main
continue
printf "reached main, tick=%u\n", uwTick
delete breakpoints
printf "\n==== break at application_init and osKernelStart ====\n"
break application_init
break osKernelStart
continue
info registers pc sp
printf "uwTick=%u  tasks=%u\n", uwTick, uxCurrentNumberOfTasks
printf "\n==== backtrace at application_init ====\n"
bt
detach
quit
