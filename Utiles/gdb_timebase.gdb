# Timebase / tick diagnosis. Attach without resetting and sample the live state.
set pagination off
set confirm off
target extended-remote localhost:3333
monitor halt
printf "\n==== live state ====\n"
printf "uwTick            = %u\n", uwTick
printf "uwTickPrio        = %d\n", uwTickPrio
printf "uwTickFreq        = %d\n", uwTickFreq
printf "HAL_GetTick ptr   = %p\n", &uwTick
printf "uxCurrentNumberOfTasks = %u\n", uxCurrentNumberOfTasks
printf "uxSchedulerSuspended   = %u\n", uxSchedulerSuspended
printf "xTickCount        = %u\n", *(unsigned int*)0x20000000
printf "\n==== RCC ====\n"
printf "RCC_CR      = 0x%08x\n", *(unsigned int*)0x40021000
printf "RCC_CFGR    = 0x%08x\n", *(unsigned int*)0x40021008
printf "RCC_APB1ENR1= 0x%08x\n", *(unsigned int*)0x40021058
printf "\n==== TIM2 (HAL timebase candidate) ====\n"
printf "TIM2_CR1  = 0x%08x\n", *(unsigned int*)0x40000000
printf "TIM2_DIER = 0x%08x\n", *(unsigned int*)0x4000000C
printf "TIM2_SR   = 0x%08x\n", *(unsigned int*)0x40000010
printf "TIM2_CNT  = 0x%08x\n", *(unsigned int*)0x40000024
printf "TIM2_PSC  = 0x%08x\n", *(unsigned int*)0x40000028
printf "TIM2_ARR  = 0x%08x\n", *(unsigned int*)0x4000002C
printf "\n==== NVIC ====\n"
printf "ISER0 = 0x%08x ISER1 = 0x%08x\n", *(unsigned int*)0xE000E100, *(unsigned int*)0xE000E104
printf "TIM2_IRQn(28) enabled: %d\n", (*(unsigned int*)0xE000E100 >> 28) & 1
printf "\n==== SysTick ====\n"
printf "SysTick CTRL=0x%08x LOAD=0x%08x VAL=0x%08x\n", *(unsigned int*)0xE000E010, *(unsigned int*)0xE000E014, *(unsigned int*)0xE000E018
printf "\n==== vector table relocation ====\n"
printf "VTOR=0x%08x\n", *(unsigned int*)0xE000ED08
printf "RAM vector TIM2 (28+16=44 word) = 0x%08x\n", *(unsigned int*)(*(unsigned int*)0xE000ED08 + 4*44)
printf "\n==== gun_trigger ====\n"
print gun_trigger.pressed
print gun_trigger.engaged
print gun_trigger.envelope_state
print gun_trigger.current_level
print gun_trigger.fires
print gun_trigger.base_ready
print gun_trigger.envelope_sent
printf "\n==== detach (leave running) ====\n"
detach
quit
