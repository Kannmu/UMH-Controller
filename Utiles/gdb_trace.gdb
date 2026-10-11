# Characterise the target state: is the core executing, are interrupts on,
# has the scheduler started, and what does the stack look like?
set pagination off
set confirm off
target extended-remote localhost:3333
printf "\n==== initial attach (no reset) ====\n"
info registers pc sp lr xpsr primask
printf "uwTick=%u\n", uwTick
printf "uxCurrentNumberOfTasks=%u\n", uxCurrentNumberOfTasks
printf "uxSchedulerSuspended=%u\n", uxSchedulerSuspended
printf "RCC_CSR=0x%08x\n", *(unsigned int*)0x40021024
printf "SysTick CTRL=0x%08x LOAD=0x%08x VAL=0x%08x\n", *(unsigned int*)0xE000E010, *(unsigned int*)0xE000E014, *(unsigned int*)0xE000E018
printf "TIM2 CR1=0x%08x CNT=0x%08x ARR=0x%08x\n", *(unsigned int*)0x40000000, *(unsigned int*)0x40000024, *(unsigned int*)0x4000002C
printf "VTOR=0x%08x\n", *(unsigned int*)0xE000ED08
printf "flash[0..7]="
x/8xb 0x08000000
printf "ICSR=0x%08x SHCSR=0x%08x\n", *(unsigned int*)0xE000ED04, *(unsigned int*)0xE000ED24
printf "\n==== sample PC/sp 5 times while running ====\n"
set $i = 0
while $i < 5
  monitor resume
  shell sleep 0.3
  monitor halt
  printf "sample %d: pc=", $i
  print/x $pc
  printf "           uwTick=%u\n", uwTick
  set $i = $i + 1
end
printf "\n==== stack top words ====\n"
x/24xw 0x20011990
printf "\n==== SP-relative search for return addresses ====\n"
info frame
detach
quit
