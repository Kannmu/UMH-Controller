# Cold-boot the target cleanly (no stale breakpoints) and sample real progress.
set pagination off
set confirm off
target extended-remote localhost:3333
delete breakpoints
monitor reset halt
printf "\n==== after reset halt ====\n"
info registers pc sp
printf "RCC_CSR=0x%08x\n", *(unsigned int*)0x40021024
set $i = 0
while $i < 8
  monitor resume
  shell ping -n 1 -w 400 127.0.0.1 > nul
  monitor halt
  printf "poll %d: uwTick=%u  tasks=%u  xTickCount=%u  pc=", $i, uwTick, uxCurrentNumberOfTasks, *(unsigned int*)0x20017574
  print/x $pc
  set $i = $i + 1
end
printf "\n==== gun_trigger after boot ====\n"
print gun_trigger.fires
print gun_trigger.pressed
detach
quit
