#include <stdio.h>
#include <stddef.h>
#include "gun_trigger.h"
int main(void) {
  printf("sizeof(umh_gun_trigger_t)=%zu\n", sizeof(umh_gun_trigger_t));
  printf("pressed=%zu engaged=%zu refresh_pending=%zu raw=%zu stable=%zu\n",
         offsetof(umh_gun_trigger_t,pressed), offsetof(umh_gun_trigger_t,engaged),
         offsetof(umh_gun_trigger_t,refresh_pending), offsetof(umh_gun_trigger_t,raw),
         offsetof(umh_gun_trigger_t,stable_samples));
  printf("fires=%zu deferred=%zu last_submit=%zu ramp_start=%zu seq=%zu\n",
         offsetof(umh_gun_trigger_t,fires), offsetof(umh_gun_trigger_t,deferred),
         offsetof(umh_gun_trigger_t,last_submit_ms), offsetof(umh_gun_trigger_t,ramp_start_ms),
         offsetof(umh_gun_trigger_t,sequence));
  printf("env=%zu level=%zu base_ready=%zu sent=%zu\n",
         offsetof(umh_gun_trigger_t,envelope_state), offsetof(umh_gun_trigger_t,current_level),
         offsetof(umh_gun_trigger_t,base_ready), offsetof(umh_gun_trigger_t,envelope_sent));
  printf("frame=%zu base_level=%zu real=%zu imag=%zu\n",
         offsetof(umh_gun_trigger_t,frame), offsetof(umh_gun_trigger_t,base_level),
         offsetof(umh_gun_trigger_t,real_accum), offsetof(umh_gun_trigger_t,imag_accum));
  printf("frame.channels=%zu frame.rgb=%zu sizeof(frame)=%zu\n",
         offsetof(umh_gun_trigger_t,frame)+offsetof(umh_output_frame_t,channels),
         offsetof(umh_gun_trigger_t,frame)+offsetof(umh_output_frame_t,rgb),
         sizeof(umh_output_frame_t));
  return 0;
}
