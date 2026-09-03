#ifndef INPUT_EVENTS_H
#define INPUT_EVENTS_H

#include <stdint.h>

typedef enum {
  INPUT_KEY0 = 0u,
  INPUT_KEY1 = 1u,
  INPUT_KEY2 = 2u,
  INPUT_KEY3 = 3u
} input_key_t;

typedef enum {
  INPUT_EVENT_PRESS = 1u,
  INPUT_EVENT_RELEASE = 2u,
  INPUT_EVENT_LONG = 3u,
  INPUT_EVENT_COMBINATION = 4u
} input_event_type_t;

typedef struct {
  uint8_t key;
  uint8_t type;
  uint16_t mask;
  uint32_t timestamp;
} input_event_t;

void input_events_init(void);
uint8_t input_events_poll(input_event_t *event);
uint8_t input_events_push(const input_event_t *event);
void input_events_sample(void);

#endif
