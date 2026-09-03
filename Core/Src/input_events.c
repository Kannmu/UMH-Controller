#include "input_events.h"
#include "main.h"
#include <string.h>

#define INPUT_EVENT_QUEUE_SIZE 16u
#define INPUT_DEBOUNCE_SAMPLES 3u
#define INPUT_LONG_PRESS_MS 800u
static input_event_t queue[INPUT_EVENT_QUEUE_SIZE];
static volatile uint8_t read_index;
static volatile uint8_t write_index;
static uint8_t key_state;
static uint8_t raw_state;
static uint8_t stable_samples[4];
static uint8_t combination_state;
static uint32_t key_down_time[4];

void input_events_init(void)
{
  memset(queue, 0, sizeof(queue));
  read_index = 0u;
  write_index = 0u;
  key_state = 0u;
  raw_state = 0u;
  combination_state = 0u;
  memset(stable_samples, 0, sizeof(stable_samples));
  memset(key_down_time, 0, sizeof(key_down_time));
}

uint8_t input_events_push(const input_event_t *event)
{
  uint8_t next;
  if (event == NULL) return 0u;
  next = (uint8_t)((write_index + 1u) % INPUT_EVENT_QUEUE_SIZE);
  if (next == read_index) return 0u;
  queue[write_index] = *event;
  write_index = next;
  return 1u;
}

uint8_t input_events_poll(input_event_t *event)
{
  if (event == NULL || read_index == write_index) return 0u;
  *event = queue[read_index];
  read_index = (uint8_t)((read_index + 1u) % INPUT_EVENT_QUEUE_SIZE);
  return 1u;
}

void input_events_sample(void)
{
  const uint16_t pins[4] = {KEY0_Pin, KEY1_Pin, KEY2_Pin, KEY3_Pin};
  uint8_t i;
  uint8_t current = 0u;
  uint8_t stable;
  uint8_t changed;
  uint8_t bit_count = 0u;
  uint32_t now = HAL_GetTick();
  input_event_t event;
  for (i = 0u; i < 4u; ++i) if (HAL_GPIO_ReadPin(GPIOA, pins[i]) == GPIO_PIN_RESET) current |= (uint8_t)(1u << i);
  raw_state = current;
  for (i = 0u; i < 4u; ++i) {
    if ((uint8_t)(current & (1u << i)) != (uint8_t)(key_state & (1u << i))) {
      if (stable_samples[i] < INPUT_DEBOUNCE_SAMPLES) ++stable_samples[i];
    } else {
      stable_samples[i] = 0u;
    }
    if (stable_samples[i] >= INPUT_DEBOUNCE_SAMPLES) {
      key_state = (uint8_t)((key_state & (uint8_t)~(1u << i)) | (current & (1u << i)));
      stable_samples[i] = 0u;
      if ((key_state & (1u << i)) != 0u) {
        key_down_time[i] = now;
        event.key = i; event.type = INPUT_EVENT_PRESS; event.mask = key_state; event.timestamp = now;
        (void)input_events_push(&event);
      } else {
        event.key = i;
        event.type = (uint32_t)(now - key_down_time[i]) >= INPUT_LONG_PRESS_MS ? INPUT_EVENT_LONG : INPUT_EVENT_RELEASE;
        event.mask = key_state; event.timestamp = now;
        (void)input_events_push(&event);
      }
    }
  }
  stable = key_state;
  for (i = 0u; i < 4u; ++i) if ((stable & (1u << i)) != 0u) ++bit_count;
  changed = (uint8_t)(stable != combination_state);
  if (changed != 0u && bit_count >= 2u) {
    event.key = 0xFFu;
    event.type = INPUT_EVENT_COMBINATION;
    event.mask = stable;
    event.timestamp = now;
    (void)input_events_push(&event);
  }
  if (bit_count < 2u) combination_state = 0u;
  else combination_state = stable;
}
