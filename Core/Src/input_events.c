#include "input_events.h"
#include "main.h"
#include <string.h>

#define INPUT_EVENT_QUEUE_SIZE 16u
#define INPUT_DEBOUNCE_SAMPLES 3u
/* A hold this long is reported as INPUT_EVENT_LONG the moment it is reached,
 * without waiting for the release.  The gun menu uses it as a mode toggle: the
 * UI has to switch while the key is still down, and the release that follows a
 * LONG produces no event at all, so one long press is exactly one toggle and the
 * click after it is an ordinary short press. */
#define INPUT_LONG_PRESS_MS 3000u
static input_event_t queue[INPUT_EVENT_QUEUE_SIZE];
static volatile uint8_t read_index;
static volatile uint8_t write_index;
static uint8_t key_state;
static uint8_t raw_state;
static uint8_t stable_samples[4];
static uint8_t combination_state;
static uint32_t key_down_time[4];
/* Bit i is set once key i has been reported long, so a hold is announced once
 * and its release is not announced as a second long press. */
static uint8_t long_reported;
/* Bit i is set for a back-key press whose click has not been reported yet.
 * KEY0 and KEY3 are only known to be clicks when they come back up before the
 * long-press threshold, so their press reports are deferred to the release; a hold
 * that reaches INPUT_LONG_PRESS_MS emits LONG without ever emitting a click. */
static uint8_t short_pending;

/* Bench counters.  "The long press does nothing" has three places to fail -- the
 * pin, the report, or the handler -- and only the last one is visible in the GUI
 * state.  These count what the input task actually queued, so a probe can tell
 * "the long press was never generated" from "it was generated and ignored".
 * gun_trigger.c's _Static_assert block owns the struct offsets; these are plain
 * globals with their own symbols. */
volatile uint32_t input_events_press_count;
volatile uint32_t input_events_release_count;
volatile uint32_t input_events_long_count;
volatile uint32_t input_events_last_key;
volatile uint32_t input_events_last_type;
volatile uint32_t input_events_dropped;

void input_events_init(void)
{
  memset(queue, 0, sizeof(queue));
  read_index = 0u;
  write_index = 0u;
  key_state = 0u;
  raw_state = 0u;
  combination_state = 0u;
  long_reported = 0u;
  short_pending = 0u;
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
        long_reported &= (uint8_t)~(1u << i);
        if (i == (uint8_t)INPUT_KEY0 || i == (uint8_t)INPUT_KEY3) {
          /* Defer the back key click until release: a hold that becomes
           * INPUT_EVENT_LONG must not first run the short-press action. */
          short_pending |= (uint8_t)(1u << i);
        } else {
          event.key = i; event.type = INPUT_EVENT_PRESS; event.mask = key_state; event.timestamp = now;
          ++input_events_press_count;
          input_events_last_key = i;
          input_events_last_type = INPUT_EVENT_PRESS;
          if (input_events_push(&event) == 0u) ++input_events_dropped;
        }
      } else if ((long_reported & (uint8_t)(1u << i)) != 0u) {
        /* The hold was already announced when it crossed the threshold; the
         * release carries no meaning of its own and must not be reported as a
         * second long press.  A deferred back key click is discarded too. */
        long_reported &= (uint8_t)~(1u << i);
        short_pending &= (uint8_t)~(1u << i);
      } else if ((short_pending & (uint8_t)(1u << i)) != 0u) {
        /* The back key was released before the long-press threshold: report
         * the deferred click now, still as PRESS so every page handler keeps
         * working unchanged. */
        short_pending &= (uint8_t)~(1u << i);
        event.key = i; event.type = INPUT_EVENT_PRESS;
        event.mask = key_state; event.timestamp = now;
        ++input_events_press_count;
        input_events_last_key = i;
        input_events_last_type = INPUT_EVENT_PRESS;
        if (input_events_push(&event) == 0u) ++input_events_dropped;
      } else {
        event.key = i; event.type = INPUT_EVENT_RELEASE;
        event.mask = key_state; event.timestamp = now;
        ++input_events_release_count;
        input_events_last_key = i;
        input_events_last_type = INPUT_EVENT_RELEASE;
        if (input_events_push(&event) == 0u) ++input_events_dropped;
      }
    }
  }
  /* Announce a hold as soon as it is long, while the key is still down. */
  for (i = 0u; i < 4u; ++i) {
    if ((key_state & (uint8_t)(1u << i)) == 0u) continue;
    if ((long_reported & (uint8_t)(1u << i)) != 0u) continue;
    if ((uint32_t)(now - key_down_time[i]) < INPUT_LONG_PRESS_MS) continue;
    long_reported |= (uint8_t)(1u << i);
    short_pending &= (uint8_t)~(1u << i);   /* a hold is not a click */
    event.key = i; event.type = INPUT_EVENT_LONG;
    event.mask = key_state; event.timestamp = now;
    ++input_events_long_count;
    input_events_last_key = i;
    input_events_last_type = INPUT_EVENT_LONG;
    if (input_events_push(&event) == 0u) ++input_events_dropped;
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
