#include "usb_audio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include <string.h>

extern osThreadId_t umh_protocol_task_handle;

volatile umh_usb_audio_stats_t umh_usb_audio_stats;
volatile uint8_t umh_usb_audio_debug_inhibit;

static int16_t pcm_ring[UMH_USB_AUDIO_RING_SAMPLES] __attribute__((aligned(4)));
static volatile uint32_t head, tail, generation, last_packet_ms;
static volatile uint8_t streaming, muted;
static volatile int16_t volume_db256;
static volatile uint16_t gain_q15 = 32768u;

/* -60..0 dB in 1 dB steps, Q1.15 (unity is 32768).  SET_CUR uses a table
 * lookup in the USB ISR; powf and floating-point ISR context are unnecessary. */
static const uint16_t volume_gain[61] = {
  33,37,41,46,52,58,65,73,82,92,104,116,130,146,164,184,207,232,260,292,
  328,368,413,463,519,583,654,734,823,924,1036,1163,1305,1464,1642,1843,
  2068,2320,2603,2920,3277,3677,4125,4629,5193,5827,6538,7336,8231,9235,
  10362,11627,13045,14637,16423,18427,20675,23198,26029,29205,32768
};

_Static_assert((UMH_USB_AUDIO_RING_SAMPLES & (UMH_USB_AUDIO_RING_SAMPLES - 1u)) == 0u,
               "USB PCM ring must be a power of two");

static void notify_protocol_from_isr(void)
{
  if (umh_protocol_task_handle != NULL) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)umh_protocol_task_handle, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

void umh_usb_audio_init(void)
{
  memset((void *)&umh_usb_audio_stats, 0, sizeof(umh_usb_audio_stats));
  umh_usb_audio_stats.build_id = UMH_USB_AUDIO_BUILD_ID;
  muted = 0u;
  volume_db256 = 0;
  gain_q15 = 32768u;
  umh_usb_audio_set_stream(0u);
}

void umh_usb_audio_set_stream(uint8_t enabled)
{
  streaming = enabled != 0u;
  head = tail = 0u;
  last_packet_ms = HAL_GetTick();
  umh_usb_audio_stats.queued_samples = 0u;
  ++generation;
  notify_protocol_from_isr();
}

void umh_usb_audio_receive(const uint8_t *data, uint16_t length)
{
  uint32_t start = DWT->CYCCNT;
  uint16_t count, index, first;
  if (streaming == 0u || length == 0u) return;
  if (data == NULL || length > UMH_USB_AUDIO_PACKET_MAX || (length & 1u) != 0u) {
    ++umh_usb_audio_stats.bad_packets;
    umh_usb_audio_set_stream(0u); /* malformed PCM fails closed */
    return;
  }
  count = length / 2u;
  if (head - tail + count > UMH_USB_AUDIO_RING_SAMPLES) {
    umh_usb_audio_stats.dropped_samples += head - tail;
    head = tail = 0u;
    ++generation; /* discard stale history and restart only with fresh PCM */
  }
  index = (uint16_t)(head & (UMH_USB_AUDIO_RING_SAMPLES - 1u));
  first = (uint16_t)(UMH_USB_AUDIO_RING_SAMPLES - index);
  if (first > count) first = count;
  memcpy(pcm_ring + index, data, first * 2u);
  if (first < count) memcpy(pcm_ring, data + first * 2u, (count - first) * 2u);
  __DMB();
  head += count;
  last_packet_ms = HAL_GetTick();
  ++umh_usb_audio_stats.packets;
  umh_usb_audio_stats.samples += count;
  umh_usb_audio_stats.queued_samples = (uint16_t)(head - tail);
  start = DWT->CYCCNT - start;
  if (start > umh_usb_audio_stats.max_isr_cycles)
    umh_usb_audio_stats.max_isr_cycles = start;
  notify_protocol_from_isr();
}

void umh_usb_audio_set_mute(uint8_t mute)
{
  muted = mute != 0u;
  notify_protocol_from_isr();
}

void umh_usb_audio_set_volume(int16_t value)
{
  int32_t db;
  if (value > 0) value = 0;
  if (value < -60 * 256) value = -60 * 256;
  db = -((-(int32_t)value + 128) / 256);
  volume_db256 = (int16_t)(db * 256);
  gain_q15 = volume_gain[db + 60];
  notify_protocol_from_isr();
}

uint8_t umh_usb_audio_mute(void) { return muted; }
int16_t umh_usb_audio_volume(void) { return volume_db256; }
uint16_t umh_usb_audio_gain_q15(void) { return gain_q15; }
uint8_t umh_usb_audio_streaming(void) { return streaming; }
uint32_t umh_usb_audio_generation(void) { return generation; }
uint32_t umh_usb_audio_last_packet_ms(void) { return last_packet_ms; }

uint16_t umh_usb_audio_read(int16_t *samples, uint16_t capacity)
{
  uint16_t count, index, first;
  /* Only this short <=96-byte copy masks the USB ISR.  No DSP/SPI operation
   * is inside the critical section; reset and packet producer cannot race it. */
  taskENTER_CRITICAL();
  count = (uint16_t)(head - tail);
  if (count > capacity) count = capacity;
  index = (uint16_t)(tail & (UMH_USB_AUDIO_RING_SAMPLES - 1u));
  first = (uint16_t)(UMH_USB_AUDIO_RING_SAMPLES - index);
  if (first > count) first = count;
  memcpy(samples, pcm_ring + index, first * 2u);
  if (first < count) memcpy(samples + first, pcm_ring, (count - first) * 2u);
  tail += count;
  umh_usb_audio_stats.queued_samples = (uint16_t)(head - tail);
  taskEXIT_CRITICAL();
  return count;
}

void umh_usb_audio_discard(void)
{
  taskENTER_CRITICAL();
  tail = head;
  umh_usb_audio_stats.queued_samples = 0u;
  taskEXIT_CRITICAL();
}
