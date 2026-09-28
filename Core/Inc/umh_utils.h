#ifndef UMH_UTILS_H
#define UMH_UTILS_H

#include <stdint.h>

/* Unified utilities and constants used across the firmware.
 * Replaces duplicate read_u16/write_u32/get_u16/put_u32 functions scattered
 * across app_freertos.c, fpga_link.c, umh_protocol.c, motion_engine.c.
 *
 * All functions are static inline for zero-cost abstraction - the compiler
 * optimizes them to direct memory access or single instructions.
 */

/* Common mathematical constants - replaces duplicate definitions in:
 * cordic.c, motion_engine.c, spatial_renderer.c, us_calibration.c,
 * demo_engine.c, hologram_engine.c */
#define UMH_PI          3.14159265358979323846f
#define UMH_TWO_PI      6.28318530717958647692f

/* Little-endian read */
static inline uint16_t umh_read_u16_le(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t umh_read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Little-endian write */
static inline void umh_write_u16_le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static inline void umh_write_u32_le(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static inline void umh_write_u64_le(uint8_t *p, uint64_t v)
{
  umh_write_u32_le(p, (uint32_t)v);
  umh_write_u32_le(p + 4, (uint32_t)(v >> 32));
}

#endif /* UMH_UTILS_H */
