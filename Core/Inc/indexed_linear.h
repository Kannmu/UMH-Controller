#pragma once

#include <stdint.h>

#define INDEXED_LINEAR_SAMPLE_COUNT       200U
#define INDEXED_LINEAR_PAYLOAD_LENGTH     239U
#define INDEXED_LINEAR_SAMPLE_COUNT_OFFSET 25U
#define INDEXED_LINEAR_CONTROL_FLAGS_OFFSET 26U
#define INDEXED_LINEAR_ORDER_OFFSET       27U
#define INDEXED_LINEAR_CRC_OFFSET         235U
#define INDEXED_LINEAR_SUPPORTED_FLAGS    0U

/* CRC-32/IEEE: reflected polynomial, init/final XOR 0xffffffff. */
static inline uint32_t Indexed_Linear_CRC32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xffffffffU;
    for (uint32_t i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; bit++)
        {
            uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xedb88320U & mask);
        }
    }
    return crc ^ 0xffffffffU;
}

static inline int Indexed_Linear_Validate_Order(const uint8_t *order,
                                                uint8_t sample_count,
                                                uint8_t control_flags)
{
    if (order == 0 || sample_count != INDEXED_LINEAR_SAMPLE_COUNT ||
        (control_flags & (uint8_t)~INDEXED_LINEAR_SUPPORTED_FLAGS) != 0U)
    {
        return 0;
    }

    uint32_t seen[(INDEXED_LINEAR_SAMPLE_COUNT + 31U) / 32U] = {0};
    for (uint32_t i = 0U; i < INDEXED_LINEAR_SAMPLE_COUNT; i++)
    {
        uint8_t spatial_index = order[i];
        if (spatial_index >= INDEXED_LINEAR_SAMPLE_COUNT)
        {
            return 0;
        }
        uint32_t word = spatial_index / 32U;
        uint32_t bit = 1U << (spatial_index % 32U);
        if ((seen[word] & bit) != 0U)
        {
            return 0;
        }
        seen[word] |= bit;
    }
    return 1;
}

static inline int Indexed_Linear_Validate_Payload(const uint8_t *payload,
                                                  uint8_t payload_length)
{
    if (payload == 0 || payload_length != INDEXED_LINEAR_PAYLOAD_LENGTH)
    {
        return 0;
    }

    uint32_t received_crc = (uint32_t)payload[INDEXED_LINEAR_CRC_OFFSET] |
        ((uint32_t)payload[INDEXED_LINEAR_CRC_OFFSET + 1U] << 8U) |
        ((uint32_t)payload[INDEXED_LINEAR_CRC_OFFSET + 2U] << 16U) |
        ((uint32_t)payload[INDEXED_LINEAR_CRC_OFFSET + 3U] << 24U);
    if (received_crc != Indexed_Linear_CRC32(payload, INDEXED_LINEAR_CRC_OFFSET))
    {
        return 0;
    }

    return Indexed_Linear_Validate_Order(
        &payload[INDEXED_LINEAR_ORDER_OFFSET],
        payload[INDEXED_LINEAR_SAMPLE_COUNT_OFFSET],
        payload[INDEXED_LINEAR_CONTROL_FLAGS_OFFSET]);
}
