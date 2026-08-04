#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "indexed_linear.h"

int main(void)
{
    static const uint8_t crc_test[] = "123456789";
    if (Indexed_Linear_CRC32(crc_test, 9U) != 0xcbf43926U)
    {
        fprintf(stderr, "CRC-32/IEEE known-vector test failed\n");
        return 1;
    }

    uint8_t order[INDEXED_LINEAR_SAMPLE_COUNT];
    for (uint32_t i = 0U; i < INDEXED_LINEAR_SAMPLE_COUNT; i++)
    {
        order[i] = (uint8_t)((i * 73U) % INDEXED_LINEAR_SAMPLE_COUNT);
    }
    if (!Indexed_Linear_Validate_Order(order, INDEXED_LINEAR_SAMPLE_COUNT, 0U))
    {
        fprintf(stderr, "valid permutation was rejected\n");
        return 1;
    }

    order[199] = order[0];
    if (Indexed_Linear_Validate_Order(order, INDEXED_LINEAR_SAMPLE_COUNT, 0U))
    {
        fprintf(stderr, "duplicate index was accepted\n");
        return 1;
    }
    uint8_t last_index = (uint8_t)((199U * 73U) % INDEXED_LINEAR_SAMPLE_COUNT);
    order[199] = 200U;
    if (Indexed_Linear_Validate_Order(order, INDEXED_LINEAR_SAMPLE_COUNT, 0U))
    {
        fprintf(stderr, "out-of-range index was accepted\n");
        return 1;
    }
    order[199] = last_index;
    if (Indexed_Linear_Validate_Order(order, 199U, 0U) ||
        Indexed_Linear_Validate_Order(order, INDEXED_LINEAR_SAMPLE_COUNT, 1U))
    {
        fprintf(stderr, "invalid metadata was accepted\n");
        return 1;
    }

    uint8_t payload[INDEXED_LINEAR_PAYLOAD_LENGTH] = {0};
    payload[0] = 5U;
    payload[INDEXED_LINEAR_SAMPLE_COUNT_OFFSET] = INDEXED_LINEAR_SAMPLE_COUNT;
    memcpy(&payload[INDEXED_LINEAR_ORDER_OFFSET], order, sizeof(order));
    uint32_t crc = Indexed_Linear_CRC32(payload, INDEXED_LINEAR_CRC_OFFSET);
    payload[INDEXED_LINEAR_CRC_OFFSET] = (uint8_t)crc;
    payload[INDEXED_LINEAR_CRC_OFFSET + 1U] = (uint8_t)(crc >> 8U);
    payload[INDEXED_LINEAR_CRC_OFFSET + 2U] = (uint8_t)(crc >> 16U);
    payload[INDEXED_LINEAR_CRC_OFFSET + 3U] = (uint8_t)(crc >> 24U);

    if (!Indexed_Linear_Validate_Payload(payload, sizeof(payload)) ||
        Indexed_Linear_Validate_Payload(payload, sizeof(payload) - 1U))
    {
        fprintf(stderr, "valid payload or payload length validation failed\n");
        return 1;
    }
    payload[INDEXED_LINEAR_ORDER_OFFSET] ^= 1U;
    if (Indexed_Linear_Validate_Payload(payload, sizeof(payload)))
    {
        fprintf(stderr, "corrupted payload was accepted\n");
        return 1;
    }

    printf("indexed Linear validation and CRC tests passed\n");
    return 0;
}
