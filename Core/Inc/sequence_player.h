#pragma once

#include <stdint.h>

#include "sequence_protocol.h"

void Sequence_Init(void);
void Sequence_Task(void);
int Sequence_Begin_Configuration(const SequenceDescriptor *descriptor);
int Sequence_Upload_States(uint16_t first_state, uint8_t count,
                           const uint8_t *phase_offsets);
int Sequence_Commit(void);
void Sequence_Push_Data(uint32_t packet_sequence, const int16_t *samples,
                        uint16_t sample_count);
void Sequence_Abort(void);
int Sequence_Is_Active(void);
void Sequence_Get_Capabilities(SequenceCapabilities *capabilities);
void Sequence_Get_Status(SequenceStatus *status, uint32_t comm_rx_dropped_bytes);
