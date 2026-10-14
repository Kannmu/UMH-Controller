#ifndef USBD_CDC_AUDIO_H
#define USBD_CDC_AUDIO_H

#include "usbd_def.h"

#define UAC_CONTROL_INTERFACE 2u
#define UAC_STREAM_INTERFACE 3u
#define UAC_OUT_EP 0x03u
#define UAC_PMA_OUT 0x180u
/* G4 RX PMA rounds sizes >62 bytes up to 32-byte blocks. */
#define UAC_PMA_BYTES 128u
#define UAC_CONFIG_DESC_SIZE 183u

extern USBD_ClassTypeDef USBD_CDC_AUDIO;
/* Suspend/disconnect callbacks call this in interrupt context.  It invalidates
 * queued audio; the protocol task stops only the UAC-owned output. */
void USBD_CDC_AUDIO_Suspend(void);
void USBD_CDC_AUDIO_Resume(void);

#endif
