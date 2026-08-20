# Audio playback control

The audio extension keeps the existing CDC framing:

`AA 55 <command> <length> <payload> <checksum> 0D 0A`

The checksum is the low byte of the sum of command, length and payload bytes.

Commands in the `0x20` range are reserved for audio:

| Command | Payload | Meaning |
|---|---:|---|
| `0x20` | none | Get capabilities. The response payload is `uint32 sample_rate`, `uint8 channels`, `uint8 bits_per_sample`, `uint16 reserved`. |
| `0x21` | none | Enter audio mode. |
| `0x22` | even length | Little-endian signed 16-bit mono PCM at 48 kHz. PCM frames do not generate ACK responses. |
| `0x23` | none | Fade out and return to the previous stimulation state. |
| `0x24` | 12 bytes | Three little-endian `float` focus coordinates in metres. |
| `0x25` | 4 bytes | Little-endian `float` level from 0 to 1. |
| `0x26` | none | Get audio status. |
| `0x27` | 1 byte | Mute (`1`) or unmute (`0`). |

Audio mode is mutually exclusive with the legacy stimulation output because TIM1, the five DMA streams and the 60 GPIO outputs are shared. Legacy stimulation commands are rejected while audio is active; exiting audio restores the saved waveform. The current USB device remains CDC-only, so this extension does not add a native Windows UAC endpoint yet.
