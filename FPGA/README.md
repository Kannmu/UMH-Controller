# UMH v7 FPGA

`UMH_7.ldf` targets `LCMXO2-2000HC-4MG132C`.  The LPF pin locations come from
`Hardware Design/Netlist_UMH_7_2026-09-02.tel`; `US_TX_01` is `us_tx[0]`.

The design accepts the STM32 SPI1 mode-0 frame format in
`Docs/UMH_v7_FPGA_Interface.md`.  A complete full-mask frame is staged while CS
is low, then copied to the output bank at a carrier wrap.  Status returns one
staging credit and the accepted frame sequence.  `STOP` and `RESET` disable all
ultrasound outputs.

The carrier is a 40 kHz fractional NCO from the 42.5 MHz PA8 reference.  Frame
phase values are quantized from 16 bits to their high 8 bits; levels use an
8-bit carrier-rate PDM gate.  This is the resolution that fits 84 independent
parallel drivers in the selected 2000HC device.  The STM32 profile must report
one FPGA frame credit and eight implemented phase bits before this bitstream is
used by host software.

`ws2812_stream.v` transmits four GRB pixels at 42.5 MHz timing.  PDM inputs are
sampled at 3.04 MHz and `spi_mic_stream.v` returns the latest two 16-bit sample
words to CH347T in SPI mode 0.  The host-side CH347 acquisition framing is not
defined elsewhere in this repository, so it remains a repeated 32-bit raw
sample transport.

`sim/tb_umh_fpga_top.v` verifies the SPI status byte order, a full 84-channel
frame, carrier-boundary activation and the accepted-sequence status field.
