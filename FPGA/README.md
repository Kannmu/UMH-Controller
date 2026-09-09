# UMH v7 FPGA

`UMH_7.ldf` targets `LCMXO2-2000HC-4MG132C`.  The LPF pin locations come from
`Hardware Design/Netlist_UMH_7_2026-09-02.tel`; `US_TX_01` is `us_tx[0]`.

The design accepts the STM32 SPI1 mode-0 frame format in
`Docs/UMH_v7_FPGA_Interface.md`.  A complete full-mask frame is staged while CS
is low, then copied to the output bank at a carrier wrap.  Status returns one
staging credit and the accepted frame sequence.  `STOP` and `RESET` disable all
ultrasound outputs.

The STM32 control SPI1 link remains 42.5 MHz (170 MHz PCLK2 divided by 4).
PA8 now supplies an 8 MHz HSE MCO to the MachXO2 PLL, which generates a 128 MHz
internal clock. The carrier is a 40 kHz 32-bit DDS in that domain. Every channel
uses the complete 16-bit phase and 8-bit level words received over SPI. The
level is a per-carrier-period duty threshold; `us_tx` remains a digital 0/1
signal, producing a 0 V/3.3 V square wave on every external channel.

`ws2812_stream.v` transmits four GRB pixels at the 128 MHz PLL timing.  PDM inputs are
sampled at 3.04 MHz and `spi_mic_stream.v` returns the latest two 16-bit sample
words to CH347T in SPI mode 0.  The host-side CH347 acquisition framing is not
defined elsewhere in this repository, so it remains a repeated 32-bit raw
sample transport.

`sim/tb_umh_fpga_top.v` verifies the SPI status byte order, a full 84-channel
frame, carrier-boundary activation and the accepted-sequence status field.

## Diamond build

The checked-in source is built with Lattice Diamond 3.13. Set the license path
before invoking the batch flow from the repository root:

```powershell
$env:LM_LICENSE_FILE = 'D:\Software\Lattice Diamond\diamond\3.13\license\license.dat'
Set-Location FPGA
& 'D:\Software\Lattice Diamond\diamond\3.13\bin\nt64\pnmainc.exe' -batch build_full.tcl
```

`build_full.tcl` opens `UMH_7.ldf`, runs Synthesis, Map, PAR and Export for
implementation `UMH_7_1`, and writes generated results under `UMH_7_1/`.
Generated files are disposable and are intentionally excluded from source
control. The input clock is the 8 MHz STM32 PA8 HSE MCO; the MachXO2
`EHXPLLL` generates the 128 MHz `fpga_clk` domain. The PLL lock signal gates
all ultrasound outputs.
