# UMH v7 FPGA

`UMH_7.ldf` targets `LCMXO2-2000HC-4MG132C`.  The LPF pin locations come from
`Hardware Design/Netlist_UMH_7_2026-09-02.tel`; `US_TX_01` is `us_tx[0]`.

The design accepts the STM32 SPI1 mode-0 frame format in
`Docs/UMH_v7_FPGA_Interface.md`. A complete full-mask frame is written to one
16-bit staging RAM while CS is low. At a carrier wrap the FPGA snapshots that
RAM into 84 parallel start/end phase registers, then evaluates all outputs
in parallel. `level=0` disables a channel, so there is no separate enable byte.
Status returns one staging credit and the accepted frame sequence. `STOP` and
`RESET` disable all ultrasound outputs.

The STM32 control SPI1 link remains 42.5 MHz (170 MHz PCLK2 divided by 4).
PA8 supplies an 8 MHz HSE MCO to the MachXO2 PLL, which generates a 128 MHz
internal clock. The carrier is a 40 kHz 32-bit DDS in that domain. Every
channel uses an 8-bit phase and an 8-bit level. The phase code has 256 values
per carrier cycle (1.40625 degrees per code); the output edge is still aligned
to the 128 MHz clock. The level is a per-carrier-period duty threshold and
`level=255` is treated as 100% high. `us_tx` remains a digital 0/1 signal,
producing a 0 V/3.3 V square wave on every external channel.

The active bank consumes 84 x 16 = 1344 flip-flops. The staging RAM is a
dual-clock 16-bit x 84 memory with `syn_ramstyle="block_ram"`; its write port is
in the SPI clock domain and its read port is in the PLL clock domain. A one-cycle
read pipeline is included before the 84-word load, so the RAM output is never
sampled on the wrong address. This removes the old 42-cycle output scan and its
channel-to-channel phase jitter.

`ws2812_stream.v` transmits the RGB output at the 128 MHz PLL timing. The shared
4 MHz-class microphone clock is sampled on both edges, producing four 16-bit PDM
words. `spi_mic_stream.v` returns a repeated 12-byte SPI mode-0 packet to CH347T;
each three-byte record is `source_id (0..3), sample[15:8], sample[7:0]`. IDs 0/1
are the rising/falling-edge channels on `MIC_DATA_0`, and IDs 2/3 are the same
channels on `MIC_DATA_1`.

`sim/tb_umh_fpga_top.v` verifies the SPI status byte order, a full 84-channel
frame, carrier-boundary activation and the accepted-sequence status field.

## Diamond build

The checked-in source is built with Lattice Diamond 3.13. The reliable Windows
entry point is `run_diamond.cmd`, which handles the spaces in this repository's
path, sets `FOUNDRY`, `LSC_DIAMOND`, `PATH` and `LM_LICENSE_FILE`, and captures
the complete Tcl transcript in `UMH_7_1/pnmainc.log`:

```powershell
Set-Location FPGA
cmd /d /c run_diamond.cmd
```

`build_full.tcl` opens `UMH_7.ldf`, runs Synthesis, Map, PAR and Export for
implementation `UMH_7_1`, and writes generated results under `UMH_7_1/`.
Generated files are disposable and are intentionally excluded from source
control. The input clock is the 8 MHz STM32 PA8 HSE MCO; the MachXO2
`EHXPLLJ` generates the 128 MHz `fpga_clk` domain. The PLL lock signal gates
all ultrasound outputs.

Do not invoke `synthesis.exe` directly from PowerShell. It requires the
Diamond installation directory and message-data paths established by the
wrapper; otherwise it can exit with `msgindex.xml` errors. Do not use
`pnmainc -batch`: this executable is a Tcl shell, so pipe the script into it
as `run_diamond.cmd` does. A successful run must contain the completion marker
`UMH_7_1 Diamond build completed` and produce Map/PAR/Trace reports under
`UMH_7_1/`. If the log stops after Synthesis, inspect the last tool message
before changing RTL; Map/PAR can be memory intensive on this design.

The wrapper returns nonzero when Tcl fails, the completion marker is missing,
or Map did not create `UMH_7_UMH_7_1_map.ncd`. Diamond may return from `prj_run`
after a milestone error, so the completion marker alone does not prove that
every stage passed. Run one Diamond instance at a time and inspect
`UMH_7_1/pnmainc.log` plus `UMH_7_1/UMH_7_UMH_7_1.mrp`; a complete build must
have fresh `.ncd`, `.par`, `.twr`, and `.bit` files.

The current Map report is the authoritative resource measurement. The checked-in
RTL uses a dual-bank 512 x 84 event-mask table in EBR and updates all 84 outputs
with one parallel XOR operation per phase event. The latest Diamond run maps
successfully at 1,386 registers, 1,006 LUT4 equivalents and 5 EBRs on the
LCMXO2-2000HC. The real-time event path is separated from the DDS carry chain;
the remaining 128 MHz setup violations are in the background frame-builder
read/modify/write and must be closed before producing a production bitstream.
The run wrapper currently leaves Export disabled, so the generated reports are
valid for synthesis, map, PAR and trace analysis but no new `.bit` file is
claimed.

For MachXO2 `EHXPLLJ`, use `FEEDBK_PATH("INT_DIVA")` with the phase-aligned
CLKOP feedback setting in the checked-in primitive. `INT_OP` is rejected by
Map, and an incompatible `CLKOP_CPHASE` makes the feedback phase illegal.
Keep the source 8 MHz clock constraint in `UMH_7.lpf`;
generated implementation LPFs are disposable.

保持FPGA及其子文件夹下文件整齐，不会存在多版本文件共存的干扰现象。

