# UMH v7 FPGA

`UMH_7.ldf` targets `LCMXO2-2000HC-4MG132C`. The source clock is the 8 MHz STM32 PA8 HSE MCO; the checked-in `EHXPLLJ` generates a 64 MHz FPGA clock (512 MHz VCO / CLKOP_DIV=8). Keep `UMH_7.lpf` as the source constraint file. Files generated under `UMH_7_1/` are build outputs and may be deleted and regenerated.

The design implements the STM32 SPI1 mode-0 control protocol, the 84-channel ultrasound output engine, the parallel four-WS2812 output, microphone clocking, and the microphone SPI return stream. All four WS2812 devices share one DI line, so the stream intentionally sends one 24-bit GRB value and all four devices receive the same colour. WS2812C-2020-V6 requires a reset low time of at least 280 us; the stream uses 312.5 us. Because this board's PLL LOCK output does not assert even though the measured PLL frequency is correct, the output engine is gated by running/stop_event, not by LOCK.

## Reproducible Diamond build on Windows

Use the wrapper from the FPGA directory. It sets the Diamond environment, feeds `build_full.tcl` to Diamond's Tcl shell, captures the complete transcript in `UMH_7_1/pnmainc.log`, runs Synthesis, Map, PAR, Bitgen and Jedecgen, and checks that all required outputs exist.

```powershell
Set-Location 'D:\Data\OneDrive\Projects\UMH\Software\UMH Controller\FPGA'
cmd /d /c run_diamond.cmd
```

Run only one Diamond build at a time. A successful wrapper exit requires the completion marker `UMH_7_1 Diamond build completed`, `PAR_SUMMARY::Run status = Completed`, `PAR_SUMMARY::Number of errors = 0`, and fresh `.ncd`, `.par`, `.twr`, `.bit`, and `.jed` files in `UMH_7_1/`. The files normally used for programming are `UMH_7_1\UMH_7_UMH_7_1.bit` and `UMH_7_1\UMH_7_UMH_7_1.jed`.

If a previous interrupted run left a locked `.ngd` or `.ncd`, close Diamond and any `pnmainc`, synthesis, map, PAR or trace process, then retry with:

```powershell
$env:CLEAN_DIAMOND = '1'
cmd /d /c run_diamond.cmd
Remove-Item Env:CLEAN_DIAMOND
```

The clean option removes stale generated implementation files before starting; it never removes source RTL, the project file, or constraints. If the wrapper still fails, inspect the last error in `UMH_7_1/pnmainc.log` and compare it with `UMH_7_1/UMH_7_UMH_7_1.mrp` and the trace report before changing RTL.

Do not invoke `synthesis.exe` directly and do not use `pnmainc -batch`. Diamond's Tcl shell must be launched through `run_diamond.cmd`, otherwise its message-data and installation paths are incomplete. For the MachXO2 PLL keep `FEEDBK_PATH("INT_DIVA")` and the phase-aligned `CLKOP_CPHASE`; `INT_OP` is rejected by Map.
