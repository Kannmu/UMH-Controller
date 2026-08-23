# Custom_Changes After Code Regeneration from STM32CubeMX

1. Replace `DMA_NORMAL` to `DMA_CIRCULAR`

2. Set all `DMA_FIFOMODE` to `DMA_FIFOMODE_DISABLE`

3. Modify `STM32H750VBTx_FLASH.ld` with the shared waveform sections:

```c
  .storage_buffer (NOLOAD) : ALIGN(32) {
        KEEP(*(.storage_buffer))
  } >RAM_D1

  .waveform_staging (NOLOAD) : ALIGN(32) {
        KEEP(*(.waveform_staging))
  } >RAM_D1
```

`Waveform_Storage` and `Waveform_Staging` are shared by ordinary stimulation
and sequence playback. Keep both sections after the `.ARM` section in the
linker script (.ld file), below the image location. Do not add a separate
`.sequence_waveforms` section.

![alt text](image.png)

4. Preserve the MPU region at `0x30000000` as non-cacheable. Both waveform
   blocks are in cacheable D1 SRAM; `dma_manager.c` cleans the active block
   before DMA activation.

5. Preserve `SCB_EnableICache()` and `SCB_EnableDCache()` after `MPU_Config()`.

6. Preserve the fixed 400 MHz system clock in `SystemClock_Config()`:
   all supported STM32H750 revisions use VOS1, with the PLL configured from the
   25 MHz HSE. TIM1 derives its period from `DMA_SAMPLING_FREQ`, so the
   ultrasound output remains at 4 MHz sampling and 40 kHz carrier.

7. Release builds use `-O3 -fno-math-errno`. Do not enable LTO with the bundled
   GCC 10.3 toolchain; its handling of STM32 HAL weak symbols causes link errors.

8. USB receive callbacks must only call `Comm_Queue_Received_Data()`. Parsing,
   ADC polling, and waveform rendering run from `Comm_Task()` in the main loop.

