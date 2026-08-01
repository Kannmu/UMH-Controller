# Custom_Changes After Code Regeneration from STM32CubeMX

1. Replace `DMA_NORMAL` to `DMA_CIRCULAR`

2. Set all `DMA_FIFOMODE` to `DMA_FIFOMODE_DISABLE`

3. Modify `STM32H750VBTx_FLASH.ld` file with the following content:

```c
  .storage_buffer (NOLOAD) : ALIGN(4) {
        KEEP(*(.storage_buffer))
  } >RAM_D2
```

Insert the code above after the `.ARM` section in the linker script (.ld file), below the image location.

  .stim_type_registry :
  {
    . = ALIGN(4);
    _stim_type_registry_start = .;
    KEEP(*(.stim_type_registry))
    _stim_type_registry_end = .;
  } >FLASH

  .demo_registry :
  {
    . = ALIGN(4);
    _demo_registry_start = .;
    KEEP(*(.demo_registry))
    _demo_registry_end = .;
  } >FLASH
  
![alt text](image.png)

