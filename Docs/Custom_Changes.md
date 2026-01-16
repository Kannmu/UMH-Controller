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

![alt text](image.png)

