# RAM Optimization Summary

## Results
- **Before**: 111456 B / 112 KB (97.18%)
- **After**: 96448 B / 112 KB (84.10%)
- **Saved**: 15008 bytes (14.7 KB, 13.1% reduction)

## Changes Implemented

### 1. USB CDC Buffer Shrinking (3968 B saved)
- `UserRxBufferFS`: 2048 → 64 bytes
- `UserTxBufferFS`: 2048 → 64 bytes
- **Files**: `USB_Device/App/usbd_cdc_if.h`
- **Rationale**: CDC_DATA_FS_MAX_PACKET_SIZE is 64 B; oversized buffers wasted RAM

### 2. USB Descriptor Buffer Shrinking (448 B saved)
- `USBD_MAX_STR_DESC_SIZ`: 512 → 64 bytes
- **Files**: `USB_Device/Target/usbd_conf.h`
- **Rationale**: Longest string descriptor only needs 44 B

### 3. Storage Buffer Consolidation (4096 B saved)
- Merged `storage_response[2048]`, `storage_data[2048]`, `storage_records[2048]` into single union
- **Files**: `Core/Src/app_freertos.c`
- **Rationale**: FLASH_LIST/READ/WRITE never overlap; records array only used by LIST

### 4. Flash Store Snapshot Merge (2048 B saved)
- Merged two separate 2048 B snapshot buffers in `rotate_metadata()` and `compact_data()` into one shared buffer
- **Files**: `Core/Src/flash_store.c`
- **Rationale**: Functions never overlap (compact may call rotate, but rotate never calls compact)

### 5. Calibration Dead Arrays Removal (1912 B saved)
- Removed `cal_profile_i[512 B]` and `cal_profile_q[512 B]` - never written after commit 0117b44
- Removed `cal_dump2[888 B]` - only writer `cal_fill_dump2()` never called
- Removed `cal_fill_dump2()` function (~40 lines)
- Dump section reads now return zeros
- **Files**: `Core/Src/us_calibration.c`

### 6. Device Profile Duplicate Removal (1118 B saved)
- Removed unused static `profile` in `device_profile.c`
- **Rationale**: `active_profile` always points to app_freertos's copy after init; fallback never used

### 7. FreeRTOS Timer Task Removal (~1400 B saved)
- Set `configUSE_TIMERS` = 0
- Set `configUSE_OS2_TIMER` = 0
- Set `INCLUDE_xTimerPendFunctionCall` = 0  
- Set `configUSE_OS2_EVENTFLAGS_FROM_ISR` = 0 (depends on timer API)
- **Files**: `Core/Inc/FreeRTOSConfig.h`
- **Rationale**: No timers created; timer task + queue + TCB wasted RAM

### 8. Stack Copy Elimination (commented, not measured separately)
- Changed `queue_storage_request()` from `memcpy()` to structure assignment
- **Files**: `Core/Src/app_freertos.c`
- **Rationale**: Structure assignment compiles to same code but clearer intent; 2066 B frame still copied by osMessageQueuePut

## Code Quality Improvements
- Added detailed comments explaining buffer consolidations and removed code
- Removed dead function `cal_fill_dump2` (~40 lines)
- Simplified `cal_dump_read_section1()` from byte-by-byte reconstruction to memset

## Testing Recommendations
1. Verify USB CDC communication still works (RX/TX with 64 B buffers)
2. Test flash storage operations (LIST/READ/WRITE)
3. Run calibration and verify dump sections return zeros correctly
4. Confirm device profile operations work without fallback static
5. Verify no timer-dependent functionality was broken

## Build Configuration
- Toolchain: GCC 10.3-2021.10
- Optimization: -Og (debug)
- Target: STM32G491 (112 KB RAM, 256 KB Flash)
