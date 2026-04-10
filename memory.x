/* STM32G474RETx — 512 KB Flash, 96 KB SRAM
 *
 * Memory map (RM0440 / datasheet):
 *   FLASH  : 0x0800_0000 – 0x0807_FFFF  (512 KB)
 *   SRAM1  : 0x2000_0000 – 0x2001_3FFF  ( 80 KB)  \  96 KB contiguous via System Bus
 *   SRAM2  : 0x2001_4000 – 0x2001_7FFF  ( 16 KB)  /
 *
 * The CCM SRAM (32 KB) lives at 0x1000_0000 (CPU D-Code only) and is aliased
 * at 0x2001_8000 for DMA/SBUS access.  It is intentionally excluded from the
 * RAM region: including the alias would silently overlap SRAM1/2 addressing,
 * and the direct window is unreachable by DMA.  Place DMA buffers in SRAM1/2.
 */
MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 96K
}

/* Stack at the top of RAM (cortex-m-rt default) */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
