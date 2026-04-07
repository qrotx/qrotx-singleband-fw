/* STM32G474RETx — 512 KB Flash, 128 KB SRAM, 32 KB CCM-SRAM
 *
 * Memory map (RM0440 Table 2):
 *   FLASH  : 0x0800_0000 – 0x0807_FFFF  (512 KB)
 *   SRAM1  : 0x2000_0000 – 0x2001_7FFF  (96 KB)   \  contiguous → 128 KB
 *   SRAM2  : 0x2001_8000 – 0x2001_FFFF  (32 KB)   /
 *   CCMRAM : 0x1000_0000 – 0x1000_7FFF  (32 KB, tightly-coupled, DMA-accessible)
 */
MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
}

/* Stack at the top of RAM (cortex-m-rt default) */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* Export CCMRAM extents so linker sections can reference them */
_ccmram_start = ORIGIN(CCMRAM);
_ccmram_end   = ORIGIN(CCMRAM) + LENGTH(CCMRAM);
