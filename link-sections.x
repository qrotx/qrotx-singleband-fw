/* link-sections.x — custom linker sections for qrotx-singleband-fw
 *
 * Adds a .ccmram section placed in the Core-Coupled Memory (CCMRAM) at
 * 0x1000_0000 (32 KB, DMA-accessible on STM32G474).
 *
 * NOLOAD: the startup copy-loop does NOT initialise this region from Flash.
 * Variables placed here must be either:
 *   a) zero-initialised (initialiser = 0 / [0; N]), or
 *   b) written by the application before first read.
 * DMA ping-pong buffers always satisfy (b), so NOLOAD is correct here.
 *
 * INSERT AFTER .bss places this section after the zero-BSS region in the
 * final link map without disturbing any cortex-m-rt internal ordering.
 */
SECTIONS
{
  .ccmram (NOLOAD) : ALIGN(4)
  {
    *(.ccmram .ccmram.*);
    . = ALIGN(4);
  } > CCMRAM
}
INSERT AFTER .bss;
