#include <stdint.h>
#include <mcu/gd32vf103.h>

asm (
	".equ	wfe,0x810\n\t"    /* Not used (yet). */
	".equ	sleepvalue,0x811" /* Not used (yet). */
);

extern int chx_allow_sleep;

void
chx_sleep_mode (int how)
{
  /*TBD*/
  (void)how;
}

void
chx_prepare_sleep_mode (void)
{
  /*TBD*/
}
