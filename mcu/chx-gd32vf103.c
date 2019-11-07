#include <stdint.h>
#include <mcu/gd32vf103.h>

extern int chx_allow_sleep;

void
chx_sleep_mode (int how)
{
  /*TBD*/
  (void)how;
}

void __attribute__((naked))
chx_idle (void)
{
  /*TBD*/
  int sleep_enabled;

  for (;;)
    {
      asm ("lw	%0,%1" : "=r" (sleep_enabled): "m" (chx_allow_sleep));
      if (sleep_enabled)
	{
	  asm volatile ("wfi" : : : "memory");
	  /* NOTE: it never comes here.  Don't add lines after this.  */
	}
    }
}
