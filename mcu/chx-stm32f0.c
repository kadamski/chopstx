#include <stdint.h>
#include <mcu/cortex-m.h>
#define MCU_STM32F0
#include <mcu/stm32.h>

extern int chx_allow_sleep;

void
chx_sleep_mode (int how)
{
  PWR->CR |= PWR_CR_CWUF;
  PWR->CR &= ~(PWR_CR_PDDS|PWR_CR_LPDS);

  if (how == 0 || how == 1 /* Sleep only (not deepsleep) */)
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP;
  else
    {			   /* Deepsleep */
      /* how == 2: deepsleep but regulator ON */
      if (how == 3)
	PWR->CR |= PWR_CR_LPDS;	/* regulator low-power mode */
      else if (how == 4)
	PWR->CR |= PWR_CR_PDDS;	/* Power down: All OFF     */

      SCB->SCR |= SCB_SCR_SLEEPDEEP;
    }
}

int
chx_prepare_sleep_mode (void)
{
  return chx_allow_sleep;
}
