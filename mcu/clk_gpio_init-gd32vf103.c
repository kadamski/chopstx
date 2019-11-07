#include <stdint.h>
#include <mcu/gd32vf103.h>

static void __attribute__((used))
clock_init (void)
{
  /* HXTAL setup */
  RCU->CTL |= RCU_CTL_HXTALEN;
  while (!(RCU->CTL & RCU_CTL_HXTALSTB))
    ;

  RCU->CFG0 &= ~RCU_CFG0_AHB_APB1_APB2_MASK;

  /* CK_AHB = CK_SYS */
  RCU->CFG0 |= RCU_CFG0_AHB_CKSYS_DIV1;
  /* CK_APB2 = CK_AHB */
  RCU->CFG0 |= RCU_CFG0_APB2_CKAHB_DIV1;
  /* CK_APB1 = CK_AHB/2 */
  RCU->CFG0 |= RCU_CFG0_APB1_CKAHB_DIV2;

  /* CK_ADC, CK_TIMER1xxx, CK_TIMER0, CK_I2S */

  /* PLL setup */
  RCU->CFG0 &= ~RCU_CFG0_PLLSRC_PLLMF_MASK;
  RCU->CFG0 |= RCU_CFG0_PLL_MUL12 | RCU_CFG0_PLLSRC_HXTAL;

  RCU->CFG1 &= ~RCU_CFG1_PREDV0SEL_MASK;
  RCU->CFG1 |= RCU_CFG1_PREDV0SEL_HXTAL;

  RCU->CTL |= RCU_CTL_PLLEN;

  while (!(RCU->CTL & RCU_CTL_PLLSTB))
    ;

  /* Select PLL as system clock */
  RCU->CFG0 &= ~RCU_CFG0_SCS_MASK;
  RCU->CFG0 |= RCU_CFG0_CKSYSSRC_PLL;

  /* Wait until PLL is selected as system clock */
  while (!(RCU->CFG0 & RCU_CFG0_SCSS_PLL))
    ;

  /* Stop IRC8M */
  RCU->CTL &= ~RCU_CTL_IRC8MEN;

  /* Flash setup: TBD */
}

static void __attribute__((used))
gpio_init (void)
{
  /* Enable GPIOC */
  RCU->APB2EN  |= RCU_APB2_GPIOC;
  RCU->APB2RST = RCU_APB2_GPIOC;
  RCU->APB2RST = 0;

  /* Configure GPIOC */
  GPIOC->CRH = 0x44244444;

  /* LED ON */
  GPIOC->ODR &= ~(1 << 13);
}
