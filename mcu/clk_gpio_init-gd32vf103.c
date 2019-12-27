#include <stdint.h>
#include <mcu/gd32vf103.h>

static void __attribute__((used,section(".text.startup.1")))
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
  /* CK_USBFS = 48MHz */
  RCU->CFG0 |= RCU_CFG0_USBFSPSC_DIV2;

  /* CK_ADC, CK_TIMER1xxx, CK_TIMER0, CK_I2S */

  /* PLL setup */
  RCU->CFG0 &= ~RCU_CFG0_PLLSRC_PLLMF_MASK;
  RCU->CFG0 |= RCU_CFG0_PLL_MUL_VALUE | RCU_CFG0_PLLSRC_HXTAL;

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

static void __attribute__((used,section(".text.startup.1")))
gpio_init (void)
{
  RCU->APB2EN  |= RCU_APB2_GPIO;
  RCU->APB2RST = RCU_APB2_GPIO;
  RCU->APB2RST = 0;

#ifdef AFIO_MAPR_SOMETHING
  AFIO->MAPR |= AFIO_MAPR_SOMETHING;
#endif

  /* LED is mandatory.  We configure it always.  */
  GPIO_LED->ODR = VAL_GPIO_LED_ODR;
  GPIO_LED->CRH = VAL_GPIO_LED_CRH;
  GPIO_LED->CRL = VAL_GPIO_LED_CRL;

  /* If there is USB enabler pin and it's independent, we configure it.  */
#if defined(GPIO_USB) && defined(VAL_GPIO_USB_ODR)
  GPIO_USB->ODR = VAL_GPIO_USB_ODR;
  GPIO_USB->CRH = VAL_GPIO_USB_CRH;
  GPIO_USB->CRL = VAL_GPIO_USB_CRL;
#endif

#ifdef GPIO_OTHER
  GPIO_OTHER->ODR = VAL_GPIO_OTHER_ODR;
  GPIO_OTHER->CRH = VAL_GPIO_OTHER_CRH;
  GPIO_OTHER->CRL = VAL_GPIO_OTHER_CRL;
#endif
}
