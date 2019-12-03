struct RCU {
  volatile uint32_t CTL;
  volatile uint32_t CFG0;
  volatile uint32_t INT;
  volatile uint32_t APB2RST;
  volatile uint32_t APB1RST;
  volatile uint32_t AHBEN;
  volatile uint32_t APB2EN;
  volatile uint32_t APB1EN;
  volatile uint32_t BDCTL;
  volatile uint32_t RSTSCK;
  volatile uint32_t AHBRST;
  volatile uint32_t CFG1;
  uint32_t rsv;
  volatile uint32_t DSV;
};
static struct RCU *const RCU = (struct RCU *)0x40021000;

#define RCU_CTL_HXTALEN  0x00010000
#define RCU_CTL_HXTALSTB 0x00020000
#define RCU_CTL_PLLSTB   0x02000000
#define RCU_CTL_PLLEN    0x01000000
#define RCU_CTL_IRC8MEN  0x00000001

#define RCU_CFG0_ADC_MASK           0x0000c000
#define RCU_CFG0_AHB_APB1_APB2_MASK 0x00003ff0
#define RCU_CFG0_AHB_CKSYS_DIV1     0x00000000
#define RCU_CFG0_APB2_CKAHB_DIV1    0x00000000
#define RCU_CFG0_APB1_CKAHB_DIV2    0x00000400
#define RCU_CFG0_PLLSRC_PLLMF_MASK  0x203d0000
#define RCU_CFG0_PLL_MUL12          0x00280000
#define RCU_CFG0_PLLSRC_HXTAL       0x00010000
#define RCU_CFG0_SCS_MASK           0x00000003
#define RCU_CFG0_SCSS_PLL           0x00000008
#define RCU_CFG0_CKSYSSRC_PLL       0x00000002

#define RCU_CFG1_PREDV0SEL_MASK     0x00010000
#define RCU_CFG1_PREDV0SEL_HXTAL    0x00000000

#define RCU_APB2_GPIOA  0x00000004
#define RCU_APB2_GPIOB  0x00000008
#define RCU_APB2_GPIOC  0x00000010

/* Semantics is exactly same as STM32F103.  */
struct GPIO {
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
};
static struct GPIO *const GPIOA = (struct GPIO *)0x40010800;
static struct GPIO *const GPIOB = (struct GPIO *)0x40010C00;
static struct GPIO *const GPIOC = (struct GPIO *)0x40011000;
