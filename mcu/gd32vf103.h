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

#define RCU_CFG0_CKOUT_MASK         0x0f000000
#define RCU_CFG0_ADC_MASK           0x1000c000
#define RCU_CFG0_AHB_APB1_APB2_MASK 0x00003ff0
#define RCU_CFG0_AHB_CKSYS_DIV1     0x00000000
#define RCU_CFG0_APB2_CKAHB_DIV1    0x00000000
#define RCU_CFG0_APB1_CKAHB_DIV2    0x00000400
#define RCU_CFG0_PREDV0_LSB         0x00020000
#define RCU_CFG0_PLLSRC_PLLMF_MASK  0x203d0000
#define RCU_CFG0_PLL_MUL12          0x00280000
#define RCU_CFG0_PLLSRC_HXTAL       0x00010000
#define RCU_CFG0_SCS_MASK           0x00000003
#define RCU_CFG0_SCSS_PLL           0x00000008
#define RCU_CFG0_CKSYSSRC_PLL       0x00000002
#define RCU_CFG0_USBFSPSC_MASK      0x00c00000
#define RCU_CFG0_USBFSPSC_DIV2      0x00c00000
#define RCU_CFG0_ADC_CKAPB2_DIV8    0x0000c000

#define RCU_CFG1_PREDV1_PREDV0_MASK 0x000000ff
#define RCU_CFG1_PREDV0SEL_MASK     0x00010000
#define RCU_CFG1_PREDV0SEL_HXTAL    0x00000000

#define RCU_APB2_GPIOA  0x00000004
#define RCU_APB2_GPIOB  0x00000008
#define RCU_APB2_GPIOC  0x00000010

#define RCU_AHB_USBFS   0x00001000

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

struct FLASH {
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
};

#define FLASH_R_BASE	(0x40000000 + 0x20000 + 0x2000)
static struct FLASH *const FLASH = (struct FLASH *)FLASH_R_BASE;

/* System Control Block */
struct SCB
{
  volatile uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t  SHP[12];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile uint32_t PFR[2];
  volatile uint32_t DFR;
  volatile uint32_t AFR;
  volatile uint32_t MMFR[4];
  volatile uint32_t ISAR[5];
};

#define SCS_BASE 0xE000E000
#define SCB_BASE (SCS_BASE + 0x0D00)
static struct SCB *const SCB = (struct SCB *)SCB_BASE;
#define SCB_SCR_SLEEPDEEP (1 << 2)
#define SCB_AIRCR_SYSRESETREQ 0x04
