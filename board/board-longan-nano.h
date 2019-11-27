#define BOARD_NAME "Longan-nano"
#define BOARD_ID    0xe65dace7
/* echo -n "Longan-nano" | sha256sum | sed -e 's/^.*\(........\)  -$/\1/' */

#define MCU_CORE_BUMBLEBEE 1
#define MCU_GD32VF1        1
/*
 * XTAL clock =  8 MHz
 * Core clock = 96 MHz
 * AHB  clock = 96 MHz
 * APB1 clock = 48 MHz
 * APB2 clock = 96 MHz
 * USB clock  = 48 MHz
 * ADC clock  = ???
 */

/*
#define RCU_CFG0_PLL_MUL_VALUE RCU_CFG0_PLL_MUL12
*/

#define GPIO_LED_BASE   GPIOA_BASE
#define GPIO_LED_CLEAR_TO_EMIT     1 /* LED Green */
#define GPIO_USB_BASE   GPIOA_BASE
#define GPIO_OTHER_BASE GPIOC_BASE

/*
 * Port A setup.
 *
 * PA1  - Push-pull output 2MHz 1 default (LED green 0: ON 1: OFF)
 * PA2  - Push-pull output 2MHz 1 default (LED blue  0: ON 1: OFF)
 * PA3  - 
 * PA4  - <Can be used for GPIO>
 * PA5  - AF output open-drain 50MHz: SPI0_SCK
 * PA6  - Input pull-up: SPI0_MISO (master)
 * PA7  - AF output open-drain 50MHz: SPI0_MOSI (master)
 * PA8  - (USBFS_SOF)
 * PA9  - AF output open-drain 2MHz: USART0_TX (USBFS_VBUS)
 * PA10 - Input pull-up: USART0_RX0 (USBFS_ID)
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled???) (USBDM)
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled???) (USBDP)
 * ------------------------ Default
 * PAx  - input with pull-up.
 */
#define VAL_GPIO_LED_ODR   0xFFFFE6FF
#define VAL_GPIO_LED_CRL   0xF8F88228      /*  PA7...PA0 */
#define VAL_GPIO_LED_CRH   0x888118E8      /* PA15...PA8 */

#define RCC_APB2_GPIO      (RCU_APB2_GPIOA|RCU_APB2_GPIOA)

/*
 * PC13 - Push-pull output 2MHz 1 default (LED red  0: ON 1: OFF)
 */
#define VAL_GPIO_OTHER_ODR 0xFFFFFFFF
#define VAL_GPIO_OTHER_CRL 0x88888888      /*  PC7...PC0 */
#define VAL_GPIO_OTHER_CRH 0x88288888      /* PC15...PC8 */

/*
 * Board specific information other than clock and GPIO initial
 * setting should not be in board-*.h, but each driver should include
 * such specific information by itself.
 */
