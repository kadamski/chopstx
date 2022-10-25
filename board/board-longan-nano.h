#define BOARD_NAME "Longan-nano"
#define BOARD_ID    0xe65dace7
/* echo -n "Longan-nano" | sha256sum | sed -e 's/^.*\(........\)  -$/\1/' */

#define MCU_CORE_BUMBLEBEE 1
#define MCU_GD32VF1        1
#define MCU_STM32F1_GD32F1 1    /* ADC is as same as GD32F1 */

/*
 * XTAL clock =  8 MHz
 * Core clock = 96 MHz
 * AHB  clock = 96 MHz
 * APB1 clock = 48 MHz
 * APB2 clock = 96 MHz
 * USB clock  = 48 MHz
 * ADC clock  = 12 MHz
 */

/* HXTAL 8MHz input => PLL output = 8 * 12 = 96 MHz */
#define RCU_CFG0_PLL_MUL_VALUE RCU_CFG0_PLL_MUL12

#define GPIO_LED   GPIOC
#define GPIO_LED_CLEAR_TO_EMIT     13 /* LED Red */
#define GPIO_USB   GPIOA
#define GPIO_OTHER GPIOB

/*
 * Port A setup.
 *
 * PA0  - input with pull-up: AN0 for NeuG
 * PA1  - Push-pull output 2MHz 1 default (LED green 0: ON 1: OFF)
 * PA2  - Push-pull output 2MHz 1 default (LED blue  0: ON 1: OFF)
 * PA3  - input with pull-up: AN3 for NeuG
 * PA4  - <Can be used for GPIO>: Ack button
 * PA5  - AF output push-pull 50MHz: SPI0_SCK
 * PA6  - Input pull-up: SPI0_MISO (master)
 * PA7  - AF output push-pull 50MHz: SPI0_MOSI (master)
 * PA8  - (USBFS_SOF)
 * PA9  - AF output push-pull 2MHz: USART0_TX (USBFS_VBUS)
 * PA10 - Input pull-up: USART0_RX0 (USBFS_ID)
 * PA11 - Push Pull output 10MHz 0 default (until USB enabled???) (USBDM)
 * PA12 - Push Pull output 10MHz 0 default (until USB enabled???) (USBDP)
 * ------------------------ Default
 * PAx  - input with pull-up.
 *
 * NOTE: It's USB device bus power configuration, where we don't
 * monitor VBUS voltage change.  For self powered system, we need to
 * monitor VBUS pin for detecting USB connection.  Supporting self
 * powered system, you need to change the USB driver, and when doing
 * so, PA9 should be used for USBFS_VBUS.
 */
#define VAL_GPIO_USB_ODR   0xFFFFE6FF
#define VAL_GPIO_USB_CRL   0xB8B88228      /*  PA7...PA0 */
#define VAL_GPIO_USB_CRH   0x888118A8      /* PA15...PA8 */

#define RCU_APB2_GPIO      (RCU_APB2_GPIOA|RCU_APB2_GPIOB|RCU_APB2_GPIOC)

/*
 * PB0 - Push-pull output 50MHz 1 default (OLED D/C#)
 * PB1 - Push-pull output 10MHz 1 default (OLED RST#)
 * PB2 - Push-pull output 50MHz 1 default (OLED CS#)
 */
#define VAL_GPIO_OTHER_ODR 0xFFFFFFFF
#define VAL_GPIO_OTHER_CRL 0x88888313      /*  PB7...PB0 */
#define VAL_GPIO_OTHER_CRH 0x88888888      /* PB15...PB8 */

/*
 * PC13 - Push-pull output 2MHz 1 default (LED red  0: ON 1: OFF)
 */
#define VAL_GPIO_LED_ODR 0xFFFFFFFF
#define VAL_GPIO_LED_CRL 0x88888888      /*  PC7...PC0 */
#define VAL_GPIO_LED_CRH 0x88288888      /* PC15...PC8 */

/*
 * Board specific information other than clock and GPIO initial
 * setting should not be in board-*.h, but each driver should include
 * such specific information by itself.
 */
