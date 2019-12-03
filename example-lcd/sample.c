#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>

#include "board.h"
#include <contrib/spi.h>
#include <contrib/usart.h>

#include <mcu/gd32vf103.h>

void
set_led_b (void)
{
  GPIOA->ODR |= (1 << 1);
  GPIOA->ODR &= ~(1 << 2);
  GPIOC->ODR |= (1 << 13);
}

void
set_led_g (void)
{
  GPIOA->ODR &= ~(1 << 1);
  GPIOA->ODR |= (1 << 2);
  GPIOC->ODR |= (1 << 13);
}

void
set_led (int on)
{
  if (on)
    GPIOC->ODR &= ~(1 << 13);
  else
    GPIOC->ODR |= (1 << 13);
}

static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

static uint8_t u, v;
static uint8_t m;		/* 0..100 */

static void
wait_for (uint32_t usec)
{
#if defined(BUSY_LOOP)
  uint32_t count = usec * 6;
  uint32_t i;

  for (i = 0; i < count; i++)
    asm volatile ("" : : "r" (i) : "memory");
#else
  chopstx_usec_wait (usec);
#endif
}

static void *
pwm (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd0, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      set_led (u&v);
      wait_for (m);
      set_led (0);
      wait_for (100-m);
    }

  return NULL;
}

static void *
blk (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd1, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      v = 0;
      wait_for (200*1000);
      v = 1;
      wait_for (200*1000);
    }

  return NULL;
}

#if defined(BUSY_LOOP)
#define PRIO_PWM (CHOPSTX_SCHED_RR|1)
#define PRIO_BLK (CHOPSTX_SCHED_RR|1)
#else
#define PRIO_PWM 3
#define PRIO_BLK 2
#endif

#define STACK_MAIN
#define STACK_PROCESS_1
#define STACK_PROCESS_2
#define STACK_PROCESS_3
#include "stack-def.h"


#define STACK_ADDR_PWM ((uint32_t)process1_base)
#define STACK_SIZE_PWM (sizeof process1_base)

#define STACK_ADDR_BLK ((uint32_t)process2_base)
#define STACK_SIZE_BLK (sizeof process2_base)

#define PRIO_USART     4
#define STACK_ADDR_USART ((uint32_t)process3_base)
#define STACK_SIZE_USART (sizeof process3_base)

#define LCD_WIDTH 160
#define LCD_HEIGHT 80

/*
 * GPIO PB1: RES#
 * low active
 */
static void
lcd_reset (int assert)
{
  if (assert)
    GPIOB->ODR &= ~(1 << 1);
  else
    GPIOB->ODR |= (1 << 1);
}

/*
 * GPIO PB2: CS#
 * low active
 */
static void
lcd_chip_select (int assert)
{
  if (assert)
    GPIOB->ODR &= ~(1 << 2);
  else
    GPIOB->ODR |= (1 << 2);
}

/*
 * GPIO PB0: D/C#
 *    Display: 1
 *    Command: 0
 */
#define LCD_DISPLAY 1
#define LCD_COMMAND 0
static void
lcd_control (int control)
{
  if (control == LCD_COMMAND)
    GPIOB->ODR &= ~(1 << 0);
  else
    GPIOB->ODR |= (1 << 0);
}


static void
lcd_send (uint8_t data)
{
  lcd_chip_select (1);
  spi_send (data);
  spi_recv ();
  lcd_chip_select (0);
}

static void
lcd_data8 (uint8_t data)
{
  lcd_send (data);
}


static void
lcd_data16 (uint16_t data)
{
  lcd_send (data>>8);
  lcd_send (data);
}

static void
lcd_command (uint8_t cmd)
{
  lcd_control (LCD_COMMAND);
  lcd_send (cmd);
  lcd_control (LCD_DISPLAY);
}

/* LCD: ST7735S */

static void
lcd_init (void)
{
  spi_init ();

  lcd_reset (1);
  chopstx_usec_wait (500*1000);
  lcd_reset (0);
  chopstx_usec_wait (50*1000);

  lcd_command (0x11);	/* SLPOUT: Sleep out */
  chopstx_usec_wait (100*1000);

  lcd_command (0x21);	/* INVON: display inversion on */
                        /* INVOFF: display inversion off: 0x20 */

  lcd_command (0x3a);	/* COLMOD: Pixel data format */
  lcd_data8 (0x05);     /* 16-bit/pixel */

  lcd_command (0x36);   /* MADCTL: Memory Data Access Control*/
  lcd_data8 (0x78);

  lcd_command (0xb1);	/* FRMCTR1: frame rate control 1 */
  lcd_data8 (0x05);
  lcd_data8 (0x3a);
  lcd_data8 (0x3a);

  lcd_command (0xb2);	/* FRMCTR2: frame rate control 2 */
  lcd_data8 (0x05);
  lcd_data8 (0x3a);
  lcd_data8 (0x3a);

  lcd_command (0xb3);	/* FRMCTR3: frame rate control 3 */
  lcd_data8 (0x05);
  lcd_data8 (0x3a);
  lcd_data8 (0x3a);
  lcd_data8 (0x05);
  lcd_data8 (0x3a);
  lcd_data8 (0x3a);

  lcd_command (0xb4);   /* INVCTR: inversion control */
  lcd_data8 (0x03);

  lcd_command (0xc0);   /* PWCTR1: power control 1 */
  lcd_data8 (0x62);
  lcd_data8 (0x02);
  lcd_data8 (0x04);

  lcd_command (0xc1);   /* PWCTR1: power control 2 */
  lcd_data8 (0xc0);

  lcd_command (0xc2);   /* PWCTR1: power control 3 */
  lcd_data8 (0x0d);
  lcd_data8 (0x00);

  lcd_command (0xc3);   /* PWCTR1: power control 4 */
  lcd_data8 (0x8d);
  lcd_data8 (0x6a);

  lcd_command (0xc4);   /* PWCTR1: power control 5 */
  lcd_data8 (0x8d);
  lcd_data8 (0xee);

  lcd_command (0xc5);   /* VCMCTR1: VCOM control 1 */
  lcd_data8 (0x0e);

  lcd_command (0xe0);   /* GMCTRP1: Gamma correction setting */
  lcd_data8 (0x10);
  lcd_data8 (0x0e);
  lcd_data8 (0x02);
  lcd_data8 (0x03);
  lcd_data8 (0x0e);
  lcd_data8 (0x07);
  lcd_data8 (0x02);
  lcd_data8 (0x07);
  lcd_data8 (0x0a);
  lcd_data8 (0x12);
  lcd_data8 (0x27);
  lcd_data8 (0x37);
  lcd_data8 (0x00);
  lcd_data8 (0x0d);
  lcd_data8 (0x0e);
  lcd_data8 (0x10);

  lcd_command (0xe1);   /* GMCTRN1: Gamma correction setting */
  lcd_data8 (0x10);
  lcd_data8 (0x0e);
  lcd_data8 (0x03);
  lcd_data8 (0x03);
  lcd_data8 (0x0f);
  lcd_data8 (0x06);
  lcd_data8 (0x02);
  lcd_data8 (0x08);
  lcd_data8 (0x0a);
  lcd_data8 (0x13);
  lcd_data8 (0x26);
  lcd_data8 (0x36);
  lcd_data8 (0x00);
  lcd_data8 (0x0d);
  lcd_data8 (0x0e);
  lcd_data8 (0x10);

  /* All set up! Now, turn on the display.  */

  lcd_command (0x29);	/* DISPON: Display On */
}

static void
lcd_address (uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  lcd_command (0x2a);           /* CASET: Column address */
  lcd_data16 (x1+1);
  lcd_data16 (x2+1);

  lcd_command (0x2b);           /* RASET: Row address */
  lcd_data16 (y1+26);
  lcd_data16 (y2+26);

  lcd_command (0x2c);           /* RAMWR: memory write */
}

static void
lcd_clear (uint16_t color)
{
  uint16_t i, j;

  lcd_address (0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);
  for (i = 0; i < LCD_WIDTH; i++)
    for (j = 0; j < LCD_HEIGHT; j++)
      lcd_data16 (color);
}

static void
lcd_load_image (const uint16_t *image)
{
  int i;

  lcd_address (0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);
  for(i = 0; i < LCD_WIDTH*LCD_HEIGHT; i++)
    lcd_data16 (image[i]);
}

/* Generated with list_565_colors.py */
static uint16_t colors[27] =
{
 0x0000, 0x000f, 0x001f, 0x03e0, 0x03ef, 0x03ff, 0x07e0, 0x07ef, 0x07ff,
 0x7800, 0x780f, 0x781f, 0x7be0, 0x7bef, 0x7bff, 0x7fe0, 0x7fef, 0x7fff,
 0xf800, 0xf80f, 0xf81f, 0xfbe0, 0xfbef, 0xfbff, 0xffe0, 0xffef, 0xffff,
};

static int
ss_notify (uint8_t dev_no, uint16_t state_bits)
{
  (void)dev_no;
  (void)state_bits;
  return 0;
}

#include "fsij-logo.c"

int
main (int argc, const char *argv[])
{
  chopstx_poll_cond_t poll_desc;
  struct chx_poll_head *ph[1];
  uint32_t timeout;
  int cl = 0;

  (void)argc;
  (void)argv;

  set_led (1);

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  m = 10;

  chopstx_create (PRIO_PWM, STACK_ADDR_PWM, STACK_SIZE_PWM, pwm, NULL);
  chopstx_create (PRIO_BLK, STACK_ADDR_BLK, STACK_SIZE_BLK, blk, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  lcd_init ();

  usart_init (PRIO_USART, STACK_ADDR_USART, STACK_SIZE_USART, ss_notify);
  usart_config (0, B115200 | CS8 | STOP1B);

  usart_read_prepare_poll (0, &poll_desc);
  ph[0] = (struct chx_poll_head *)&poll_desc;

  timeout = 200*1000*6;
  while (1)
    {
      chopstx_poll (&timeout, 1, ph);
      if (timeout == 0)
	{
	  usart_write (0, "Hello\r\n", 7);
	  u ^= 1;
	  timeout = 200*1000*6;

          lcd_clear (colors[cl]);

          chopstx_usec_wait (1000*1000);
          lcd_load_image (fsij_logo);
          chopstx_usec_wait (1000*1000);

          cl++;
          if (cl >= 27)
            cl = 0;
	}
      else
	{
	  char buf[256];
	  int r;
	  r = usart_read (0, buf, 256);
	  if (r)
	    usart_write (0, buf, r);
	}
    }

  return 0;
}
