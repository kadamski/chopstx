#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>

#include <mcu/gd32vf103.h>

static void
set_led (int on)
{
  if (on)
    GPIOC->ODR &= ~(1 << 13);
  else
    GPIOC->ODR |= (1 << 13);
}

void
set_led_0 (void)
{
  GPIOA->ODR |= ((1 << 2)|(1 << 1));
  GPIOC->ODR |= (1 << 13);
}

void
set_led_1 (void)
{
  GPIOC->ODR &= ~(1 << 13);
}

void
set_led_w (void)
{
  GPIOA->ODR &= ~((1 << 2)|(1 << 1));
  GPIOC->ODR &= ~(1 << 13);
}

void
set_led_r (void)
{
  GPIOA->ODR |= ((1 << 2)|(1 << 1));
  GPIOC->ODR &= ~(1 << 13);
}

void
set_led_g (void)
{
  GPIOA->ODR &= ~(1 << 1);
  GPIOA->ODR |= (1 << 2);
  GPIOC->ODR |= (1 << 13);
}

void
set_led_b (void)
{
  GPIOA->ODR |= (1 << 1);
  GPIOA->ODR &= ~(1 << 2);
  GPIOC->ODR |= (1 << 13);
}


static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

static uint8_t u, v;
static uint8_t m;		/* 0..100 */

static void
busy_wait_for (uint32_t usec)
{
  uint32_t count = usec * 20;
  uint32_t i;

  for (i = 0; i < count; i++)
    asm volatile ("" : : "r" (i) : "memory");
}

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
#if 0
      set_led (u&v);
      wait_for (m);
      set_led (0);
      wait_for (100-m);
#else
      wait_for (300*1000);
      set_led (1);
      wait_for (300*1000);
      set_led (0);
#endif
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
#include "stack-def.h"


#define STACK_ADDR_PWM ((uint32_t)process1_base)
#define STACK_SIZE_PWM (sizeof process1_base)

#define STACK_ADDR_BLK ((uint32_t)process2_base)
#define STACK_SIZE_BLK (sizeof process2_base)

// static uint8_t *x = (uint8_t *)0x20001000;

int
main (int argc, const char *argv[])
{
  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  m = 10;

  chopstx_create (PRIO_PWM, STACK_ADDR_PWM, STACK_SIZE_PWM, pwm, NULL);
  // chopstx_create (PRIO_BLK, STACK_ADDR_BLK, STACK_SIZE_BLK, blk, NULL);

  chopstx_usec_wait (200*1000);

  set_led_r ();

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  u = 1;
  v = 1;

  for (;;)
    {
      set_led_w ();
      busy_wait_for (1000*1000);
      set_led_0 ();
      busy_wait_for (1000*1000);
    }

  wait_for (100*1000);

#if 0
  asm volatile (
	"ebreak"
	: : : "memory" );

  asm volatile (
	"csrsi	mstatus,8"
	: : : "memory" );

  asm volatile (
	"lw	a0,0(%0)"
	: : "r" (0x08000001) : "a0", "memory" );

  asm volatile (
	"ebreak"
	: : : "memory" );
  asm volatile (
	"csrsi	mstatus,8"
	: : : "memory" );

  asm volatile (
	"lw	a0,0(%0)"
	: : "r" (0xB0000000) : "a0", "memory" );

  asm volatile (
	"ecall"
	: : : "memory" );
#endif

  while (1)
    {
#if 0
      set_led_g ();
      chopstx_usec_wait (1000*1000);
      set_led_b ();

      chopstx_usec_wait (1000*1000);
#endif
      u ^= 1;
      wait_for (200*1000*6);
#if 0
      set_led (u);
      chopstx_usec_wait (1000*1000);
#endif
    }

  return 0;
}
