/*
 * usart-stm32.c - USART driver for STM32F103 (USART2 and USART3)
 *
 * Copyright (C) 2017, 2019  g10 Code GmbH
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of Chopstx, a thread library for embedded.
 *
 * Chopstx is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chopstx is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * recipients of GNU GPL by a written offer.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include <mcu/stm32.h>
#include <contrib/usart.h>
#include <contrib/usart-impl.h>
#include <contrib/usart-impl-f103.h>

#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define USART2 ((struct USART *)USART2_BASE)
#define USART3 ((struct USART *)USART3_BASE)

static struct usart_stat usart2_stat;
static struct usart_stat usart3_stat;

static struct chx_intr usart2_intr;
static struct chx_intr usart3_intr;

#define BUF_A2H_SIZE 256
#define BUF_H2A_SIZE 512
static uint8_t buf_usart2_rb_a2h[BUF_A2H_SIZE];
static uint8_t buf_usart2_rb_h2a[BUF_H2A_SIZE];
static uint8_t buf_usart3_rb_a2h[BUF_A2H_SIZE];
static uint8_t buf_usart3_rb_h2a[BUF_H2A_SIZE];

static struct rb usart2_rb_a2h;
static struct rb usart2_rb_h2a;
static struct rb usart3_rb_a2h;
static struct rb usart3_rb_h2a;

static chopstx_poll_cond_t usart2_app_write_event;
static chopstx_poll_cond_t usart3_app_write_event;

/* Global variables so that it can be easier to debug.  */
static int usart2_tx_ready;
static int usart3_tx_ready;

#define INTR_REQ_USART2 38
#define INTR_REQ_USART3 39

#define USART_DEVNO_START 2
#define USART_DEVNO_END 3

static const struct usart usart_array[] =
  {
   { USART2, &usart2_intr, INTR_REQ_USART2,
     &usart2_stat, &usart2_rb_a2h, &usart2_rb_h2a, buf_usart2_rb_a2h,
     buf_usart2_rb_h2a, &usart2_app_write_event, &usart2_tx_ready },
   { USART3, &usart3_intr, INTR_REQ_USART3,
     &usart3_stat, &usart3_rb_a2h, &usart3_rb_h2a, buf_usart3_rb_a2h,
     buf_usart3_rb_h2a, &usart3_app_write_event, &usart3_tx_ready },
  };

/* We assume 36MHz f_PCLK */
static const struct brr_setting brr_table[] = {
  { B600,    (3750 << 4)},
  { B1200,   (1875 << 4)},
  { B2400,   ( 937 << 4)|8},
  { B9600,   ( 234 << 4)|6},
  { B19200,  ( 117 << 4)|3},
  { B57600,  (  39 << 4)|1},
  { B115200, (  19 << 4)|8},
  { B230400, (   9 << 4)|12},
  { B460800, (   4 << 4)|14},
  { B921600, (   2 << 4)|7},
  { BSCARD1,  ( 232 << 4)|8},	/*   9677 */
  { BSCARD2,  ( 116 << 4)|4},	/*  19354 */
  { BSCARD4,  (  58 << 4)|2},	/*  38709 */
  { BSCARD8,  (  29 << 4)|1},	/*  77419 */
  { BSCARD12, (  19 << 4)|6},	/* 116129 */
  { BSCARD16, (  14 << 4)|9},	/* 154506 */
  { BSCARD20, (  11 << 4)|10},	/* 193548 */
};

#include "usart-common.c"

static void
usart_rcc_setup (void)
{
  /* Enable USART2 and USART3 clocks, and strobe reset.  */
  RCC->APB1ENR |= ((1 << 18) | (1 << 17));
  RCC->APB1RSTR = ((1 << 18) | (1 << 17));
  RCC->APB1RSTR = 0;
}

#include "usart-common-f103.c"
