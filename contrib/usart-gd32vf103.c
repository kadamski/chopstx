/*
 * usart-gd32vf103.c - USART driver for GD32VF103 (USART0)
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

#define USART0_BASE           (APB2PERIPH_BASE + 0x3800)
#define USART0 ((struct USART *)USART0_BASE)

static struct usart_stat usart0_stat;

static struct chx_intr usart0_intr;

#define BUF_A2H_SIZE 256
#define BUF_H2A_SIZE 512
static uint8_t buf_usart0_rb_a2h[BUF_A2H_SIZE];
static uint8_t buf_usart0_rb_h2a[BUF_H2A_SIZE];

static struct rb usart0_rb_a2h;
static struct rb usart0_rb_h2a;

static chopstx_poll_cond_t usart0_app_write_event;

/* Global variables so that it can be easier to debug.  */
static int usart0_tx_ready;

#define INTR_REQ_USART0 56

#define USART_DEVNO_START 0
#define USART_DEVNO_END 0

static const struct usart usart_array[] =
  {
   { USART0, &usart0_intr, INTR_REQ_USART0,
     &usart0_stat, &usart0_rb_a2h, &usart0_rb_h2a, buf_usart0_rb_a2h,
     buf_usart0_rb_h2a, &usart0_app_write_event, &usart0_tx_ready },
  };

/* It's 96MHz f_PCLK */
static const struct brr_setting brr_table[] = {
  /* Can't be represented in 16-bit { B600,    (10000 << 4)}, */
  /* Can't be represented in 16-bit { B1200,   (5000 << 4)},  */
  { B2400,   ( 2500 << 4)|0},
  { B9600,   (  625 << 4)|0},
  { B19200,  (  312 << 4)|8},
  { B57600,  (  104 << 4)|3},
  { B115200, (   52 << 4)|1},
  { B230400, (   26 << 4)|0},
  { B460800, (   13 << 4)|0},
  { B921600, (    6 << 4)|8},
  { BSCARD1,  ( 620 << 4)|0},	/*   9677 */
  { BSCARD2,  ( 310 << 4)|0},	/*  19354 */
  { BSCARD4,  ( 155 << 4)|0},	/*  38709 */
  { BSCARD8,  (  77 << 4)|8},	/*  77419 */
  { BSCARD12, (  51 << 4)|11},	/* 116129 */
  { BSCARD16, (  38 << 4)|13},	/* 154506 */
  { BSCARD20, (  31 << 4)|0},	/* 193548 */
};

#include "usart-common.c"

static void
usart_rcc_setup (void)
{
  /* Enable USART0 clock, and strobe reset.  */
  RCC->APB2ENR |= (1 << 14);
  RCC->APB2RSTR = (1 << 14);
  RCC->APB2RSTR = 0;
}

#include "usart-common-f103.c"
