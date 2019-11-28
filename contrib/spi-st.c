/*
 * spi-st.c - SPI driver for STM32F103/GD32F103/GD32VF103
 *
 * Copyright (C) 2019  Flying Stone Technology
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

/* Hardware registers */
struct SPI {
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
};

static struct chx_intr spi_intr;

void
spi_init (void)
{
  uint32_t reg;

  /* Enable SPI module */
  RCC->APB2RSTR = (1 << 12);
  RCC->APB2RSTR = 0;
  RCC->APB2RENR |= (1 << 12);

  if (master)
    reg |= SPICR_MSTR;

  if (nss_no_hw_nsspin)
    {
      reg |= (1 << 9);          /* In RM0008, it's called SSM */
      if (master)
        reg |= (1 << 8);        /* In RM0008, it's called SSI */
    }

  /* transfer mode is 2 unidirectional lines, full duplex */
  /* (1<<15), (1<<14), (1<<10) */
  reg |= (1 << 13); /* enable CRC */
  /* data-frame-format: 8-bit 0 (16-bit (1<<11)) */
  /* bit-order: MSB first 0, LSB first (1<<7)  */
  reg |= (0x02 << 3);            /* prescaler: fPCLK/8 */
  reg |= (1 << 1);               /* clock polarity 1==idle */
  reg |= (1 << 0);               /* second clock data capture edge */
  reg |= (1 << 6);               /* SPI enable */

  SPIx->CR1 = reg;
  // SPIx->CRCPR = 7; /*Just use reset default value of 7*/
  // Don't touch I2SCFGR, I2SPR

  chopstx_claim_irq (&spi_intr, INTR_REQ_SPI0);

  /* Notification for Tx empty, Rx ready, or Error */
  SPIx->CR2 = (1 << 7) | (1 << 6) | (1 << 5);
}

static void
clear_crc_err (void)
{
  SPIx->SR &= ~(1 << 4);
}

static void
put_data (uint16_t data)
{
  SPIx->DR = (uint32_t)data;
}

static uint16_t
get_data (void)
{
  return ((uint16_t)SPIx->DR);
}


static int tx_ready;
static int rx_done;
#define CHECK_TX 0
#define CHECK_RX 1

static void
check_transmit (int for_what)
{
  for (;;)
    {
      uint16_t status;

      if (for_what == CHECK_TX && tx_ready)
        return;

      if (for_what == CHECK_RX && rx_done)
        return;

      if (for_what == CHECK_TX)
        SPIx->CR2 |= (1 << 7);

      if (for_what == CHECK_RX)
        SPIx->CR2 |= (1 << 6);

      /* Wait an event */
      chopstx_intr_wait (&spi_intr);
      status = SPIx->SR;

      if ((status & TXE))
        {
          tx_ready = 1;
          SPIx->CR2 &= ~(1 << 7);
        }

      if ((status & RXNE))
        {
          rx_done = 1;
          SPIx->CR2 &= ~(1 << 6);
        }

      if ((status & ERRBITS))
        {
          /*FIXME: stat count++ */
          clear_crc_err ();
        }

      chopstx_intr_done (&spi_intr)
    }
}

void
spi_data_send (uint16_t data)
{
  check_transmit (CHECK_TX);
  put_data (data);
}

uint16_t
spi_data_receive (void)
{
  check_transmit (CHECK_RX);
  return get_data ();
}
