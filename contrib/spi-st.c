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

#ifndef SPI_USE_INTERRUPT
#define SPI_BUSY_WAIT 1
#endif

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

/* It is named SPI0 in GD32VF103, while it's "SPI1" in RM0008 of STM32F103 */
#define SPI0_BASE           (APB2PERIPH_BASE + 0x3000)
#define SPI0 ((struct SPI *)SPI0_BASE)

#define INTR_REQ_SPI0 54        /* For GD32VF103 */

static struct SPI *SPIx = SPI0;
#ifndef SPI_BUSY_WAIT
static struct chx_intr spi_intr;
#endif
static int tx_ready;
static int rx_done;

/*
 * For now, it only support configuration of a specific use case:
 *
 *     SPI0 of GD32VF103
 *     Master mode
 *     Hardware NSS pin not used
 *     Two unidirectional lines, full duplex
 *     CRC enabled
 *     8-bit data frame
 *     MSBit first
 *     Clock polarity: High==idle 
 *
 * We need to consider what kind of configuration can be specified
 * by the API.  Perhaps, it should return an error code when it is
 * not supported by the hardware.
 */

int
spi_init (void)
{
  uint32_t cr1_reg = 0;

  /* Enable SPI module */
  RCC->APB2ENR |= (1 << 12);
  RCC->APB2RSTR = (1 << 12);
  RCC->APB2RSTR = 0;

  /* Master mode */
  cr1_reg |= (1 << 2);          /* In RM0008, it's called MSTR */

  /* No use of hardware NSS pin */
  cr1_reg |= (1 << 9);          /* In RM0008, it's called SSM */
  cr1_reg |= (1 << 8);          /* In RM0008, it's called SSI */

  /* Two unidirectional lines, full duplex: Bit15=0, Bit14=0, Bit10=0 */
  /* Bit11: data-frame-format: 8-bit 0 (16-bit (1<<11)) */
  /* But7:  bit-order: MSB first 0, LSB first (1<<7)  */

  /* Enable CRC */
  cr1_reg |= (1 << 13);

  cr1_reg |= (0x02 << 3);            /* prescaler: fPCLK/8 */
  cr1_reg |= (1 << 1);               /* clock polarity 1==idle */
  cr1_reg |= (1 << 0);               /* second clock data capture edge */
  cr1_reg |= (1 << 6);               /* SPI enable */

  SPIx->CR1 = cr1_reg;

  /* Don't touch CRCPR: Just use reset default value of 7 */
  /* Don't touch I2SCFGR, I2SPR */


#ifndef SPI_BUSY_WAIT
  chopstx_claim_irq (&spi_intr, INTR_REQ_SPI0);
  /* Enable notification for Tx empty, Rx done, or Error */
  SPIx->CR2 = (1 << 7) | (1 << 6) | (1 << 5);
#endif

  tx_ready = 0;
  rx_done = 0;

  return 0;
}

static void
clear_crc_err (void)
{
  SPIx->SR &= ~(1 << 4);
}

static void
put_data (uint16_t data)
{
  tx_ready = 0;
  rx_done = 0;
  SPIx->DR = (uint32_t)data;
}

static uint16_t
get_data (void)
{
  return ((uint16_t)SPIx->DR);
}


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

#ifndef SPI_BUSY_WAIT
      /* Wait an event */
      chopstx_intr_wait (&spi_intr);
#endif

      status = SPIx->SR;

      if ((status & (1 << 0)))
	rx_done = 1;

      if ((status & (1 << 1)))
	tx_ready = 1;

      if ((status & (1 << 4)))
	{
	  /*FIXME: stat crc_err_count++ */
	  clear_crc_err ();
	}

#if 0
      /*FIXME: stat err_count++ */
      if ((status & 0x00fc))
	{
	}
#endif

#ifndef SPI_BUSY_WAIT
      chopstx_intr_done (&spi_intr);
#endif
    }
}

void
spi_send (uint16_t data)
{
  check_transmit (CHECK_TX);
  put_data (data);
}

uint16_t
spi_recv (void)
{
  check_transmit (CHECK_RX);
  return get_data ();
}
