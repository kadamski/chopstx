/*
 * usb-gd32vf103.c - USB driver for GD32VF103
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
#include <string.h>

#include <chopstx.h>
#include <mcu/gd32vf103.h>

#include "usb_lld.h"

/*
 * This driver is for USB device functionality (not host).
 *
 * This driver doesn't support isochronous transfer (yet).
 * Only supports one packet at a time.
 */

#ifdef DEBUG_BY_SERIAL_OUTPUT
extern void debug (const char *fmt, ...);
#else
#define debug(a,...) /**/
#endif

#define USB_DIR_OUT  0
#define USB_DIR_IN   1
#define USB_DIR_BOTH 2

#define USBFS_BASE 0x50000000

/* USB Global registers */
struct USB_G {
  volatile uint32_t GOTGCS;
  volatile uint32_t GOTGINTF;
  volatile uint32_t GAHBCS;
  volatile uint32_t GUSBCS;

  volatile uint32_t GRSTCTL;
  volatile uint32_t GINTF;
  volatile uint32_t GINTEN;
  volatile uint32_t GRSTATR;

  volatile uint32_t GRSTATP;
  volatile uint32_t GRFLEN;
  volatile uint32_t DIEP0TFLEN_HNPTFLEN;
  volatile uint32_t HNPTFQSTAT;

  uint32_t rsvd0;
  uint32_t rsvd1;
  volatile uint32_t GCCFG;
  volatile uint32_t CID;
  uint32_t rsvd2[48];
  uint32_t rsvd3;
  volatile uint32_t DIEP_TFLEN[3];
};
static struct USB_G *const USB_G = (struct USB_G *)USBFS_BASE;

/* USB Device registers */
struct USB_D {
  volatile uint32_t DCFG;
  volatile uint32_t DCTL;
  volatile uint32_t DSTAT;
  uint32_t rsvd0;

  volatile uint32_t DIEPINTEN;
  volatile uint32_t DOEPINTEN;
  volatile uint32_t DAEPINT;
  volatile uint32_t DAEPINTEN;

  uint32_t rsvd1;
  uint32_t rsvd2;
  volatile uint32_t DVBUSDT;
  volatile uint32_t DVPUSPT;

  uint32_t rsvd3;
  volatile uint32_t DIEPFEINTEN;
};
static struct USB_D *const USB_D = (struct USB_D *)(USBFS_BASE+0x800);

/* USB device endpoint IN registers */
struct USB_I {
  volatile uint32_t DIEP_CTL;
  uint32_t rsvd0;
  volatile uint32_t DIEP_INTF;
  uint32_t rsvd1;

  volatile uint32_t DIEP_LEN;
  uint32_t rsvd2;
  volatile uint32_t DIEP_TFSTAT;
  uint32_t rsvd3;
};
static struct USB_I *const USB_I = (struct USB_I *)(USBFS_BASE+0x900);

/* USB device endpoint OUT registers */
struct USB_O {
  volatile uint32_t DOEP_CTL;
  uint32_t rsvd0;
  volatile uint32_t DOEP_INTF;
  uint32_t rsvd1;

  volatile uint32_t DOEP_LEN;
  uint32_t rsvd2;
  uint32_t rsvd3;
  uint32_t rsvd4;
};
static struct USB_O *const USB_O = (struct USB_O *)(USBFS_BASE+0xB00);

#if 0
/* USB Power control register */
struct USB_P {
  volatile uint32_t PWRCLKCTL;
};
static struct USB_P *const USB_P = (struct USB_P *)(USBFS_BASE+0xE00);
#endif

struct USB_FIFO {
  struct {
    volatile uint32_t data;
    uint32_t rsvd[1023];
  } FIFO[4];
};
static struct USB_FIFO *const USB_FIFO = (struct USB_FIFO *)(USBFS_BASE+0x1000);

#define USB_ENDPOINT_MAX 4

static void
gd32vf_usb_clock_init (void)
{
  /* Reset the clock.  */
  if ((RCU->AHBEN & RCU_AHB_USBFS)
      && (RCU->AHBRST & RCU_AHB_USBFS) == 0)
    /* Make sure the device is disconnected, even after core reset.  */
    {
      RCU->AHBEN &= ~RCU_AHB_USBFS;
      RCU->AHBRST = RCU_AHB_USBFS;

      /* Disconnect requires SE0 (>= 2.5uS).  */
      chopstx_usec_wait (10);
    }

  if ((RCU->AHBEN & RCU_AHB_USBFS) == 0)
    {
      RCU->AHBEN |= RCU_AHB_USBFS;
      RCU->AHBRST = RCU_AHB_USBFS;
      RCU->AHBRST = 0;
    }
}

static void
gd32vf_usb_init (void)
{
  uint32_t usb_ramaddr = 0;
  int i;

  gd32vf_usb_clock_init ();

  /* Undocumented setting of PHY (GD specific).  */
  USB_G->GUSBCS |= (1 << 6);    /* Use embedded PHY */

  /* Enable the transceiver.
     Disable monitoring VBUS pin (GD specific).  */
  USB_G->GCCFG = (1 << 21) | (1 << 19) | (1 << 18) | (1 << 16);

  chopstx_usec_wait (20*1000);

  /* Disconnect.  */
  USB_D->DCTL |= (1 << 1);

  /* Select device mode (not host mode or OTG).  */
  USB_G->GUSBCS &= ~((1 << 30) | (1 << 29));
  USB_G->GUSBCS |= (1 << 30);

  chopstx_usec_wait (10*1000);

  /* Configure the receive FIFO RAM address */
  USB_G->GRFLEN = 64;
  usb_ramaddr += 64;

  USB_G->DIEP0TFLEN_HNPTFLEN = (64 << 16) | usb_ramaddr;
  usb_ramaddr += 64;

  for (i = 1; i < USB_ENDPOINT_MAX; i++)
    {
      USB_G->DIEP_TFLEN[i - 1] = (64 << 16) | usb_ramaddr;
      usb_ramaddr += 64;
    }

  /* Configure device mode.  */
  USB_D->DCFG &= ~((1 << 12) | (1 << 11)); /* 80% of frame interval */
  USB_D->DCFG |= ((1 << 1)| (1 << 0));     /* Full speed */

  /* Connect */
  USB_D->DCTL &= ~(1 << 1);

  /* Interrupt setup.  */
  USB_D->DIEPINTEN = 0;
  USB_D->DOEPINTEN = 0;
  USB_D->DAEPINTEN = 0;

  /* Transfer finished.  */
  USB_D->DIEPINTEN |= (1 << 0);
  /* Setup finished, Transfer finished.  */
  USB_D->DOEPINTEN |= (1 << 3) | (1 << 0);
  /* Configure for endpoint 0, IN and OUT.  */
  USB_D->DAEPINTEN = (1 << 16) | (1 << 0);

  /* Wakeup and Suspend.  */
  USB_G->GINTEN = (1 << 31) | (1 << 11);
  /* IN and OUT.  */
  USB_G->GINTEN |= (1 << 19) | (1 << 18);
  /* USB Reset.  */
  USB_G->GINTEN |= (1 << 12);
  /* Speed enumeration.  */
  USB_G->GINTEN |= (1 << 13);

  /* RxFIFO non-empty */
  USB_G->GINTEN |= (1 << 4);

  /* Note: We don't enable event for start of frame (1 << 3) */

  /* TxFIFO completely empty, Enable interrupt from the USB module.  */
  USB_G->GAHBCS |= (1 << 7) | (1 << 0);
}

static void
gd32vf_set_daddr (uint8_t daddr)
{
  USB_D->DCFG &= ~(0x7f << 4);
  USB_D->DCFG |= (daddr << 4);
}

static void
gd32vf_prepare_ep0_setup (struct usb_dev *dev)
{
  dev->ctrl_data.addr = (uint8_t *)&dev->dev_req;
  dev->ctrl_data.len = 8;
  dev->ctrl_data.require_zlp = 0;

  /* Configure control endpoint to accept 8-byte SETUP stage.  */
  /*
   * Note: It makes no sense to configure back-to-back setup.
   * In a correct control transfer, a setup stage is always
   * followed by data stage or status stage.
   */
  /* setup_cnt=1, packet_cnt=1, buflen=8-byte.  */
  USB_O[0].DOEP_LEN = (1 << 29) | (1 << 19) | 8;
  USB_O[0].DOEP_CTL = (1 << 31) | (1 << 26); /* Enable and Clear NAK */
}

static void
gd32vf_tx_fifo_flush (int n)
{
  USB_G->GRSTCTL = (n << 6) | (1 << 5);
  while ((USB_G->GRSTCTL & (1 << 5)))
    chopstx_usec_wait (3);
  chopstx_usec_wait (3);
}

static void
gd32vf_tx_fifo_write (int n, const void *buf, uint8_t len)
{
  const uint8_t *p = buf;
  uint32_t data;
  int i;

#if 0
  if ((USB_I[n].DIEP_TFSTAT & 0xffff) < len)
    {                           /* Not enough space, yet */
      return;
    }
#endif

  for (i = 0; i < len / 4; i++)
    {
      data = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
      p += 4;
      USB_FIFO->FIFO[n].data = data;
    }

  if ((len % 4))
    {
      data = 0;
      for (i = 0; i < (len % 4); i++)
	data |= (*p++) << (i * 8);
      USB_FIFO->FIFO[n].data = data;
    }
}

static void
gd32vf_prepare_ep0_in (uint8_t len)
{
  USB_I[0].DIEP_LEN = (1 << 19) | len; /* 1-packet, len-byte */
  if (len)
    USB_D->DIEPFEINTEN |= (1 << 0);
  USB_I[0].DIEP_CTL = (1 << 31) | (1 << 26); /* Enable and Clear NAK */
}

/* Note: No support of multiple DATA transfer.  It's by DATA1 only.  */
static void
gd32vf_prepare_ep0_out (uint8_t len)
{
  /* Set pktcnt=1 and length, while keeping SETUP_COUNT field.  */
  USB_O[0].DOEP_LEN &= ~127;                 /* Clear length field.  */
  USB_O[0].DOEP_LEN |= (1 << 19) | len;
  USB_O[0].DOEP_CTL = (1 << 31) | (1 << 26); /* Enable and Clear NAK */
}

static int
gd32vf_ep_is_active (uint8_t n, int dir)
{
  uint32_t bit_to_examine;

  if (dir == USB_DIR_IN)
    bit_to_examine = (USB_I[n].DIEP_CTL & (1 << 15));
  else
    bit_to_examine = (USB_O[n].DOEP_CTL & (1 << 15));

  return bit_to_examine != 0;
}

static int
gd32vf_ep_is_stalled (uint8_t n, int dir)
{
  uint32_t bit_to_examine;

  if (dir == USB_DIR_IN)
    bit_to_examine = (USB_I[n].DIEP_CTL & (1 << 21));
  else
    bit_to_examine = (USB_O[n].DOEP_CTL & (1 << 21));

  return bit_to_examine != 0;
}

static void
gd32vf_ep_stall (uint8_t n, int dir)
{
  if (dir == USB_DIR_IN || dir == USB_DIR_BOTH)
    {
      /* When it's enabled, disable it.  */
      if (USB_I[n].DIEP_CTL & (1 << 31))
	USB_I[n].DIEP_CTL |= (1 << 30);

      USB_I[n].DIEP_CTL |= (1 << 21);
    }

  if (dir == USB_DIR_OUT || dir == USB_DIR_BOTH)
    USB_O[n].DOEP_CTL |= (1 << 21);
}

static void
gd32vf_ep_clear_stall (uint8_t n, int dir)
{
  if (dir == USB_DIR_IN || dir == USB_DIR_BOTH)
    {
      USB_I[n].DIEP_CTL &= ~(1 << 21);
      USB_I[n].DIEP_CTL |= (1 << 28);
    }

  if (dir == USB_DIR_OUT || dir == USB_DIR_BOTH)
    {
      USB_O[n].DOEP_CTL &= ~(1 << 21);
      USB_O[n].DOEP_CTL |= (1 << 28);
    }
}

#include "usb_lld_driver.h"

static int handle_in0 (struct usb_dev *dev);
static void handle_out0 (struct usb_dev *dev);
static int handle_setup0 (struct usb_dev *dev);


void
usb_lld_stall_tx (struct usb_dev *dev, int n)
{
  (void)dev;
  gd32vf_ep_stall (n, USB_DIR_IN);
}

void
usb_lld_stall_rx (struct usb_dev *dev, int n)
{
  (void)dev;
  gd32vf_ep_stall (n, USB_DIR_OUT);
}


void
usb_lld_ctrl_error (struct usb_dev *dev)
{
  dev->state = STALLED;
  gd32vf_ep_stall (ENDP0, USB_DIR_BOTH);
  gd32vf_prepare_ep0_setup (dev);
}

int
usb_lld_ctrl_ack (struct usb_dev *dev)
{
  debug ("C W ack");
  /* Zero length packet for ACK.  */
  dev->state = WAIT_STATUS_IN;
  gd32vf_prepare_ep0_in (0);
  return USB_EVENT_OK;
}


void
usb_lld_init (struct usb_dev *dev, uint8_t feature)
{
  int i;

  usb_lld_set_configuration (dev, 0);
  dev->feature = feature;
  dev->state = WAIT_SETUP;
  gd32vf_usb_init ();
  dev->ctrl_data.addr = (uint8_t *)&dev->dev_req;
  dev->ctrl_data.len = 8;
  dev->ctrl_data.require_zlp = 0;
  for (i = 0; i < 3; i++)
    {
      dev->epctl_rx[i].addr = NULL;
      dev->epctl_tx[i].addr = NULL;
      dev->epctl_rx[i].len = 0;
      dev->epctl_tx[i].len = 0;
    }
}


static void
read_from_rx_fifo (struct usb_dev *dev)
{
  /* RxFIFO ready.  */
  uint32_t stat = USB_G->GRSTATP;
  uint8_t ep_num = (stat & 0x0f);
  uint16_t len = (stat >> 4) & 0x7ff;
  uint8_t pkt = (stat >> 17) & 0x0f;

  if (ep_num == 0)
    {
      uint8_t *p = dev->ctrl_data.addr;
      uint32_t data;
      int i;

      if (pkt == 0x06)
	{
	  debug ("\rrx S %d", len);
	  if (dev->ctrl_data.len == 0)
	    {
	      /* Receiving re-send of SETUP packet.  */
	      p = dev->ctrl_data.addr - 8;
	    }
	}
      else if (pkt == 0x02)
	{
	  if (len > dev->ctrl_data.len)
	    {
	      len = dev->ctrl_data.len;
	      debug ("rx O %d*", len);
	    }
	  else
	    {
	      debug ("rx O %d", len);
	    }
	}
      else
	return;

      for (i = 0; i < len / 4; i++)
	{
#ifdef DEBUG_BY_SERIAL_OUTPUT
	  uint8_t *p0 = p;
#endif
	  data = USB_FIFO->FIFO[0].data;
	  *p++ = (data & 0xff);
	  data >>= 8;
	  *p++ = (data & 0xff);
	  data >>= 8;
	  *p++ = (data & 0xff);
	  data >>= 8;
	  *p++ = (data & 0xff);
	  debug ("%02x %02x %02x %02x", p0[0], p0[1], p0[2], p0[3]);
	}

      if ((len % 4))
	{
	  data = USB_FIFO->FIFO[0].data;
	  for (i = 0; i < (len % 4); i++)
	    {
	      *p++ = (data & 0xff);
	      debug ("%02x", (data & 0xff));
	      data >>= 8;
	    }
	}

      dev->ctrl_data.len -= len;
      dev->ctrl_data.addr += len;
    }
  else 
    {
      uint8_t *p = dev->epctl_rx[ep_num-1].addr;
      uint32_t data;
      int i;

      if (pkt == 0x02)
	{
	  if (len > dev->epctl_rx[ep_num-1].len)
	    {
	      len = dev->epctl_rx[ep_num-1].len;
	      debug ("\rrx EP%d O %d*", ep_num, len);
	    }
	  else
	    {
	      debug ("\rrx EP%d O %d", ep_num, len);
	    }

	  dev->epctl_rx[ep_num-1].len = len;
	}
      else
	return;

      for (i = 0; i < len / 4; i++)
	{
#ifdef DEBUG_BY_SERIAL_OUTPUT
	  uint8_t *p0 = p;
#endif
	  data = USB_FIFO->FIFO[ep_num].data;
	  *p++ = (data & 0xff);
	  data >>= 8;
	  *p++ = (data & 0xff);
	  data >>= 8;
	  *p++ = (data & 0xff);
	  data >>= 8;
	  *p++ = (data & 0xff);
	  debug ("%02x %02x %02x %02x", p0[0], p0[1], p0[2], p0[3]);
	}

      if ((len % 4))
	{
	  data = USB_FIFO->FIFO[ep_num].data;
	  for (i = 0; i < (len % 4); i++)
	    {
	      *p++ = (data & 0xff);
	      debug ("%02x", (data & 0xff));
	      data >>= 8;
	    }
	}
    }
}

#define USB_MAKE_EV(event) (event<<24)
#define USB_MAKE_TXRX(ep_num,txrx,len) ((txrx? (1<<23):0)|(ep_num<<16)|len)

int
usb_lld_event_handler (struct usb_dev *dev)
{
  uint32_t gintr = (USB_G->GINTF & USB_G->GINTEN);
  uint32_t bits;
  uint32_t ep_num = 0;
  static uint32_t sof = 0;

  if (!gintr)
    return USB_EVENT_OK;
  else if ((gintr & (1 << 3)))
    {
      /* sof */
      if ((sof++ & 0xff) == 0xff)
	{
	  debug ("sof");
	}
      USB_G->GINTF = (1 << 3);
      return USB_EVENT_OK;
    }
  else if ((gintr & (1 << 31)))
    {
      /* wakeup */
      debug ("\rwakeup");
      USB_G->GINTF = (1 << 31);
      return USB_MAKE_EV (USB_EVENT_DEVICE_WAKEUP);
    }
  else if ((gintr & (1 << 11)))
    {
      debug ("\rsuspend");
      /* suspend */
      USB_G->GINTF = (1 << 11);
      if ((USB_D->DSTAT & 1))
	return USB_MAKE_EV (USB_EVENT_DEVICE_SUSPEND);
      else
	return USB_EVENT_OK;
    }
  else if ((gintr & (1 << 19)))
    {
      /* SETUP or OUT */
      for (bits = ((USB_D->DAEPINT >> 16) & 0x0f); bits; bits >>= 1, ep_num++)
	if ((bits & 1))
	  {
#if 0
	    if ((USB_O[ep_num].DOEP_INTF & (1 << 6)))
	      {
		/* Multiple SETUP packets received.  */
		USB_O[ep_num].DOEP_INTF = (1 << 6);
	      }
#endif

	    if (ep_num == 0 && (USB_O[ep_num].DOEP_INTF & (1 << 3)))
	      {
		int r = USB_MAKE_EV (handle_setup0 (dev));
		USB_O[ep_num].DOEP_INTF = (1 << 3);
		return r;
	      }
	    else if ((USB_O[ep_num].DOEP_INTF & (1 << 0)))
	      {
		USB_O[ep_num].DOEP_INTF = (1 << 0);
		debug ("O %d", ep_num);
		if (ep_num == 0)
		  {
		    /* Ignore transfer finished for OUT which comes at SETUP */
		    if (dev->state != WAIT_SETUP)
		      handle_out0 (dev);
		    return USB_EVENT_OK;
		  }
		else
		  return USB_MAKE_TXRX (ep_num, 0, dev->epctl_rx[ep_num-1].len);
	      }
	  }
    }
  else if ((gintr & (1 << 18)))
    {
      /* IN */
      for (bits = (USB_D->DAEPINT & 0x0f); bits; bits >>= 1, ep_num++)
	if ((bits & 1))
	  {
#if 0
	    if ((USB_I[ep_num].DIEP_INTF & (1 << 3)))
	      {
		USB_I[ep_num].DIEP_INTF = (1 << 3);
	      }
#endif

	    if ((USB_I[ep_num].DIEP_INTF & (1 << 0)))
	      {
		USB_I[ep_num].DIEP_INTF = (1 << 0);
		debug ("I %d", ep_num);
		if (ep_num == 0)
		  return USB_MAKE_EV (handle_in0 (dev));
		else
		  return USB_MAKE_TXRX (ep_num, 1, dev->epctl_tx[ep_num-1].len);
	      }

	    if ((USB_I[ep_num].DIEP_INTF & (1 << 7)))
	      {
		uint8_t len;

		len = USB_I[ep_num].DIEP_LEN & 0x7f;
		debug ("fifo tx %d (%d)", ep_num, len);
		if (ep_num)
		  {
		    gd32vf_tx_fifo_write (ep_num, dev->epctl_tx[ep_num-1].addr,
					  len);
		    USB_D->DIEPFEINTEN &= ~(1 << ep_num);
		  }
		else
		  {
		    gd32vf_tx_fifo_write (0, dev->ctrl_data.addr, len);

		    dev->ctrl_data.len -= len;
		    dev->ctrl_data.addr += len;
		    if (dev->ctrl_data.len == 0)
		      {
			USB_D->DIEPFEINTEN &= ~(1 << ep_num);
			debug ("< %d: done", len);
		      }
		    else
		      {
			debug ("< %d", len);
		      }
		  }

		USB_I[ep_num].DIEP_INTF = (1 << 7);
	      }
	  }
    }
  else if ((gintr & (1 << 4)))
    {
      read_from_rx_fifo (dev);
    }
  else if ((gintr & (1 << 13)))
    {
      /* Speed enumeration: HS/FS negotiation.  */
      /* Clear global IN NAK.  */
      USB_D->DCTL &= ~(1 << 8);
      USB_D->DCTL |= (1 << 8);
      /*
       * Set the UTT (USB turnaround time).
       * Value is not documented for GD.
       * Need change for STM32 (when ported).
       */
      USB_G->GUSBCS &= ~(0x0f << 10);
      USB_G->GUSBCS |= (0x05 << 10);
      USB_G->GINTF = (1 << 13);
      debug ("spd=FS");
    }
  else if ((gintr & (1 << 12)))
    {
      /* USB RESET */
      USB_G->GINTF = (1 << 12);
      return USB_MAKE_EV (USB_EVENT_DEVICE_RESET);
    }

  return USB_EVENT_OK;
}


static void
handle_datastage_out (struct usb_dev *dev)
{
  if (dev->ctrl_data.len == 0)
    {
      debug ("C W ack!");
      dev->state = WAIT_STATUS_IN;
      gd32vf_prepare_ep0_in (0);
    }
  else
    {
      uint8_t len = dev->ctrl_data.len;
      if (len > USB_MAX_PACKET_SIZE)
	len = USB_MAX_PACKET_SIZE;

      dev->state = OUT_DATA;
      gd32vf_prepare_ep0_out (len);
    }
}


static void
handle_datastage_in (struct usb_dev *dev)
{
  uint32_t len = USB_MAX_PACKET_SIZE;
  struct ctrl_data *data_p = &dev->ctrl_data;

  if ((data_p->len == 0) && (dev->state == LAST_IN_DATA))
    {
      if (data_p->require_zlp)
	{
	  data_p->require_zlp = 0;

	  /* No more data to send.  Send empty packet */
	  gd32vf_prepare_ep0_in (0);
	}
      else
	{
	  /* No more data to send, proceed to receive OUT acknowledge.  */
	  dev->state = WAIT_STATUS_OUT;
	  gd32vf_prepare_ep0_out (0);
	}

      return;
    }

  dev->state = (data_p->len <= len) ? LAST_IN_DATA : IN_DATA;

  if (len > data_p->len)
    len = data_p->len;

  gd32vf_prepare_ep0_in (len);
}

typedef int (*HANDLER) (struct usb_dev *dev);

static int
std_none (struct usb_dev *dev)
{
  (void)dev;
  return -1;
}

static uint16_t status_info;

static int
direction_of_endpoint (struct device_req *arg)
{
  return (arg->index & 0x80)? USB_DIR_IN : USB_DIR_OUT;
}

static int
std_get_status (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (arg->value != 0 || arg->len != 2 || (arg->index >> 8) != 0
      || USB_SETUP_SET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->index == 0)
	{
	  /* Get Device Status */
	  uint8_t feature = dev->feature;

	  /* Remote Wakeup enabled */
	  if ((feature & (1 << 5)))
	    status_info |= 2;
	  else
	    status_info &= ~2;

	  /* Bus-powered */
	  if ((feature & (1 << 6)))
	    status_info |= 1;
	  else /* Self-powered */
	    status_info &= ~1;

	  return usb_lld_ctrl_send (dev, &status_info, 2);
	}
    }
  else if (rcp == INTERFACE_RECIPIENT)
    {
      if (dev->configuration == 0)
	return -1;

      return USB_EVENT_GET_STATUS_INTERFACE;
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t n = (arg->index & 0x0f);

      if ((arg->index & 0x70) || n == ENDP0)
	return -1;

      if (!gd32vf_ep_is_active (n, direction_of_endpoint (arg)))
	return -1;

      status_info = gd32vf_ep_is_stalled (n, direction_of_endpoint (arg));

      return usb_lld_ctrl_send (dev, &status_info, 2);
    }

  return -1;
}

static int
std_clear_feature (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->len != 0 || arg->index != 0)
	return -1;

      if (arg->value == FEATURE_DEVICE_REMOTE_WAKEUP)
	{
	  dev->feature &= ~(1 << 5);
	  return USB_EVENT_CLEAR_FEATURE_DEVICE;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t n = (arg->index & 0x0f);

      if (dev->configuration == 0)
	return -1;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != FEATURE_ENDPOINT_HALT || n == ENDP0)
	return -1;

      if (!gd32vf_ep_is_active (n, direction_of_endpoint (arg)))
	return -1;

      gd32vf_ep_clear_stall (n, direction_of_endpoint (arg));

      return USB_EVENT_CLEAR_FEATURE_ENDPOINT;
    }

  return -1;
}

static int
std_set_feature (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->len != 0 || arg->index != 0)
	return -1;

      if (arg->value == FEATURE_DEVICE_REMOTE_WAKEUP)
	{
	  dev->feature |= 1 << 5;
	  return USB_EVENT_SET_FEATURE_DEVICE;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t n = (arg->index & 0x0f);

      if (dev->configuration == 0)
	return -1;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != FEATURE_ENDPOINT_HALT || n == ENDP0)
	return -1;

      if (!gd32vf_ep_is_active (n, direction_of_endpoint (arg)))
	return -1;

      if ((arg->index & 0x80))	/* IN endpoint */
	gd32vf_ep_stall (n, USB_DIR_IN);
      else
	gd32vf_ep_stall (n, USB_DIR_OUT);

      return USB_EVENT_SET_FEATURE_ENDPOINT;
    }

  return -1;
}

static int
std_set_address (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT && arg->len == 0 && arg->value <= 127
      && arg->index == 0 && dev->configuration == 0)
    {
      gd32vf_set_daddr (arg->value);
      return usb_lld_ctrl_ack (dev);
    }

  return -1;
}

static int
std_get_descriptor (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  if (USB_SETUP_SET (arg->type))
    return -1;

  return USB_EVENT_GET_DESCRIPTOR;
}

static int
std_get_configuration (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_SET (arg->type))
    return -1;

  if (arg->value != 0 || arg->index != 0 || arg->len != 1)
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    return usb_lld_ctrl_send (dev, &dev->configuration, 1);

  return -1;
}

static int
std_set_configuration (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT && arg->index == 0 && arg->len == 0)
    return USB_EVENT_SET_CONFIGURATION;

  return -1;
}

static int
std_get_interface (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_SET (arg->type))
    return -1;

  if (arg->value != 0 || (arg->index >> 8) != 0 || arg->len != 1)
    return -1;

  if (dev->configuration == 0)
    return -1;

  if (rcp == INTERFACE_RECIPIENT)
    return USB_EVENT_GET_INTERFACE;

  return -1;
}

static int
std_set_interface (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type) || rcp != INTERFACE_RECIPIENT
      || arg->len != 0 || (arg->index >> 8) != 0
      || (arg->value >> 8) != 0 || dev->configuration == 0)
    return -1;

  return USB_EVENT_SET_INTERFACE;
}

static int
handle_setup0 (struct usb_dev *dev)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  int r = -1;
  HANDLER handler;

  debug ("S");
  data_p->addr = NULL;
  data_p->len = 0;
  data_p->require_zlp = 0;

  if ((dev->dev_req.type & REQUEST_TYPE) == STANDARD_REQUEST)
    {
      switch (dev->dev_req.request)
	{
	case 0: handler = std_get_status;  break;
	case 1: handler = std_clear_feature;  break;
	case 3: handler = std_set_feature;  break;
	case 5: handler = std_set_address;  break;
	case 6: handler = std_get_descriptor;  break;
	case 8: handler = std_get_configuration;  break;
	case 9: handler = std_set_configuration;  break;
	case 10: handler = std_get_interface;  break;
	case 11: handler = std_set_interface;  break;
	default: handler = std_none;  break;
	}

      if ((r = (*handler) (dev)) < 0)
	{
	  usb_lld_ctrl_error (dev);
	  return USB_EVENT_OK;
	}
      else
	return r;
    }
  else
    return USB_EVENT_CTRL_REQUEST;
}

static int
handle_in0 (struct usb_dev *dev)
{
  int r = 0;

  if (dev->state == IN_DATA || dev->state == LAST_IN_DATA)
    handle_datastage_in (dev);
  else if (dev->state == WAIT_STATUS_IN)
    { /* Control WRITE transfer done successfully.  */
      if ((dev->dev_req.request == SET_ADDRESS) &&
	  ((dev->dev_req.type & (REQUEST_TYPE | RECIPIENT))
	   == (STANDARD_REQUEST | DEVICE_RECIPIENT)))
	r = USB_EVENT_DEVICE_ADDRESSED;
      else
	r = USB_EVENT_CTRL_WRITE_FINISH;
      dev->state = WAIT_SETUP;
      gd32vf_prepare_ep0_setup (dev);
    }
  else
    {
      dev->state = STALLED;
      gd32vf_ep_stall (ENDP0, USB_DIR_BOTH);
      gd32vf_prepare_ep0_setup (dev);
    }

  return r;
}

static void
handle_out0 (struct usb_dev *dev)
{
  if (dev->state == OUT_DATA)
    /* It's normal control WRITE transfer.  */
    handle_datastage_out (dev);
  else if (dev->state == WAIT_STATUS_OUT)
    { /* Control READ transfer done successfully.  */
      debug ("C R ack");
      dev->state = WAIT_SETUP;
      gd32vf_prepare_ep0_setup (dev);
    }
  else
    {
      debug (".");
      /*
       * dev->state == IN_DATA || dev->state == LAST_IN_DATA
       * (Host aborts the transfer before finish)
       * Or else, unexpected state.
       * STALL the endpoint, until we receive the next SETUP token.
       */
      dev->state = STALLED;
      gd32vf_ep_stall (ENDP0, USB_DIR_BOTH);
      gd32vf_prepare_ep0_setup (dev);
    }
}

void
usb_lld_reset (struct usb_dev *dev, uint8_t feature)
{
  usb_lld_set_configuration (dev, 0);
  dev->feature = feature;
  dev->state = WAIT_SETUP;

  USB_D->DCTL &= ~(1 << 0);     /* Clear wakeup.  */

  /* This TX FIFO flush is required.  With no Tx FIFO flush, writing
     about 300 times, it starts erroneous bytes on wire.  */
  gd32vf_tx_fifo_flush (0);

  gd32vf_set_daddr (0);

  /* Max packet length: 64-byte.  */
  USB_I[0].DIEP_CTL &= ~0x03;

  gd32vf_prepare_ep0_setup (dev);
}

void
usb_lld_setup_endp (struct usb_dev *dev, int n, int ep_type,
		    int rx_en, int tx_en)
{
  (void)dev;

  if (n == ENDP0)
    return;

  if (rx_en)
    {
      if ((USB_O[n].DOEP_CTL & (1 << 15)) == 0)
	{
	  /* Select DATA0 and EP_TYPE.  */
	  USB_O[n].DOEP_CTL = ((1 << 28) | (1 << 27)
			       | (ep_type << 18)
			       | (1 << 15) | 64);
	  USB_O[n].DOEP_CTL &= ~(1 << 21);
	}

      USB_D->DAEPINTEN |= (1 << (16 + n));
    }

  if (tx_en)
    {
      gd32vf_tx_fifo_flush (n);

      if ((USB_I[n].DIEP_CTL & (1 << 15)) == 0)
	{
	  /* Select DATA0, TX FIFO number, and EP_TYPE.	 */
	  /* Initially, no data to send, respond with NAK.
	     Activate the endpoint.  */
	  USB_I[n].DIEP_CTL = ((1 << 28) | (1 << 27)
			       | (n << 22) | (ep_type << 18)
			       | (1 << 15) | 64);
	  USB_I[n].DIEP_CTL &= ~(1 << 21);
	}

      USB_D->DAEPINTEN |= (1 << n);
    }
}


void
usb_lld_set_configuration (struct usb_dev *dev, uint8_t config)
{
  dev->configuration = config;
}

uint8_t
usb_lld_current_configuration (struct usb_dev *dev)
{
  return dev->configuration;
}

int
usb_lld_ctrl_recv (struct usb_dev *dev, void *p, size_t len)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  data_p->addr = (uint8_t *)p;
  data_p->len = len;
  dev->state = OUT_DATA;
  if (len > USB_MAX_PACKET_SIZE)
    len = USB_MAX_PACKET_SIZE;
  gd32vf_prepare_ep0_out (len);
  return USB_EVENT_OK;
}

/*
 * BUF: Pointer to data memory.
 *
 * BUFLEN: size of the data.
 */
int
usb_lld_ctrl_send (struct usb_dev *dev, const void *buf, size_t buflen)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  uint32_t len_asked = dev->dev_req.len;
  uint32_t len;

  debug ("<p %d:%d", buflen, len_asked);

  data_p->addr = (void *)buf;
  data_p->len = buflen;

  /* Restrict the data length to be the one host asks for */
  if (data_p->len >= len_asked)
    data_p->len = len_asked;
  /* ZLP is only required when host doesn't expect the end of packets.  */
  else if (data_p->len != 0 && (data_p->len % USB_MAX_PACKET_SIZE) == 0)
    data_p->require_zlp = 1;

  if (data_p->len <= USB_MAX_PACKET_SIZE)
    {
      len = data_p->len;
      dev->state = LAST_IN_DATA;
    }
  else
    {
      len = USB_MAX_PACKET_SIZE;
      dev->state = IN_DATA;
    }

  gd32vf_prepare_ep0_in (len);

  return USB_EVENT_OK;
}

void
usb_lld_rx_enable_buf (struct usb_dev *dev, int n, void *buf, size_t len)
{
  dev->epctl_rx[n-1].addr = (uint8_t *)buf;
  dev->epctl_rx[n-1].len = (uint16_t)len;

  debug ("\rrx %d (%d)", n, len);
  USB_O[n].DOEP_LEN = (1 << 19) | len; /* 1-packet, len-byte */
  USB_O[n].DOEP_CTL |= (1 << 31) | (1 << 26); /* Enable and Clear NAK */
}


void
usb_lld_tx_enable_buf (struct usb_dev *dev, int n, const void *buf, size_t len)
{
  dev->epctl_tx[n-1].addr = (uint8_t *)buf;
  dev->epctl_tx[n-1].len = (uint16_t)len;

  debug ("\rtx %d (%d)", n, len);
  USB_I[n].DIEP_LEN = (1 << 19) | len; /* 1-packet, len-byte */
  if (len)
    gd32vf_tx_fifo_write (n, buf, len);
  USB_I[n].DIEP_CTL |= (1 << 31) | (1 << 26); /* Enable and Clear NAK */
}
