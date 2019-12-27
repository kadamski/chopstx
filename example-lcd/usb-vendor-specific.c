#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include <string.h>
#include "board.h"
#include "usb_lld.h"
#include "vrsp.h"

extern uint16_t screen_image[12800];

static chopstx_intr_t usb_intr;

struct vrsp {
  struct usb_dev dev;
  chopstx_mutex_t mtx;
  chopstx_cond_t cnd;
  uint32_t device_state     : 3;     /* USB device status */
  uint32_t screen_mem_owner : 1;     /* 0: driver, 1: app */
  uint32_t mode        : 8;
  uint16_t *screen_ptr;
};

static struct vrsp vrsp0;

/*
 * Locate VRSP structure from interface number or endpoint number.
 * Currently, it always returns vrsp0, because we only have the one.
 */
static struct vrsp *
vrsp_get (int interface, uint8_t ep_num)
{
  struct vrsp *v = &vrsp0;

  (void)interface;
  (void)ep_num;
  return v;
}

static void vrsp_update_finish (struct vrsp *v, int value);

/* USB Device Descriptor */
static const uint8_t vrsp_device_desc[18] = {
  18,   /* bLength */
  DEVICE_DESCRIPTOR,		/* bDescriptorType */
  0x10, 0x01,			/* bcdUSB = 1.1 */
  0x00,				/* bDeviceClass.                    */
  0x00,				/* bDeviceSubClass.                 */
  0x00,				/* bDeviceProtocol.                 */
  0x40,				/* bMaxPacketSize.                  */
  0xFF, 0xFF, /* idVendor  */
  0x01, 0x00, /* idProduct */
  0x00, 0x01, /* bcdDevice  */
  1,				/* iManufacturer.                   */
  2,				/* iProduct.                        */
  3,				/* iSerialNumber.                   */
  1				/* bNumConfigurations.              */
};

#define FEATURE_BUS_POWERED	0x80

/* Configuration Descriptor */
static const uint8_t vrsp_config_desc[32] = {
  9,
  CONFIG_DESCRIPTOR,		/* bDescriptorType: Configuration */
  /* Configuration Descriptor.*/
  32, 0x00,			/* wTotalLength.                    */
  0x01,				/* bNumInterfaces.                  */
  0x01,				/* bConfigurationValue.             */
  0,				/* iConfiguration.                  */
  FEATURE_BUS_POWERED,		/* bmAttributes.                    */
  50,				/* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  9,
  INTERFACE_DESCRIPTOR,
  0x00,		   /* bInterfaceNumber.                */
  0x00,		   /* bAlternateSetting.               */
  0x02,		   /* bNumEndpoints.                   */
  0xFF,		   /* bInterfaceClass (Vendor specific).  */
  0x00,		   /* bInterfaceSubClass.  */
  0x00,		   /* bInterfaceProtocol.  */
  0,	           /* iInterface.                      */
  /* Endpoint 1-OUT Descriptor.*/
  7,
  ENDPOINT_DESCRIPTOR,
  ENDP1,         /* bEndpointAddress.    */
  0x02,          /* bmAttributes (Bulk).             */
  0x40, 0x00,	 /* wMaxPacketSize.                  */
  0x00,		 /* bInterval.                       */
  /* Endpoint 1-IN Descriptor.*/
  7,
  ENDPOINT_DESCRIPTOR,
  ENDP1|0x80,    /* bEndpointAddress.    */
  0x02,          /* bmAttributes (Bulk).             */
  0x40, 0x00,	 /* wMaxPacketSize.                  */
  0x00,		 /* bInterval.                       */
};


/*
 * U.S. English language identifier.
 */
static const uint8_t vrsp_string0[4] = {
  4,				/* bLength */
  STRING_DESCRIPTOR,
  0x09, 0x04			/* LangID = 0x0409: US-English */
};

static const uint8_t vrsp_string1[] = {
  23*2+2,			/* bLength */
  STRING_DESCRIPTOR,		/* bDescriptorType */
  /* Manufacturer: "Flying Stone Technology" */
  'F', 0, 'l', 0, 'y', 0, 'i', 0, 'n', 0, 'g', 0, ' ', 0, 'S', 0,
  't', 0, 'o', 0, 'n', 0, 'e', 0, ' ', 0, 'T', 0, 'e', 0, 'c', 0,
  'h', 0, 'n', 0, 'o', 0, 'l', 0, 'o', 0, 'g', 0, 'y', 0, 
};

static const uint8_t vrsp_string2[] = {
  14*2+2,			/* bLength */
  STRING_DESCRIPTOR,		/* bDescriptorType */
  /* Product name: "Chopstx Sample" */
  'C', 0, 'h', 0, 'o', 0, 'p', 0, 's', 0, 't', 0, 'x', 0, ' ', 0,
  'S', 0, 'a', 0, 'm', 0, 'p', 0, 'l', 0, 'e', 0,
};

/*
 * Serial Number string.
 */
static const uint8_t vrsp_string3[10] = {
  10,				    /* bLength */
  STRING_DESCRIPTOR,		    /* bDescriptorType */
  '0', 0,  '.', 0,  '0', 0, '0', 0, /* Version number */
};

#define NUM_INTERFACES 1

static void
usb_device_reset (struct usb_dev *dev)
{
  usb_lld_reset (dev, FEATURE_BUS_POWERED);

  chopstx_mutex_lock (&vrsp0.mtx);
  vrsp0.device_state = USB_DEVICE_STATE_ATTACHED;
  vrsp0.screen_mem_owner = 0;
  vrsp0.screen_ptr = screen_image;
  vrsp0.mode = 2;
  chopstx_cond_signal (&vrsp0.cnd);
  chopstx_mutex_unlock (&vrsp0.mtx);
}


static void
usb_ctrl_write_finish (struct usb_dev *dev)
{
  (void)dev;
}


static int
usb_setup (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t type_rcp = arg->type & (REQUEST_TYPE|RECIPIENT);

  if (USB_SETUP_SET (arg->type))
    {
      if (type_rcp == (VENDOR_REQUEST | INTERFACE_RECIPIENT)
	  && arg->request == 0x00
	  && arg->index == 0 /* interface 0 */)
	{
	  struct vrsp *v = vrsp_get (arg->index, 0);

	  /* Finish the screen update.  */
	  vrsp_update_finish (v, arg->value);
	  return usb_lld_ctrl_ack (dev);
	}
    }

  return -1;
}

static int
usb_get_descriptor (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;
  uint8_t desc_type = (arg->value >> 8);
  uint8_t desc_index = (arg->value & 0xff);

  if (rcp != DEVICE_RECIPIENT)
    return -1;

  if (desc_type == DEVICE_DESCRIPTOR)
    return usb_lld_ctrl_send (dev,
			      vrsp_device_desc, sizeof (vrsp_device_desc));
  else if (desc_type == CONFIG_DESCRIPTOR)
    return usb_lld_ctrl_send (dev,
			      vrsp_config_desc, sizeof (vrsp_config_desc));
  else if (desc_type == STRING_DESCRIPTOR)
    {
      const uint8_t *str;
      int size;

      switch (desc_index)
	{
	case 0:
	  str = vrsp_string0;
	  size = sizeof (vrsp_string0);
	  break;
	case 1:
	  str = vrsp_string1;
	  size = sizeof (vrsp_string1);
	  break;
	case 2:
	  str = vrsp_string2;
	  size = sizeof (vrsp_string2);
	  break;
	case 3:
	  str = vrsp_string3;
	  size = sizeof (vrsp_string3);
	  break;
	default:
	  return -1;
	}

      return usb_lld_ctrl_send (dev, str, size);
    }

  return -1;
}

static const uint8_t status_zero;

static void
vrsp_setup_endpoints_for_interface (struct usb_dev *dev,
				    uint16_t interface, int stop)
{
  if (interface == 0)
    {
      if (!stop)
	{
          struct vrsp *v = vrsp_get (interface, 0);

	  usb_lld_setup_endp (dev, ENDP1, EP_BULK, 1, 1);

	  if (v->screen_mem_owner == 0)
	    usb_lld_tx_enable_buf (&v->dev, ENDP1, &status_zero, 1);
	}
      else
	{
	  usb_lld_stall_rx (dev, ENDP1);
	  usb_lld_stall_tx (dev, ENDP1);
	}
    }
}

static int
usb_set_configuration (struct usb_dev *dev)
{
  uint8_t current_conf;

  current_conf = usb_lld_current_configuration (dev);
  if (current_conf == 0)
    {
      if (dev->dev_req.value != 1)
	return -1;

      usb_lld_set_configuration (dev, 1);
      vrsp_setup_endpoints_for_interface (dev, 0, 0);
    }
  else if (current_conf != dev->dev_req.value)
    {
      if (dev->dev_req.value != 0)
	return -1;

      usb_lld_set_configuration (dev, 0);
      vrsp_setup_endpoints_for_interface (dev, 0, 1);
    }

  return usb_lld_ctrl_ack (dev);
}


static int
usb_set_interface (struct usb_dev *dev)
{
  uint16_t interface = dev->dev_req.index;
  uint16_t alt = dev->dev_req.value;

  if (interface >= NUM_INTERFACES)
    return -1;

  if (alt != 0)
    return -1;
  else
    {
      vrsp_setup_endpoints_for_interface (dev, interface, 0);
      return usb_lld_ctrl_ack (dev);
    }
}

static int
usb_get_interface (struct usb_dev *dev)
{
  const uint8_t zero = 0;
  uint16_t interface = dev->dev_req.index;

  if (interface >= NUM_INTERFACES)
    return -1;

  /* We don't have alternate interface, so, always return 0.  */
  return usb_lld_ctrl_send (dev, &zero, 1);
}

static int
usb_get_status_interface (struct usb_dev *dev)
{
  const uint16_t status_info = 0;
  uint16_t interface = dev->dev_req.index;

  if (interface >= NUM_INTERFACES)
    return -1;

  return usb_lld_ctrl_send (dev, &status_info, 2);
}

static void
vrsp_update_receive (struct vrsp *v, int len)
{
  chopstx_mutex_lock (&v->mtx);
  if (v->screen_mem_owner == 0 && v->screen_ptr)
    {
      v->screen_ptr += len / 2;
      if (v->screen_ptr < screen_image + 12800)
	usb_lld_rx_enable_buf (&v->dev, ENDP1, v->screen_ptr, 64);
    }
  chopstx_mutex_unlock (&v->mtx);
}

static void
vrsp_update_finish (struct vrsp *v, int value)
{
  chopstx_mutex_lock (&v->mtx);
  if (v->screen_mem_owner == 0 && v->screen_ptr)
    {
      v->screen_ptr = NULL;
      v->mode = value;
      chopstx_cond_signal (&v->cnd);
    }
  chopstx_mutex_unlock (&v->mtx);
}

static void
usb_tx_done (uint8_t ep_num, uint16_t len)
{
  struct vrsp *v = vrsp_get (-1, ep_num);

  (void)len;

  /*
   * Coming here means that notification ("it's ready to accept screen
   * data") is retrieved by host.  Enable the receive endpoint to
   * accept data.
   */

  /* EP_NUM should be ENDP1 in this implementation.  */
  usb_lld_rx_enable_buf (&v->dev, ENDP1, v->screen_ptr, 64);
}

static void
usb_rx_ready (uint8_t ep_num, uint16_t len)
{
  struct vrsp *v = vrsp_get (-1, ep_num);

  if (ep_num == ENDP1)
    vrsp_update_receive (v, len);
}

#define PRIO_VRSP      4

#define STACK_PROCESS_2
#include "stack-def.h"
#define STACK_ADDR_VRSP ((uint32_t)process2_base)
#define STACK_SIZE_VRSP (sizeof process2_base)

static void *
vrsp_main (void *arg)
{
  struct vrsp *v = arg;
  struct usb_dev *dev = &v->dev;
  int e;

  chopstx_claim_irq (&usb_intr, INTR_REQ_USB);
  usb_lld_init (dev, FEATURE_BUS_POWERED);

  while (1)
    {
      chopstx_intr_wait (&usb_intr);
      if (usb_intr.ready)
	{
	  uint8_t ep_num;

	  /*
	   * When interrupt is detected, call usb_lld_event_handler.
	   * The event may be one of following:
	   *    (1) Transfer to endpoint (bulk or interrupt)
	   *        In this case EP_NUM is encoded in the variable E.
	   *    (2) "NONE" event: some trasfer was done, but all was
	   *        done by lower layer, no other work is needed in
	   *        upper layer.
	   *    (3) Device events: Reset or Suspend
	   *    (4) Device requests to the endpoint zero.
	   *
	   */
	  e = usb_lld_event_handler (dev);
	  chopstx_intr_done (&usb_intr);
	  ep_num = USB_EVENT_ENDP (e);

	  if (ep_num != 0)
	    {
	      if (USB_EVENT_TXRX (e))
		usb_tx_done (ep_num, USB_EVENT_LEN (e));
	      else
		usb_rx_ready (ep_num, USB_EVENT_LEN (e));
	    }
	  else
	    switch (USB_EVENT_ID (e))
	      {
	      case USB_EVENT_DEVICE_RESET:
		usb_device_reset (dev);
		continue;

	      case USB_EVENT_DEVICE_ADDRESSED:
		/* The addres is assigned to the device.  We don't
		 * need to do anything for this actually, but in this
		 * application, we maintain the USB status of the
		 * device.  Usually, just "continue" as EVENT_OK is
		 * OK.
		 */
		chopstx_mutex_lock (&v->mtx);
		v->device_state = USB_DEVICE_STATE_ADDRESSED;
		chopstx_cond_signal (&v->cnd);
		chopstx_mutex_unlock (&v->mtx);
		continue;

	      case USB_EVENT_GET_DESCRIPTOR:
		if (usb_get_descriptor (dev) < 0)
		  usb_lld_ctrl_error (dev);
		continue;

	      case USB_EVENT_SET_CONFIGURATION:
		if (usb_set_configuration (dev) < 0)
		  usb_lld_ctrl_error (dev);
		continue;

	      case USB_EVENT_SET_INTERFACE:
		if (usb_set_interface (dev) < 0)
		  usb_lld_ctrl_error (dev);
		continue;

	      case USB_EVENT_CTRL_REQUEST:
		/* Device specific device request.  */
		if (usb_setup (dev) < 0)
		  usb_lld_ctrl_error (dev);
		continue;

	      case USB_EVENT_GET_STATUS_INTERFACE:
		if (usb_get_status_interface (dev) < 0)
		  usb_lld_ctrl_error (dev);
		continue;

	      case USB_EVENT_GET_INTERFACE:
		if (usb_get_interface (dev) < 0)
		  usb_lld_ctrl_error (dev);
		continue;

	      case USB_EVENT_SET_FEATURE_DEVICE:
	      case USB_EVENT_SET_FEATURE_ENDPOINT:
	      case USB_EVENT_CLEAR_FEATURE_DEVICE:
	      case USB_EVENT_CLEAR_FEATURE_ENDPOINT:
		usb_lld_ctrl_ack (dev);
		continue;

	      case USB_EVENT_CTRL_WRITE_FINISH:
		/* Control WRITE transfer finished.  */
		usb_ctrl_write_finish (dev);
		continue;

	      case USB_EVENT_OK:
	      case USB_EVENT_DEVICE_SUSPEND:
	      default:
		continue;
	      }
	}
    }

  return NULL;
}

static int
vrsp_ready_check (void *arg)
{
  struct vrsp *v = arg;
  return v->screen_mem_owner == 0 && v->screen_ptr == NULL;
}


/* API exposed to application.  */

struct vrsp *
vrsp_open (void)
{
  struct vrsp *v = &vrsp0;     /* Now, we only have a single VRSP.  */

  chopstx_mutex_init (&v->mtx);
  chopstx_cond_init (&v->cnd);
  v->device_state = USB_DEVICE_STATE_UNCONNECTED;
  v->screen_mem_owner = 0;
  v->screen_ptr = screen_image;
  v->mode = 1;

  chopstx_create (PRIO_VRSP, STACK_ADDR_VRSP, STACK_SIZE_VRSP, vrsp_main, v);
  return v;
}


void
vrsp_prepare_poll (struct vrsp *v, chopstx_poll_cond_t *poll_desc)
{
  poll_desc->type  = CHOPSTX_POLL_COND;
  poll_desc->ready = 0;
  poll_desc->cond  = &v->cnd;
  poll_desc->mutex = &v->mtx;
  poll_desc->check = vrsp_ready_check;
  poll_desc->arg   = v;
}


/*
 * vrsp_screen_acquire: Acquire the screen memory by the application to
 * update the LCD.
 *
 * Returns  <  0 on error
 *	    >= 0 on success
 */
int
vrsp_screen_acquire (struct vrsp *v)
{
  int r = 0;

  chopstx_mutex_lock (&v->mtx);
  while (1)
    if (v->screen_mem_owner != 0)
      {
	/* something goes wrong, it's your turn already.  */
	r = -1;
	break;
      }
    else if (v->screen_ptr)
      /* The use of screen has not been finished yet.  */
      chopstx_cond_wait (&v->cnd, &v->mtx);
    else
      {
	v->screen_mem_owner = 1;
	r = v->mode;
	break;
      }
  chopstx_mutex_unlock (&v->mtx);
  return r;
}


/*
 * vrsp_screen_release: release the screen memory by the application
 */
void
vrsp_screen_release (struct vrsp *v)
{
  chopstx_mutex_lock (&v->mtx);
  v->screen_mem_owner = 0;
  v->screen_ptr = screen_image;
  chopstx_mutex_unlock (&v->mtx);

  /* Put the notification to host.  */
  usb_lld_tx_enable_buf (&v->dev, ENDP1, &status_zero, 1);
}
