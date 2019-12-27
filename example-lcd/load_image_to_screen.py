#! /usr/bin/python3

import usb.core
import usb.util
import time
import sys

class Longan_LCD(object):
    def __init__(self):
        self.dev = usb.core.find(idVendor=0xffff, idProduct=0x0001)

        if self.dev is None:
            raise ValueError('Device not found')

        cfg = self.dev.get_active_configuration()
        intf = cfg[(0,0)]

        self.ep_out = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_OUT)

        self.ep_in = usb.util.find_descriptor(
            intf,
            # match the first IN endpoint
            custom_match = \
            lambda e: \
                usb.util.endpoint_direction(e.bEndpointAddress) == \
                usb.util.ENDPOINT_IN)

        if self.ep_out is None:
            raise ValueError('Endpoint to send data not found')

        if self.ep_in is None:
            raise ValueError('Endpoint to receive data not found')

    def prepare_send(self):
        w=self.ep_in.read(64)
        # print(w)

    def finish_send(self):
        self.dev.ctrl_transfer(0x41, 0x00, 0x00, 0x00, "")
        # print(".", end='', flush=True)

    def load_image_to_screen(self, image):
        self.prepare_send()
        self.ep_out.write(image)
        self.finish_send()
