#! /usr/bin/python3

import usb.core
import usb.util

dev = usb.core.find(idVendor=0xffff, idProduct=0x0001)

if dev is None:
    raise ValueError('Device not found')

# dev.set_configuration()

# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]

ep_out = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_OUT)

ep_in = usb.util.find_descriptor(
    intf,
    # match the first IN endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_IN)

if ep_out is None:
    raise ValueError('Endpoint to send data not found')

if ep_in is None:
    raise ValueError('Endpoint to receive data not found')

def prepare_send():
    w=ep_in.read(64)
    # print(w)

def finish_send():
    dev.ctrl_transfer(0x41, 0x00, 0x00, 0x00, "")
    # print(".", end='', flush=True)


color0 = [ 0xff, 0x03 ]
color1 = [ 0xef, 0x7f ]
color2 = [ 0x00, 0x78 ]
color3 = [ 0xff, 0xff ]

image = color0*160*80
# + color1*160*40
image64 = color0*32
image = color1*160*80
point1600a=(color0*40+color1*40)*20
point1600b=(color1*40+color0*40)*20

point1600c=(color2*20+color3*20)*40
point1600d=(color3*20+color2*20)*40

import time
import sys

for i in range(10):
    prepare_send()

    ep_out.write(point1600a)
    print(".", end='', flush=True)
    ep_out.write(point1600b)
    print(".", end='', flush=True)
    ep_out.write(point1600a)
    print(".", end='', flush=True)
    ep_out.write(point1600b)
    print(".", end='', flush=True)
    ep_out.write(point1600a)
    print(".", end='', flush=True)
    ep_out.write(point1600b)
    print(".", end='', flush=True)
    ep_out.write(point1600a)
    print(".", end='', flush=True)
    ep_out.write(point1600b)
    print(".")
    finish_send()

    prepare_send()
    ep_out.write(point1600c*2+point1600d*2+point1600c*2+point1600d*2)
    print(".")
    finish_send()

prepare_send()
dev.ctrl_transfer(0x41, 0x00, 0x01, 0x00, "")
