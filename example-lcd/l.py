#! /usr/bin/python3

# pos = [
#   0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
#   1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,
#   3,  4,  4,  4,  4,  5,  5,  5,  6,  6,  6,  7,  7,  7,  8,  8,  9,  9, 10, 11,
#  12, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 25, 26, 28, 30, 31, 33, 35,
#  37, 40, 42, 44, 47, 50, 53, 56, 59, 63, 66, 70, 75, 79, 84, 89, 94,100,105,112,
# 118,125,133,141,149,158
# ]

pos = [
  0,  1,  1,  1,
  1,  2,  2,  3,
  4,  6,  8,  11,
 15, 20, 26, 35,
 47, 63, 84, 112,
149
]

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

color_black = [ 0, 0 ]
color0 = [ 0xff, 0x03 ]
color1 = [ 0xef, 0x7f ]
color2 = [ 0x00, 0x78 ]
color3 = [ 0xff, 0xff ]

def gen_image(x):
    image = []
    for i in range(80):
        image = image + (color_black * x) + (color0 * (160-x))
    return image

images = [ gen_image(p) for p in pos ]
# images = [ gen_image(p) for p in range(160) ]

import time
import sys

def prepare_send():
    w=ep_in.read(64)
    # print(w)

def finish_send():
    dev.ctrl_transfer(0x41, 0x00, 0x00, 0x00, "")
    # print(".", end='', flush=True)

for i in range(len(images)*8):
    i0 = i % (len(images)*2)
    if i0 < len(images):
        x = (i % len(images))
    else:
        x = len(images)- 1 - (i % len(images))
    # print("%d %d: %d" % (i, x, len(images[x])))
    if True:
        prepare_send()
        ep_out.write(images[x])
        finish_send()

prepare_send()
dev.ctrl_transfer(0x41, 0x00, 0x01, 0x00, "")
