from load_image_to_screen import Longan_LCD

from gnupg_logo_16bit_160x80 import gnupg_logo_16bit_160x80

image = gnupg_logo_16bit_160x80

pos = [ 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 12, 14, 17, 20, 23, 28, 33, 40, 47, 56  ]

background0 = [ 0x5d, 0xdf ]
background1 = [ 0x7e, 0xdf ]

def fill(i, bg0, bg1):
    if (i % 2) == 0:
        return bg0
    else:
        return bg1

def move_right(x, bg0, bg1, img):
    result = []
    for i in range(80):
        for j in range(x):
            result = result + fill(j, bg0, bg1)
        result = result + (img[i*160*2:(i+1)*160*2])[0:160*2-x*2]
    return result

images = [ bytes(move_right(x, background0, background1, image)) for x in pos ]

lcd = Longan_LCD()

for loop in range(10):
    for i in range(len(pos)):
        lcd.load_image_to_screen(images[len(pos)-1-i])
    for i in range(len(pos)):
        lcd.load_image_to_screen(images[i])
