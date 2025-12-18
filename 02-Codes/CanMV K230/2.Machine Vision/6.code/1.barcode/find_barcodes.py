'''
Demo Name：Barcode recognition
Platform：01Studio CanMV K230
Tutorial：wiki.01studio.cc
'''

import time, math, os, gc

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API

#Define barcode type
def barcode_name(code):
    if(code.type() == image.EAN2):
        return "EAN2"
    if(code.type() == image.EAN5):
        return "EAN5"
    if(code.type() == image.EAN8):
        return "EAN8"
    if(code.type() == image.UPCE):
        return "UPCE"
    if(code.type() == image.ISBN10):
        return "ISBN10"
    if(code.type() == image.UPCA):
        return "UPCA"
    if(code.type() == image.EAN13):
        return "EAN13"
    if(code.type() == image.ISBN13):
        return "ISBN13"
    if(code.type() == image.I25):
        return "I25"
    if(code.type() == image.DATABAR):
        return "DATABAR"
    if(code.type() == image.DATABAR_EXP):
        return "DATABAR_EXP"
    if(code.type() == image.CODABAR):
        return "CODABAR"
    if(code.type() == image.CODE39):
        return "CODE39"
    if(code.type() == image.PDF417):
        return "PDF417"
    if(code.type() == image.CODE93):
        return "CODE93"
    if(code.type() == image.CODE128):
        return "CODE128"


sensor = Sensor() #Constructing a camera object
sensor.reset() # reset the Camera
sensor.set_framesize(width=800, height=480) # Set the frame size to LCD resolution (800x480), channel 0
sensor.set_pixformat(Sensor.RGB565) # Set the output image format, channel 0

#Use 3.5-inch mipi screen and IDE buffer to display images at the same time, 800x480 resolution
Display.init(Display.ST7701, to_ide=True)
#Display.init(Display.VIRT, sensor.width(), sensor.height()) ##Use only the IDE buffer to display images

MediaManager.init() #Initialize the media resource manager

sensor.run() #Start the camera

clock = time.clock()

while True:

    ####################
    ## Write codes here
    ####################
    clock.tick()

    img = sensor.snapshot() # Take a picture

    codes = img.find_barcodes() # Find all barcodes in an image

    for code in codes:

        #Draw a rectangle to represent the barcode
        img.draw_rectangle(code.rect(),thickness=2)

        #Print related information
        print_args = (barcode_name(code), code.payload(), (180 * code.rotation()) / math.pi, code.quality())
        print("Barcode %s, Payload \"%s\", rotation %f (degrees), quality %d" % print_args)

        img.draw_string_advanced(0, 0, 30, code.payload(), color = (255, 255, 255)) #Image display barcode information

    Display.show_image(img) #Display images

    print(clock.fps()) #FPS
