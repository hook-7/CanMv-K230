'''
Demo Name：QR Code recognition
Platform：01Studio CanMV K230
Tutorial：wiki.01studio.cc
'''

import time, math, os, gc

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API

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


    res = img.find_qrcodes() #Find QR Code

    if len(res) > 0:

        #Display QR code information in images and terminals
        img.draw_rectangle(res[0].rect(), thickness=2)
        img.draw_string_advanced(0, 0, 30, res[0].payload(), color = (255, 255, 255))

        print(res[0].payload()) #Print QR code information on the serial terminal

    Display.show_image(img) #Display images

    print(clock.fps()) #FPS
