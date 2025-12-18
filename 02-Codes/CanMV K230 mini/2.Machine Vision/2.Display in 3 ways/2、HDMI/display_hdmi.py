'''
Demo Name：Display in 3 ways
Platform：01Studio CanMV K230
Description: Camera image acquisition through IDE, HDMI and MIPI screen display
Tutorial：wiki.01studio.cc
'''

import time, os, sys

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API

sensor = Sensor() #Constructing a camera object
sensor.reset() #reset the Camera
sensor.set_framesize(Sensor.FHD) #Set frame size to FHD (1920x1080), default channel 0
#sensor.set_framesize(width=800,height=480) #Set frame size to 800x480,mipi LCD,channel0
sensor.set_pixformat(Sensor.RGB565) #Set the output image format, channel 0

##############################################################
## 3 different ways to display images (modify annotations)
#############################################################

#Display.init(Display.VIRT, sensor.width(), sensor.height()) #Displaying images via IDE buffer
Display.init(Display.LT9611, to_ide=True) #Displaying images via HDMI
#Display.init(Display.ST7701, to_ide=True) #Display images through 01Studio 3.5-inch mipi display

MediaManager.init() #Initialize the media resource manager

sensor.run() #Start the camera

clock = time.clock()

while True:

    ####################
    ## Write codes here
    ####################
    clock.tick()

    img = sensor.snapshot() #Take a picture

    Display.show_image(img) #Show the Picture

    print(clock.fps()) #FPS
