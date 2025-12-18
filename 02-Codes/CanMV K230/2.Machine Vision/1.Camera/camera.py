'''
Demo Name：Camera
Platform：01Studio CanMV K230
Description: Realize camera image acquisition and display
Tutorial：wiki.01studio.cc
'''

import time, os, sys

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API

sensor = Sensor() #Constructing a camera object
sensor.reset() #reset the Camera
sensor.set_framesize(Sensor.FHD) #Set frame size to FHD (1920x1080), default channel 0
sensor.set_pixformat(Sensor.RGB565) #Set the output image format, channel 0

#Use IDE buffer to output images, the display size is consistent with sensor configuration.
Display.init(Display.VIRT, sensor.width(), sensor.height())

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

