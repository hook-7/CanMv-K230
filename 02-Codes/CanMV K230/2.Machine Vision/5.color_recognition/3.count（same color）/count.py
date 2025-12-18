'''
Demo Name：Object counting（same color）
Platform：01Studio CanMV K230
Tutorial：wiki.01studio.cc
'''

import time, os, sys

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API

thresholds = [(18, 72, -13, 31, 18, 83)] # Yellow jumper cap threshold

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

    blobs = img.find_blobs([thresholds[0]]) # Looking for yellow jumper cap

    if blobs:
        for b in blobs:
            tmp=img.draw_rectangle(b[0:4])
            tmp=img.draw_cross(b[5], b[6])

    #Display calculation information
    img.draw_string_advanced(0, 0, 30, 'FPS: '+str("%.3f"%(clock.fps()))+'       Num: '
                             +str(len(blobs)), color = (255, 255, 255))

    Display.show_image(img) # Display image

    print(clock.fps()) #FPS

