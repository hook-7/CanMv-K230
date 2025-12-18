'''
Demo Name：find circles
Platform：01Studio CanMV K230
Tutorial：wiki.01studio.cc
Description: It is recommended to use a resolution below 320x240.
             A resolution that is too high will cause the frame rate to drop.
'''

import time, os, sys

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API

sensor = Sensor(width=1280, height=960) #Build a camera object and set the camera image length and width to 4:3
sensor.reset() # reset the Camera
sensor.set_framesize(width=320, height=240) #Set the frame size to resolution (320x240), default channel 0
sensor.set_pixformat(Sensor.RGB565) #Set the output image format, channel 0

Display.init(Display.ST7701, to_ide=True) #Use 3.5-inch mipi screen and IDE buffer to display images at the same time
#Display.init(Display.VIRT, sensor.width(), sensor.height()) #Use only the IDE buffer to display images

MediaManager.init() #Initialize the media resource manager

sensor.run() #Start the camera

clock = time.clock()

while True:

    ####################
    ## Write codes here
    ####################
    clock.tick()

    img = sensor.snapshot() # Take a picture

    # The circle class has 4 parameter values: center (x, y), r (radius) and magnitude;
    # The larger the magnitude, the higher the credibility of the identified circle.
    # `threshold` The parameter controls the number of circles found, and increasing the value will reduce the total #             number of recognized circles.
    # `x_margin`, `y_margin`, and `r_margin` Controls the merge adjustment for detected approaching circles.
    # r_min, r_max, and r_step Used to specify the radius range of the test circle.
    for c in img.find_circles(threshold = 2000, x_margin = 10, y_margin= 10,
                              r_margin = 10,r_min = 2, r_max = 100, r_step = 2):
        #Draw a red circle as an indication
        img.draw_circle(c.x(), c.y(), c.r(), color = (255, 0, 0),thickness=2)

        print(c) #Print circular information

    #Display.show_image(img) #Display images

    #Display images, only used for LCD center display
    Display.show_image(img, x=round((800-sensor.width())/2),y=round((480-sensor.height())/2))

    print(clock.fps()) #FPS
