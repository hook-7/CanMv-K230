'''
Demo Name：Draw
Platform：01Studio CanMV K230
Description: Draw various graphics and write characters, and display them through IDE and LCD.
Tutorial：wiki.01studio.cc
'''

import time, os, sys

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API

sensor = Sensor() #Constructing a camera object
sensor.reset() #reset the Camera
#sensor.set_framesize(Sensor.FHD) #Set frame size to FHD (1920x1080), default channel 0
sensor.set_framesize(width=800,height=480) #Set frame size to 800x480,mipi LCD,channel0
sensor.set_pixformat(Sensor.RGB565) #Set the output image format, channel 0

##############################################################
## 3 different ways to display images (modify annotations)
#############################################################

#Display.init(Display.VIRT, sensor.width(), sensor.height()) #Displaying images via IDE buffer
#Display.init(Display.LT9611, to_ide=True) #Displaying images via HDMI
Display.init(Display.ST7701, to_ide=True) #Display images through 01Studio 3.5-inch mipi display

MediaManager.init() #Initialize the media resource manager

sensor.run() #Start the camera

clock = time.clock()

while True:

    ####################
    ## Write codes here
    ####################
    clock.tick()

    img = sensor.snapshot() #Take a picture

    # Draw a line segment: from x0, y0 to x1, y1, color red, line width 2.
    img.draw_line(20, 20, 100, 20, color = (255, 0, 0), thickness = 2)

    #Draw a rectangle: green without filling.
    img.draw_rectangle(150, 20, 100, 30, color = (0, 255, 0), thickness = 2, fill = False)

    #Draw a circle: no blue fill.
    img.draw_circle(60, 120, 30, color = (0, 0, 255), thickness = 2, fill = False)

    #Draw arrow: white.
    img.draw_arrow(150, 120, 250, 120, color = (255, 255, 255), size = 20, thickness = 2)

    #Draw Cross
    img.draw_cross(60, 200, color = (255, 255, 255), size = 20, thickness = 2)

    #Draw String
    #img.draw_string(150, 200, "Hello 01Studio!", color = (255, 255, 255), scale = 4, mono_space = False)

    #Write characters, support Chinese.
    img.draw_string_advanced(150, 180, 30, "Hello 01Studio", color = (255, 255, 255))
    img.draw_string_advanced(40, 300, 30, "人生苦短, 我用Python", color = (255, 255, 255))

    Display.show_image(img)

    print(clock.fps()) #FPS


