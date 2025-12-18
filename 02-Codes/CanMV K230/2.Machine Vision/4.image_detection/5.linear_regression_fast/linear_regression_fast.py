'''
Demo Name：linear regression fast（Line patrol）
Platform：01Studio CanMV K230
Tutorial：wiki.01studio.cc
'''

import time, os, sys

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API


THRESHOLD = (0, 100)  # Grayscale threshold for black and white images
BINARY_VISIBLE = True # Using binarized images allows you to see what linear regression is doing.
                      # This may reduce FPS.

sensor = Sensor(width=1280, height=960) #Build a camera object and set the camera image length and width to 4:3
sensor.reset() # reset the Camera
sensor.set_framesize(width=640, height=480) # Set the frame size to resolution (320x240), default channel 0
sensor.set_pixformat(Sensor.GRAYSCALE) # Set the output image format, channel 0

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

    #image.binary([THRESHOLD]) # The gray value in the THRESHOLD range becomes white
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()

    # Returns an object similar to find_lines() and find_line_segments().
    # The following functions are used: x1(), y1(), x2(), y2(), length(),
    # theta() (rotation in degrees), rho(), and magnitude().
    #
    # magnitude() represents the linear regression instruction, and its value is (0, INF].
    # 0 represents a circle, and the larger the INF value, the better the linear fit effect.

    line = img.get_regression([(255,255) if BINARY_VISIBLE else THRESHOLD])

    if (line):

        img.draw_line(line.line(), color = 127,thickness=4)

        print(line) #Printing Results

    #Display images, only used for LCD center display
    Display.show_image(img, x=round((800-sensor.width())/2),y=round((480-sensor.height())/2))


    print("FPS %f, mag = %s" % (clock.fps(), str(line.magnitude()) if (line) else "N/A"))

