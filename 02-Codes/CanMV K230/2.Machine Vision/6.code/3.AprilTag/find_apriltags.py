'''
Demo Name：AprilTags recognition
Platform：01Studio CanMV K230
Tutorial：wiki.01studio.cc/en
Note: It is recommended to use QVGA (320x240) resolution. If the resolution is too high, the frame rate will drop.
'''

import time, math, os, gc

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API


# The apriltag code supports processing up to 6 tag families at the same time.
# The returned tag object will have its tag family and its id within the tag family.

tag_families = 0
tag_families |= image.TAG16H5 # Comment out to disable this family
tag_families |= image.TAG25H7 # Comment out to disable this family
tag_families |= image.TAG25H9 # Comment out to disable this family
tag_families |= image.TAG36H10 # Comment out to disable this family
tag_families |= image.TAG36H11 # Comment out to disable this family (default family)
tag_families |= image.ARTOOLKIT # Comment out to disable this family

#What is the difference between tag families? So, for example, the TAG16H5 family is actually a 4x4 square tag.
#So, this means that it can be seen at a longer distance than the 6x6 TAG36H11 tag.
#However, the lower H value (H5 vs. H11), means that the false positive rate of the 4x4 tag is much higher than the 6x6 tag.
# So, unless you have a reason to use another tag family, use the default family TAG36H11.

def family_name(tag):
    if(tag.family() == image.TAG16H5):
        return "TAG16H5"
    if(tag.family() == image.TAG25H7):
        return "TAG25H7"
    if(tag.family() == image.TAG25H9):
        return "TAG25H9"
    if(tag.family() == image.TAG36H10):
        return "TAG36H10"
    if(tag.family() == image.TAG36H11):
        return "TAG36H11"
    if(tag.family() == image.ARTOOLKIT):
        return "ARTOOLKIT"


sensor = Sensor(width=1280, height=960) #Build a camera object and set the camera's length and width to 4:3
sensor.reset() # reset the Camera
sensor.set_framesize(width=320, height=240) # Set the frame size to LCD resolution, channel 0
sensor.set_pixformat(Sensor.RGB565) # Set the output image format, channel 0

#Use 3.5-inch mipi screen and IDE buffer to display images at the same time
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


    for tag in img.find_apriltags(families=tag_families): # If no family is given, TAG36H11 is the default.

            img.draw_rectangle(tag.rect(), color = (255, 0, 0), thickness=4)
            img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0), thickness=2)
            print_args = (family_name(tag), tag.id(), (180 * tag.rotation()) / math.pi) #Print label information
            print("Tag Family %s, Tag ID %d, rotation %f (degrees)" % print_args)

    #Display.show_image(img) # Display images

    #Display pictures, LCD centered display
    Display.show_image(img, x=round((800-sensor.width())/2),y=round((480-sensor.height())/2)) #显示图片

    print(clock.fps()) #FPS
