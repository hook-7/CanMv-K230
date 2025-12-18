'''
Demo Name：Robot line patrol (solid line)
Platform：01Studio CanMV K230
Tutorial：wiki.01studio.cc

# Black Grayscale Line Following Example
#
# Making a robot that follows a robot takes a lot of effort. This example script
# demonstrates how to make a line following robot for the machine vision part. You
# can use the output of this script to drive a differential drive robot
# to follow a line. This script only generates a rotation value (angle of deviation) that indicates
# your robot is pointing left or right.
#
# For this example to work properly, you should aim the camera at a straight line (solid line)
# and adjust the camera to a 45 degree position on the horizontal plane. Make sure there is only 1 straight line in the
# picture.
'''

import time, os, sys, math

from media.sensor import * #Import the sensor module and use the camera API
from media.display import * #Import the display module and use display API
from media.media import * #Import the media module and use meida API

# Trace the black line. Use [(128, 255)] to trace the white line.
GRAYSCALE_THRESHOLD = [(0, 64)]

# Below is a list of roi [region] tuples. Each roi is a rectangle represented by (x, y, w, h).

'''
# The Sampling image QQVGA 160*120, the list divides the image into 3 rectangles of roi, the rectangle closer to the camera
# field of view (usually below the image) has a greater weight.
ROIS = [ # [ROI, weight]
        (0, 100, 160, 20, 0.7), # It can be adjusted according to different robot situations.
        (0,  50, 160, 20, 0.3),
        (0,   0, 160, 20, 0.1)
       ]
'''

# The sampled image is QVGA 320*240. The list divides the image into three rectangles using roi. The rectangles closer to
# the camera's field of view (usually below the image) have greater weights.
ROIS = [ # [ROI, weight]
        (0, 200, 320, 40, 0.7), # It can be adjusted according to different robot situations.
        (0,  100, 320, 40, 0.3),
        (0,   0, 320, 40, 0.1)
       ]

# Calculate the sum of the weights of the above three rectangles. The sum does not necessarily need to be 1.
weight_sum = 0
for r in ROIS: weight_sum += r[4] # r[4] is the rectangle weight value.


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

    centroid_sum = 0

    for r in ROIS:
        blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True) # r[0:4] is the roi tuple defined above.

        if blobs:
            # Find the blob with the most pixels.
            largest_blob = max(blobs, key=lambda b: b.pixels())

            # Draw a rect around the blob.
            img.draw_rectangle(largest_blob.rect())
            img.draw_cross(largest_blob.cx(),
                           largest_blob.cy())

            centroid_sum += largest_blob.cx() * r[4] # r[4] is the weight value of each roi.

    center_pos = (centroid_sum / weight_sum) # Determine the center of the line.

    # Convert the center position of the line into an angle for easier robot processing.
    deflection_angle = 0

    # Use the inverse tangent function to calculate the deviation angle of the center of the line. You can draw a
    # picture to understand it yourself.
    # The weight X coordinate falls on the left half of the image and is recorded as a positive deviation, and falls on
    # the right half and is recorded as a negative deviation, so the calculation result is added with a negative sign.

    #deflection_angle = -math.atan((center_pos-80)/60) #Use when the image is QQVGA 160*120

    deflection_angle = -math.atan((center_pos-160)/120) #Use when the image is QVGA 320*240

    # Convert the deviation value to a deviation angle.
    deflection_angle = math.degrees(deflection_angle)

    # After calculating the deviation angle, the robot can be controlled to make adjustments.
    print("Turn Angle: %f" % deflection_angle)

    # LCD displays the offset angle, and the scale parameter can change the font size
    img.draw_string_advanced(2,2,20, str('%.1f' % deflection_angle), color=(255,255,255))

    #Display.show_image(img) #Dispaly images

    #Display images, only used for LCD center display
    Display.show_image(img, x=round((800-sensor.width())/2),y=round((480-sensor.height())/2))

    print(clock.fps()) #FPS
