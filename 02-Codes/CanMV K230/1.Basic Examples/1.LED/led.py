'''
Demo Name：Light up the Blue LED
Version：v1.0
Author：01Studio
Platform：01Studio CanMV K230
Tutorial：wiki.01studio.cc
'''

from machine import Pin
from machine import FPIOA
import time

#Configure GPIO52 as a normal GPIO
fpioa = FPIOA()
fpioa.set_function(52,FPIOA.GPIO52)

LED=Pin(52,Pin.OUT) #Construct LED object, GPIO52, output
LED.value(0) #To turn on the LED, you can also use led.on()
