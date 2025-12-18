'''
Demo Name：RELAY
Platform：01Studio CanMV K230
Tutorial：wiki.01studio.cc
Description: Change the on/off state of the relay by pressing the button
'''

from machine import Pin
from machine import FPIOA
import time


#Configure GPIO2 and GPIO21 to normal GPIO mode
fpioa = FPIOA()
fpioa.set_function(2,FPIOA.GPIO2)
fpioa.set_function(21,FPIOA.GPIO21)

RELAY=Pin(2,Pin.OUT) #Build the relay object
KEY=Pin(21,Pin.IN,Pin.PULL_UP) #Build KEY object

state=0 #Relay on/off status

while True:

    if KEY.value()==0:   # The key is pressed
        time.sleep_ms(10) # Eliminate jitter
        if KEY.value()==0: # Confirm that the key is pressed

            state=not state  # Use not statement instead of ~ statement
            RELAY.value(state) # Relay on/off state flip
            print('KEY')

            while not KEY.value(): # Check if the key is released
                pass
                
                
