#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import numpy as np
import os


def offPi(channel):
    
    os.system("sudo shutdown -h now")
    print "ahora"
    
   

GPIO.setmode(GPIO.BOARD)
GPIO.setup(29,GPIO.IN)
GPIO.add_event_detect(29,GPIO.FALLING,callback=offPi)


while(True):
   time.sleep(50)
   print "nada"
