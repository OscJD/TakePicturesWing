#!/usr/bin/env python
from pexif import JpegFile
import RPi.GPIO as GPIO
from picamera import PiCamera
import time
import cv2
import numpy as np
import time
from time import gmtime,strftime
from dronekit import connect, VehicleMode
from droneFunctions import *
import argparse
import sys 
LatName="{0}"
LonName="{0}"
time.sleep(20)
DateName="/home/pi/Desktop/TakePicture/{0}.txt"
date=strftime("%Y-%m-%d-%H:%M",gmtime())

def take(channel):
    
    hour=strftime("%Y-%m-%d-%H:%M:%S",gmtime())
    HourStamp=strftime("%H",gmtime())
    MinuteStamp=strftime("%M",gmtime())
    SecondStamp=strftime("%S",gmtime())
    TimeStamp=int(HourStamp)*3600+int(MinuteStamp)*60+int(SecondStamp)
    camera.capture(PicName.format(hour))
        
    time.sleep(0.02)

    lat=vehicle.location.global_frame.lat
    on=vehicle.location.global_frame.lon
    print lat
    print lon
    infile = open("/home/pi/Desktop/TakePicture/TimeStamp.txt", "a")
    infile.write(str(TimeStamp))
    infile.write("\n")
    infile.close()
    infile = open("/home/pi/Desktop/TakePicture/LatLon.txt", "a")
    infile.write(LatName.format(lat))
    infile.write(",")
    infile.write(LonName.format(lon))
    infile.write("\n")
    infile.close()
    infile = open(DateName.format(date), "a")
    infile.write(LatName.format(lat))
    infile.write(",")
    infile.write(LonName.format(lon))
    infile.write("\n")
    infile.close()
    ef = JpegFile.fromFile(PicName.format(hour))
    ef.set_geo(float(lat), float(lon))
    ef.writeFile(PicName.format(hour))

    move_servo(vehicle,11,1000)
    time.sleep(0.05)
    move_servo(vehicle,11,2000)
    time.sleep(0.1)
    move_servo(vehicle,11,1000)
    time.sleep(0.1)
    print "ahora"
    
   

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(3,GPIO.IN)
GPIO.add_event_detect(3,GPIO.RISING,callback=take)
p=GPIO.PWM(11,50)

p.start(5)
time.sleep(0.5)
#GPIO.add_event_callback(3,take)



# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution=(2592,1944)
PicName="/home/pi/Desktop/PiCamera/Picture{0}.jpg"
lat=0
lon=0

############################################################
#Set up option parsing to get connection string
while(True):
    try: 
        parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
        parser.add_argument('--connect', 
                           help="vehicle connection target string. If not specified, SITL automatically started and used.")
        args = parser.parse_args()
        #connection_string = args.connect
        connection_string = "/dev/serial0"
        sitl = None

        #Start SITL if no connection string specified
        if not connection_string:
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            #connection_string = sitl.connection_string()
            connection_string = "--connect /dev/serial0"

        # Connect to the Vehicle. 
        #   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
        print "\nConnecting to vehicle on: %s" % connection_string
        vehicle = connect(connection_string,baud=57600,wait_ready=True)
    except:
        print "Error de conexion"
    #####################################################RP########

    try:

        while(True):
            GPIO.output(13,True)
            lat=vehicle.location.global_frame.lat
            lon=vehicle.location.global_frame.lon
            print lat
            print lon
            time.sleep(1)
            GPIO.output(13,False)
            time.sleep(1)
        vehicle.close()    
            
    except:
        print "cerrando conexion con mavlink"
        
