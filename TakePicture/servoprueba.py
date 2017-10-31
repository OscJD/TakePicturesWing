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
            
            move_servo(vehicle,3,1000)
            time.sleep(2)
            move_servo(vehicle,3,2000)
            time.sleep(2)
            move_servo(vehicle,3,1000)
            time.sleep(2)
            print "ahora"

        vehicle.close()    
            
    except:
        print "cerrando conexion con mavlink"
        
