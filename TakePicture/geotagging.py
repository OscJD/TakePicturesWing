#!/usr/bin/env python
import RPi.GPIO as GPIO
from pexif import JpegFile
from glob import glob
import os
import time
from time import gmtime,strftime
contador=0
totalMapir=0
contadorMapir=0
GPIO.setmode(GPIO.BOARD)
GPIO.setup(33,GPIO.OUT)
indiceLatLon=0
indiceError=0
a=[0]*1000
segtotales=[0]*1000
bandera=True

def interval():
    cont=int(0)
    infile = open("/home/pi/Desktop/TakePicture/TimeStamp.txt", "r")
    for i in infile:
        print cont
        a[cont]=int(i)
        cont=cont+1    
    infile.close()
    b=[0]*(cont-1)
    cont2=0
    for j in range(cont-1):
        rest=a[j+1]-a[j]
        b[cont2]=rest
        cont2=cont2+1
    print (b)
    return b

def intervalMapir():
    cont=0
    files_list=glob(os.path.join('/media/pi/VOLUME1/DCIM/Photo/','*.JPG'))
    files_list.sort()
    for p in files_list:
        ano,mesdia,hms,num=p.split("_")
        hora=int(hms[0]+hms[1])
        minu=int(hms[2]+hms[3])
        seg=int(hms[4]+hms[5])
        segtotales[cont]=hora*3600+minu*60+seg
        cont=cont+1
    c=[0]*(cont-1)
    cont2=0
    for j in range(cont-1):
        rest=segtotales[j+1]-segtotales[j]
        c[cont2]=rest
        cont2=cont2+1
    print c
    return c

def indices(b,c):
    cont=0
    cont2=0
    cont3=0
    for h in range(len(c)):
        resta=abs(c[h]-b[h])
        if(resta>1):
            cont=cont+1
    cont=abs(len(b)-len(c))
    indic=[0]*cont
    for f in range(len(c)):
        print "contador", f
        resta=abs(c[f]-b[cont3])
        print "resta",resta
        if(resta>1):
            print cont2
            indic[cont2]=f
            cont2=cont2+1
            cont3=cont3+1
        cont3=cont3+1
    print indic
    return indic
    

while True:
    try:

          if(os.path.exists('/media/pi/VOLUME1/DCIM/Photo')):
             LLerror=indices(interval(),intervalMapir())
             print LLerror

             piDir="/home/pi/Desktop/MapirPhoto/"
             mapirDir="/media/pi/VOLUME1/DCIM/Photo/{0}/"
             date=strftime("%d_%H_%M",gmtime())
             mapDir=mapirDir.format(date)
             comCreate="sudo mkdir {0}"
             comCreate=comCreate.format(mapDir)
             os.system(comCreate)
             comMove="sudo mv "
             #comMove=comMove.format(" ",mapDir)
             piDir=piDir+date+"/"
             os.system("sudo mkdir "+piDir)  
             files_list=glob(os.path.join('/media/pi/VOLUME1/DCIM/Photo','*.JPG'))
             files_list.sort()
             infile = open("/home/pi/Desktop/TakePicture/LatLon.txt", "r")
             for j in infile:
                  contador=contador+1
             infile.close()
             infile = open("/home/pi/Desktop/TakePicture/LatLon.txt", "r")
             latlonTot=infile.readlines()
             print latlonTot
             infile.close()
             for k in files_list:
                  totalMapir=totalMapir+1

             contadorMapir= totalMapir-contador
             GPIO.output(33,True)
             for i in files_list:
                      while(bandera==True):
                          for kk in LLerror:
                              if(indiceLatLon==kk):
                                  bandera=True
                                  indiceLatLon=indiceLatLon+1
                                  break
                              else:
                                  bandera=False
                      print(indiceLatLon)    
                      cadena=latlonTot[indiceLatLon]
                      latlon,trash=cadena.split("\n")
                      #print latlon
                      lat,lon=cadena.split(",")
                      print(float(lat),float(lon))
                      ef = JpegFile.fromFile(i)
                      ef.set_geo(float(lat), float(lon))
                      ef.writeFile(piDir+os.path.basename(i))
                      indiceLatLon=indiceLatLon+1
                      bandera=True
                    
                        
                           
             for k in files_list:
                  print k
                  comMov=comMove+k+" "+mapDir
                  print comMov
                  os.system(comMov)
                  

             
             GPIO.output(33,False)
             time.sleep(0.2)
             os.remove("/home/pi/Desktop/TakePicture/LatLon.txt")
             os.remove("/home/pi/Desktop/TakePicture/TimeStamp.txt")
             os.system("sudo eject /media/pi/VOLUME1/")
          else:
             print("Camara no detectada")

          time.sleep(5)
    except:
          GPIO.output(33,True)
          time.sleep(3)
          GPIO.output(33,False)
          print "No hay lat/lon"
          time.sleep(3)
     
