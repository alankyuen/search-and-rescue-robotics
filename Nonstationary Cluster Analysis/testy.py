import DBSCAN3, serial
from DBSCAN3 import *

ser = serial.Serial ('/dev/cu.usbmodem1411', 115200)

file = open("test.csv",'wb')
soccer_field = field()

field_bearing = 30.0

while False:
    line = ser.readline()
    data = line.split(",")
    
    if(len(data) != 4):
        continue
    try:
        timestamp = int(data[0])
        dist = float(data[1])
        angle = float(data[2])
        quality = int(data[3])

        gps_from_ph = [33.703317, -117.779587]
        pt = point(timestamp, gps_from_ph, [dist,angle,quality])
        string = str(timestamp)+","+str(gps_from_ph[0])+","+str(gps_from_ph[1])+","+str(dist)+","+str(angle)+","+str(quality)+"\n"
        file.write(string)
        print(string)
        #soccer_field.addPoint(pt)
    except ValueError:
        continue

    

#0) 
#A) get field bearing & store in field: read compasscreate field
#B) get gps waypoints & store in field: waypointGen()
#C) read raw_measurement from pyserial, read gps from pixhawk, store into point object
#D) add point to field
#E) check which cells to scan and analyze
#G) 
#F) get results from DBSCAN and store into field
