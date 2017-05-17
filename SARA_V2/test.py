from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil  # Needed for command message definitions
from dbscan import *
from constants import *
from field import *
import time, serial, os.path,random, csv

GPS_ORIGIN = LocationGlobal(33.6725012,-117.7763352,0)
FIELD_BEARING = 44.0#44.0
pt_arr = []
f = open('23.csv', 'rb')

reader = csv.reader(f)
for row in reader:
	"""
	timestamp = float(row[0])
	dist = float(row[6])
	angle = float(row[7])
	gps_from_ph = LocationGlobal(float(row[4]),float(row[5]),0)
	robot_bearing = 90
	"""

	"""
	timestamp = 0
	dist = 0
	angle = 0
	gps_from_ph = LocationGlobal(0,0,0)
	robot_bearing = 90
	pt = point(timestamp, gps_from_ph, [angle,dist,50],robot_bearing)
	pt.abs_ft = [float(row[0])*30.48,float(row[1])*30.48]
	pt_arr.append(pt)
	"""

	
	timestamp = float(row[0])
	dist = float(row[6])
	angle = float(row[7])
	gps_from_ph = LocationGlobal(float(row[4]),float(row[5]),0)
	robot_bearing = 90
	pt = point(timestamp, gps_from_ph, [angle,dist,50],robot_bearing)
	pt.calculateAbsPos(GPS_ORIGIN,FIELD_BEARING)
	#pt.abs_ft[0] *= 3.048
	#pt.abs_ft[1] *= 3.048
	pt_arr.append(pt)
	
f.close()

dbscan = DBSCAN(30, 5/3.048)
dbscan.reset(pt_arr)
cs = dbscan.analyze() #need to check for overlapping clusters
for centroid in cs:
	centroid_string = str(centroid[0])+","+str(centroid[1])+"\n"
	print(centroid_string)
	print(calcGPS_from_map(GPS_ORIGIN,35,[centroid[0],centroid[1]]))

"""

*****rel_bearing = radians(180.0-((450.0 - field_bearing + get_bearing(gps_origin,self.abs_gps))%360.0))TRUE:
33.6725319676,-117.776295425
33.6725733468,-117.776313963
33.6725662433,-117.776251834
33.6726425098,-117.776285502

***WHY NEGATIVE***
NON-NEGATED
1m: 33.6725428287,-117.776314063
2m: 33.6725424838,-117.776261
3m: 33.672588641,-117.77629027
4m: 33.6725883958,-117.776192639

print"error for 1m bucket: {}".format(haversine(LocationGlobal(33.6725287955,-117.776289617,0),LocationGlobal(33.6725319676,-117.776295425,0)))
print"error for 2m bucket: {}".format(haversine(LocationGlobal(33.6725672756,-117.776302781,0),LocationGlobal(33.6725733468,-117.776313963,0)))
print"error for 3m bucket: {}".format(haversine(LocationGlobal(33.6725576773,-117.776236696,0),LocationGlobal(33.6725662433,-117.776251834,0)))
print"error for 4m bucket: {}".format(haversine(LocationGlobal(33.6726292785,-117.776263772,0),LocationGlobal(33.6726425098,-117.776285502,0)))
"""


