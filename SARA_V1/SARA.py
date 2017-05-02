from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, serial
from pymavlink import mavutil  # Needed for command message definitions
from dbscan import *
from constants import *

import time

class SARA:
	def __init__(self):
		self.vehicle = None
		#the gps coordinate of the start of the field
		self.GPS_ORIGIN = []

		#the angle from absolute east to north of field
		self.FIELD_BEARING = 0

		#gps_waypoints (path of navigation)
		self.GPS_WPS = []
		self.currentWP_ID = 0

		#200ft by 200ft map
		self.map = field()

		#run condition
		self.MISSION_ENABLED = False

		#dbscan
		self.dbscan = DBSCAN(4, 100)
		self.current_scanned_cells = []
		self.clusters = []

		self.ser = serial.Serial ('/dev/cu.usbmodem1411', 115200)
		self.file = open("test.csv",'wb')

	def init(self, fieldbearing, gpsorigin):
		# connection_string = "/dev/cu.usbserial-FTZ1626T, 921600"
		# connection_string = "/dev/ttyUSB0,921600"
		# connection_string = "tcp:127.0.0.1:5760"
		connection_string = "/dev/serial/by-id/usb-FTDI_C232HD-DDHSP-0_FTZ1626T-if00-port0,921600"

		# Connect to the Vehicle.
		#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
		#print "\nConnecting to vehicle on: %s" % connection_string
		self.vehicle = connect(connection_string)
		self.vehicle.armed = True
		self.vehicle.home_location = vehicle.location.global_frame
		self.vehicle.groundspeed = 5

		#get field bearing
		self.FIELD_BEARING = self.vehicle.heading
		self.GPS_ORIGIN = self.vehicle.global_frame

		self.GPS_WPS = self.map.waypointGen(self.FIELD_BEARING)

		self.MISSION_ENABLED = True

	def hasReached(self, targetLocation):
	    difference = 0.0001
	    if 	abs(self.vehicle.location.global_frame.lat - targetLocation.lat) < difference and
	    	abs(self.vehicle.location.global_frame.lon - targetLocation.lon) < difference:
	        return True
	    else:
	        return False

	def scan_lidar(self):
		line = ser.readline()
	    data = line.split(",")
	    
	    if(len(data) != 4):
	        continue
	    try:
	        timestamp = int(data[0])
	        dist = float(data[1])
	        angle = float(data[2])
	        quality = int(data[3])

	        gps_from_ph = self.vehicle.location.global_frame
	        pt = point(timestamp, gps_from_ph, [dist,angle,quality])

	        cell_id = self.map.addPoint(pt)

	        if not(cell_id in self.current_scanned_cells):
	        	self.current_scanned_cells.append(cell_id)

	        #string = str(timestamp)+","+str(gps_from_ph[0])+","+str(gps_from_ph[1])+","+str(dist)+","+str(angle)+","+str(quality)+"\n"
	        #file.write(string)
	        #print(string)
	        
	    except ValueError:
	        continue

	def run(self):
		if not self.MISSION_ENABLED:
			return False

		if(hasReached(self.GPS_WPS[self.currentWP_ID])):
			#stop vehicle
			self.vehicle.simple_goto(self.GPS_WPS[self.currentWP_ID], 0)

			self.dbscan.reset(self.current_scanned_cells)
			cs = self.dbscan.analyze() #need to check for overlapping clusters
			for c in cs:
				

			self.currentWP_ID += 1
			#if no more WPS, mission complete
			if(self.currentWP_ID == len(self.GPS_WPS)):
				self.MISSION_ENABLED == False
			else:
				self.vehicle.simple_goto(self.GPS_WPS[self.currentWP_ID], 5)
		else:
			#scan_lidar, simple_goto
			#[DO YOU HAVE TO CALL THIS EVERY LOOP?]
			#self.vehicle.simple_goto(self.GPS_WPS[self.currentWP_ID], 5)
			self.scan_lidar()

		return True
