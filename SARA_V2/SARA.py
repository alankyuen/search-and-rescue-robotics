from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil  # Needed for command message definitions
from dbscan import *
from constants import *
from field import *
import time, serial, os.path, random

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
        self.dbscan = DBSCAN(30, 50)
        self.current_scanned_cells = []
        self.clusters = []

        #serial port to arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600)

        #data files for TESTING/RECORDING
        fname_count = 0
        fname = str(fname_count)+".csv"
        while(os.path.isfile(fname)):
            fname_count += 1
            fname = str(fname_count)+".csv"
        print("***************************")
        print(fname)
        print("***************************")
        self.points_file = open(fname,'wb')
        self.buckets_file = open("buckets.csv",'wb')

    def init(self):
        # connection_string = "/dev/cu.usbserial-FTZ1626T, 921600"
        # connection_string = "/dev/ttyUSB0,921600"
        # connection_string = "tcp:127.0.0.1:5760"
        connection_string = "/dev/serial/by-id/usb-FTDI_C232HD-DDHSP-0_FTZ1626T-if00-port0,921600"

        # Connect to the Vehicle.
        self.vehicle = connect(connection_string)
        
        #wait for vehicle to be armable
        while not self.vehicle.is_armable:
            print "Waiting for is_armable..."
            time.sleep(2)

        #if armable: arm vehicle
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        print"Vehicle is armed"

        #wait for home_location GPS fix
        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not self.vehicle.home_location:
                print "Waiting for home location..."
        print "Home Location Fixed: {}".format(self.vehicle.home_location)

        self.vehicle.groundspeed = 3

        #get field bearing
        self.FIELD_BEARING = (-self.vehicle.heading + 450.0)%360.0
        print("*****@*@*@*@*@*@*@*FIELDBEARING",self.FIELD_BEARING)

        #very corner of the field
        self.GPS_ORIGIN = self.vehicle.home_location

        #END:   33.7161505,-117.8305266
        #START: 33.7161308,-117.8305199
        START = LocationGlobal(33.7161308,-117.8305199, 0)
        END = LocationGlobal(33.7161505,-117.8305266, 0)
        self.GPS_WPS = [START, END, START] 
        #self.map.waypointGen(self.GPS_ORIGIN, self.FIELD_BEARING)

        """#TEST 5/3##########
        print(len(self.GPS_WPS))
        for g in self.GPS_WPS: 
            printGPS(g)
        ###################"""
	
        self.MISSION_ENABLED = True

        #self.vehicle.simple_goto(self.GPS_WPS[self.currentWP_ID], 5)

    def deconstruct(self):
        self.points_file.close()
        self.buckets_file.close()
        self.vehicle.armed = False
        self.vehicle.close()

    def hasReached(self, targetLocation):
        difference = 0.00005
        if abs(self.vehicle.location.global_frame.lat - targetLocation.lat) < difference and abs(self.vehicle.location.global_frame.lon - targetLocation.lon) < difference:
            return True
        else:
            return False

    def scan_lidar(self):
        line = self.ser.readline()
        data = line.split(",")
            
        if(len(data) != 4):
            return

        try:
            timestamp = int(data[0])
            dist = float(data[1])
            angle = float(data[2])
            quality = int(data[3])
            if(quality < 5):
                return
            gps_from_ph = self.vehicle.location.global_frame

            robot_bearing = (-self.vehicle.heading + 450)%360 - self.FIELD_BEARING + 90
            pt = point(timestamp, gps_from_ph, [angle,dist,quality], robot_bearing)
            pt.calculateAbsPos(self.GPS_ORIGIN, self.FIELD_BEARING)
    
            cell_id = self.map.addPoint(pt)

            if not(cell_id in self.current_scanned_cells):
                self.current_scanned_cells.append(cell_id)

            #TESTING LIDAR DATA
            point_string = str(timestamp)+","+str(quality)+","+str(pt.abs_ft[0])+","+str(pt.abs_ft[1])+","+str(gps_from_ph.lat)+","+str(gps_from_ph.lon)+","+str(dist)+","+str(angle)+","+str(robot_bearing)+"\n"
            self.points_file.write(point_string)
        except ValueError:
            return

    def run(self):
        if not self.MISSION_ENABLED:
            return False

        #print "Bearing: {}".format(get_bearing(self.vehicle.location.global_frame,self.GPS_WPS[self.currentWP_ID]))
        #checks if robot has reached the current destination waypoint
        if(self.hasReached(self.GPS_WPS[self.currentWP_ID])):
            print "reached wp"
            #stop vehicle
            self.vehicle.simple_goto(self.GPS_WPS[self.currentWP_ID], 0)
            time.sleep(5)

            #populate pt_arr
            pt_arr = []
            for cell_id in self.current_scanned_cells:
                for pt in self.map.cells[cell_id[0]][cell_id[1]]:
                    pt_arr.append(pt)

            #init dbscan
            if not(pt_arr is None or len(pt_arr) == 0):
                self.dbscan.reset(pt_arr)

            #dbscan
    		cs = self.dbscan.analyze() #need to check for overlapping clusters
           	for centroid in cs:
           		centroid_string = str(centroid[0])+","+str(centroid[1])+"\n"
        		self.buckets_file.write(centroid_string)
           	
        	self.currentWP_ID += 1
            
            #if no more WPS, mission complete
        	if(self.currentWP_ID == len(self.GPS_WPS)):
          		self.MISSION_ENABLED == False
        	else:
            	self.vehicle.simple_goto(self.GPS_WPS[self.currentWP_ID], 5)

        #if robot is still travelling to the current waypoint
        else:
            self.scan_lidar()

        """
        TESTING 5/16
        robot_bearing = (-self.vehicle.heading + 450)%360 - self.FIELD_BEARING + 90
        pt = point(0, self.vehicle.location.global_frame, [0,0,50], robot_bearing)
        pt.calculateAbsPos(self.GPS_ORIGIN, self.FIELD_BEARING)
        point_string = str(pt.abs_ft[0])+","+str(pt.abs_ft[1])+"\n"
        self.gps_pos_file.write(point_string)
        if(self.heading_count < 200):
            self.avg_r_heading += robot_bearing
            self.avg_p_heading += self.vehicle.heading
            self.heading_count += 1.0
            print"Avg Robot Heading: {} Avg PH Heading: {}".format(self.avg_r_heading/self.heading_count, self.avg_p_heading/self.heading_count)
        """
        return True

