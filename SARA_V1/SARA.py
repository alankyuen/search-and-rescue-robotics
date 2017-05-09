from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil  # Needed for command message definitions
from dbscan import *
from constants import *
from field import field
import time, serial

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

        #serial port to arduino
        self.ser = serial.Serial('/dev/ttyACM0', 115200)

        #data files for recording 
        self.points_file = open("points.csv",'wb')
        self.buckets_file = open("buckets.csv",'wb')

    def init(self):
        # connection_string = "/dev/cu.usbserial-FTZ1626T, 921600"
        # connection_string = "/dev/ttyUSB0,921600"
        # connection_string = "tcp:127.0.0.1:5760"
        connection_string = "/dev/serial/by-id/usb-FTDI_C232HD-DDHSP-0_FTZ1626T-if00-port0,921600"

        # Connect to the Vehicle.
        #   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
        #print "\nConnecting to vehicle on: %s" % connection_string
        self.vehicle = connect(connection_string)
        
        #wait for vehicle to be armable
        while not self.vehicle.is_armable:
            print "Waiting for is_armable..."
            time.sleep(1)

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

        """
        while self.vehicle.gps_0.fix_type < 2:
            print "Waiting for GPS...", vehcile.gps_o.fix_type
            time.sleep(1)
        """

        self.vehicle.groundspeed = 3

        #get field bearing
        self.FIELD_BEARING = (-self.vehicle.heading + 450.0)%360.0
        #very corner of the field
        self.GPS_ORIGIN = self.vehicle.home_location

        #TEST 5/3##########
        print"vehicle.home_location vs GPS_ORIGIN: {} vs {}".format(self.vehicle.home_location, self.GPS_ORIGIN)
        print"field_bearing: {} vs robot_heading {}".format(self.FIELD_BEARING,self.vehicle.heading)
        ###################


        self.GPS_WPS = [LocationGlobal(33.716175, -117.830442, 0),LocationGlobal(33.716041, -117.830221, 0), LocationGlobal(33.716175, -117.830442, 0),LocationGlobal(33.716034, -117.830611, 0)]#self.map.waypointGen(self.GPS_ORIGIN, self.FIELD_BEARING)

        #TEST 5/3##########
        print(len(self.GPS_WPS))
        for g in self.GPS_WPS: 
            printGPS(g)
        ###################

        self.MISSION_ENABLED = True

        while not(self.hasReached(self.GPS_WPS[self.currentWP_ID])):
            self.vehicle.simple_goto(self.GPS_WPS[self.currentWP_ID], 3)
            time.sleep(0.5)


    def deconstruct(self):
        self.points_file.close()
        self.buckets_file.close()
        self.vehicle.armed = False
        self.vehicle.close()

    def hasReached(self, targetLocation):
            difference = 0.00005
            if  abs(self.vehicle.location.global_frame.lat - targetLocation.lat) < difference and abs(self.vehicle.location.global_frame.lon - targetLocation.lon) < difference:
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
            angle = float(data[2]) + 90.0
            quality = int(data[3])

            if(quality < 10):
                return

            gps_from_ph = self.vehicle.location.global_frame
            robot_bearing = (-self.vehicle.heading + 450)%360 - self.FIELD_BEARING + 90
            pt = point(timestamp, gps_from_ph, [dist,angle,quality], robot_bearing)
            #beginning = int(round(time.time()))
            pt.calculateAbsPos(self.GPS_ORIGIN, self.FIELD_BEARING)
            
            #TEST 5/3##########
            #print"time for calculateAbsPos:{}".format(beginning - int(round(time.time())))
            #pt.printPt()
            ###################
            cell_id = self.map.addPoint(pt)

            if not(cell_id in self.current_scanned_cells):
                self.current_scanned_cells.append(cell_id)
            #[ts,gps.lat,gps.lon,dist,angle,quality,robot_bearing]
            #point_string = str(timestamp)+","+str(gps_from_ph.lat)+","+str(gps_from_ph.lon)+","+str(dist)+","+str(angle)+","+str(quality)+","+str(robot_bearing)+","+str(pt.abs_ft[0])+","+str(pt.abs_ft[1])+"\n"
            point_string = str(timestamp)+","+str(quality)+","+str(pt.abs_ft[0])+","+str(pt.abs_ft[1])+"\n"
            self.points_file.write(point_string)
            #print(string)
                
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
            pt_arr = []
            for cell_id in self.current_scanned_cells:
                for pt in self.map.cells[cell_id[0]][cell_id[1]]:
                    pt_arr.append(pt)
            if not(pt_arr is None or len(pt_arr) == 0):
                self.dbscan.reset(pt_arr)
                cs = self.dbscan.analyze() #need to check for overlapping clusters
                for centroid in cs:
                    centroid_string = str(centroid[0])+","+str(centroid[1])+"\n"
                    self.buckets_file.write(centroid_string)

            self.currentWP_ID += 1
            #if no more WPS, mission complete
            if(self.currentWP_ID == 2):#len(self.GPS_WPS)):
                self.MISSION_ENABLED == False
            else:
                self.vehicle.simple_goto(self.GPS_WPS[self.currentWP_ID], 5)

        #if robot is still travelling to the current waypoint
        else:
            #scan_lidar, simple_goto
            #[DO YOU HAVE TO CALL THIS EVERY LOOP?]
            #self.vehicle.simple_goto(self.GPS_WPS[self.currentWP_ID], 5)
            self.scan_lidar()

        return True

