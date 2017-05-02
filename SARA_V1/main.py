#main.py


#!/usr/bin/env python
# -*- coding: utf-8 -*-
# alanyuen95@gmail.com
"""
Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html

execute command at propt in sudo mode:: python vehicle_state_mod2_play.py
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil  # Needed for command message definitions
import time
import SARA

def robotIsAtLocation(targetLocation):
    difference = 0.0001
    if 	abs(vehicle.location.global_frame.lat - targetLocation.lat) < difference and
    	abs(vehicle.location.global_frame.lon - targetLocation.lon) < difference:
        return True
    else:
        return False

def getGPS():
	return [vehicle.location.global_frame.lat,vehicle.location.global_frame.lon]

# connection_string = "/dev/cu.usbserial-FTZ1626T, 921600"
# connection_string = "/dev/ttyUSB0,921600"
# connection_string = "tcp:127.0.0.1:5760"
connection_string = "/dev/serial/by-id/usb-FTDI_C232HD-DDHSP-0_FTZ1626T-if00-port0,921600"

# Connect to the Vehicle.
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print
"\nConnecting to vehicle on: %s" % connection_string
vehicle = connect(connection_string)

"""
# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
#print " Autopilot Firmware version: %s" % vehicle.version
#print "   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration
print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print " Local Location: %s" % vehicle.location.local_frame
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
print " GPS: %s" % vehicle.gps_0
print " Gimbal status: %s" % vehicle.gimbal
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Heading: %s" % vehicle.heading
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Groundspeed: %s" % vehicle.groundspeed    # settable
#print " Airspeed: %s" % vehicle.airspeed    # settable
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable
"""
vehicle.armed = True
# sleep(1)
# set home location
vehicle.home_location = vehicle.location.global_frame

vehicle.groundspeed = 5


"""
while not robotIsAtLocation(park1):
    vehicle.simple_goto(park1, 5)
    time.sleep(0.5)

while not robotIsAtLocation(park2):
    vehicle.simple_goto(park2, 5)
    time.sleep(0.5)

while not robotIsAtLocation(park3):
    vehicle.simple_goto(park3, 5)
    time.sleep(0.5)

while not robotIsAtLocation(park4):
    vehicle.simple_goto(park4, 5)
    time.sleep(0.5)

while not robotIsAtLocation(park5):
    vehicle.simple_goto(park5, 5)
    time.sleep(0.5)
"""
vehicle.armed = False
print
" Now Vehicle is not armed"

# Close vehicle object before exiting script
print
"\nClose vehicle object"
vehicle.close()

print("Completed")
