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

#instantiate SARA object
rover = SARA.SARA()

#initialize SARA
rover.init()
#[[33.716175, -117.830442] ,[33.716041, -117.830221] , [33.716175, -117.830442,33],[33.716034, -117.830611]]
#33.716041, -117.830221
#33.716034, -117.830611
#time.sleep(10)
while(rover.MISSION_ENABLED):
    rover.run()


rover.deconstruct()


"""
#1 towards first car of staff
Home Location Fixed: LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25
vehicle.home_location vs GPS_ORIGIN: LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25 vs LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25
field_bearing: 187.0


#2 towards sidewalk next to staff car
Home Location Fixed: LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25
vehicle.home_location vs GPS_ORIGIN: LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25 vs LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25
field_bearing: 65.0

#3 towards (atep) corner speed limit sign
Home Location Fixed: LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25
vehicle.home_location vs GPS_ORIGIN: LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25 vs LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25
field_bearing: 77.0


#4
>>> Reached destination
Home Location Fixed: LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25
vehicle.home_location vs GPS_ORIGIN: LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25 vs LocationGlobal:lat=33.7166519165,lon=-117.831016541,alt=21.25
field_bearing: 79.0


****************************

#1
"""
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

vehicle.armed = True
# sleep(1)
# set home location
vehicle.home_location = vehicle.location.global_frame

vehicle.groundspeed = 5



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
