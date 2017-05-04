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

rover = SARA.SARA()
rover.init()
while(rover.MISSION_ENABLED):
    rover.run()
rover.deconstruct()



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
