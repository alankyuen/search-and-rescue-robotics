from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil  # Needed for command message definitions
import SARA

#instantiate SARA object
rover = SARA.SARA()

#initialize SARA
rover.init()
while(rover.MISSION_ENABLED):
    rover.run()
rover.deconstruct()

