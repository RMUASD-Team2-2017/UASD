################################################################################################
# @File DroneKitPX4.py
# Example usage of DroneKit with PX4
#
# @author Sander Smeets <sander@droneslab.com>
#
# Code partly based on DroneKit (c) Copyright 2015-2016, 3D Robotics.
################################################################################################

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math


################################################################################################
# Settings
################################################################################################

# Connect to FCU through serial
fcu_connection_string       = '127.0.0.1:14540' # /dev/ttyAMA0 (also set baud=57600)

# Sniff mavlink/to and /from through serial
from_connection_string	= /dev/ttyAMA1 # Tx
to_connection_string	= /dev/ttyAMA2 # Rx

# gcs_connection_string		= '127.0.0.1:11311'

# MAV_MODE
MAV_MODE_PREFLIGHT = 0 # System is not ready to fly, booting, calibrating, etc. No flag is set.
MAV_MODE_MANUAL_DISARMED = 64 # System is allowed to be active, under manual (RC) control, no
                        # stabilization
MAV_MODE_TEST_DISARMED = 66 # UNDEFINED mode. This solely depends on the autopilot - use with
                        # caution, intended for developers only.
MAV_MODE_STABILIZE_DISARMED = 80 # System is allowed to be active, under assisted RC control.
MAV_MODE_GUIDED_DISARMED = 88 # System is allowed to be active, under autonomous control, manual
                        # setpoint
MAV_MODE_AUTO_DISARMED = 92 # System is allowed to be active, under autonomous control and
                        # navigation (the trajectory is decided
                        # onboard and not pre-programmed by MISSIONs)
MAV_MODE_MANUAL_ARMED = 192 # System is allowed to be active, under manual (RC) control, no
                        # stabilization
MAV_MODE_TEST_ARMED = 194 # UNDEFINED mode. This solely depends on the autopilot - use with
                        # caution, intended for developers only.
MAV_MODE_STABILIZE_ARMED = 208 # System is allowed to be active, under assisted RC control.
MAV_MODE_GUIDED_ARMED = 216 # System is allowed to be active, under autonomous control, manual
                        # setpoint
MAV_MODE_AUTO_ARMED = 220 # System is allowed to be active, under autonomous control and
                        # navigation (the trajectory is decided
                        # onboard and not pre-programmed by MISSIONs)
MAV_MODE_ENUM_END = 221 # 
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py


# Parse connection argument
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# Init
################################################################################################

# Connect to the Vehicle
print "Connecting"
vehicle = connect(fcu_connection_string, wait_ready=True, source_system=1)
mavlink_to = connect(to_connection_string, baud=57600)
mavlink_from = connect(from_connection_string, baud=57600)

def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)



def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)



################################################################################################
# Listeners
################################################################################################

home_position_set = False

#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True

@vehicle.on_message('last_heartbeat')
def listener(self, name, last_heartbeat):
	global lost_link
	if last_heartbeat > 10:
		print "Heartbeat connection lost."
		lost_link = True
	else:
		lost_link = False
	
@mavlink_to.on_message('*')
def listener(self, name, msg):
	# Listen to all mavlink/to messages
	print "On mavlink/to, intercepted message %s" %msg
	
@mavlink_from.on_message('*')
def listener(self, name, msg):
	# Listen to all mavlink/from messages
	print "On mavlink/from, intercepted message %s" %msg

################################################################################################
# Start mission example
################################################################################################

# wait for a home position lock
while not home_position_set:
    print "Waiting for home position..."
    time.sleep(1)

# Display basic vehicle state
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt

# Change to AUTO mode
PX4setMode(MAV_MODE_AUTO_ARMED)
time.sleep(1)

# Load commands
cmds = vehicle.commands
cmds.clear()

home = vehicle.location.global_relative_frame

# takeoff to 10 meters
wp = get_location_offset_meters(home, 0, 0, 10);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# land
wp = get_location_offset_meters(home, 0, 0, 10);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# Upload mission
cmds.upload()
time.sleep(2)

# Arm vehicle
vehicle.armed = True

# monitor mission execution
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
	if time.time() - last_heartbeat > 10: # This is a stupid thing for the on-board computer to monitor
		print "Heartbeat lost."
		PX4setMode(MAV_MODE_STABILIZE_ARMED)
	if vehicle.commands.next > nextwaypoint:
		display_seq = vehicle.commands.next+1
		print "Moving to waypoint %s" % display_seq
		nextwaypoint = vehicle.commands.next
	time.sleep(1)

# wait for the vehicle to land
while vehicle.commands.next > 0:
	time.sleep(1)

# Disarm vehicle
vehicle.armed = False
time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()
mavlink_to.close()
mavlink_from.close()
time.sleep(1)
