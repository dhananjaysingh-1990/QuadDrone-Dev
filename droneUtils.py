# Standard import
import time
import math
from time import sleep
import sys
# Library imports
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, LocationGlobal

from pymavlink import mavutil


# Helper function to connect to a vehicle on a given IP.
def set_home_loc(vehicle, home_loc=''):
    present_loc = LocationGlobal(18.589266, 73.863886, 10)
    vehicle.home_location = present_loc

    # If home_loc is null, then print/log the current home location.
    if not home_loc:
        # Get Vehicle Home location - will be `None` until first set by autopilot
        while not vehicle.home_location:
            cmds = vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not vehicle.home_location:
                print ("Waiting for home location ...")

    # We have a home location, so print it!
    print ("Home location: {}".format(vehicle.home_location))
    print("Vehicle global frmae: {}".format(vehicle.location.global_frame))
    print ("Location in Global frame: {}".format(vehicle.location.global_frame))
    print ("Location in global relative frame: {}".format(vehicle.location.global_relative_frame))
    print ("Location in Local frame: {}".format(vehicle.location.local_frame))


# Launch sequence
# Accepts altitude in meters to takeoff to.
def arm_takeoff(vehicle, tgt_alt):
    print("Poll on Vehicle.is_armable until the vehicle is ready to arm.")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise..")
        sleep(1)

    # Set the mode to GUIDED
    print("Set and wait until the Vehicle.mode is GUIDED.")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting to enter/confirm GUIDED mode..")
        sleep(1)

    # Arm the vehicle for takeoff.
    print("Set Vehicle.armed to True and poll until the vehicle is armed.")
    vehicle.armed = True
    # Check if the state is set to True/Armed
    while not vehicle.armed:
        print("Waiting for arm confirmation")
        sleep(1)

    print("Vehicle Ready for takeoff !")
    print("Simple_takeoff to achieve altitude of {} meters.".format(tgt_alt))
    vehicle.simple_takeoff(tgt_alt)
    while True:
        print("Climbing to Altitude: {}".format(vehicle.location.global_relative_frame.alt))
        # Break and return from function just below target altitude. Error margin of 5%.
        if vehicle.location.global_relative_frame.alt >= .95*tgt_alt:
            print("Reached target altitude")
            return
        time.sleep(1)

def land(vehicle):
    print("Preparing to LAND the vehicle")
    vehicle.mode = VehicleMode("LAND")
    # Wait till plane enters the LAND mode
    while vehicle.mode != "LAND":
        print("Waiting to enter 'LAND' flight Mode")
        time.sleep(1)
    # while not vehicle.location.global_relative_frame.alt==0:
    #	if vehicle.location.global_relative_frame.alt < 2:
    #    		set_velocity_body(vehicle,0,0,0.1)
    print("Vehicle landing Completed.")


def battery_check(vehicle):
    battery_perc = vehicle.battery.level
    print('Battery --> {}'.format(battery_perc))
    if(battery_perc < 9.9):
        print ("Caution: Battery Low --> LANDING")
        land(vehicle)
        sys.exit(0)
    else:
        print ("Battery: {}".format(battery_perc))
        return True


def send_local_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # vx, vy, vz velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    vehicle.send_mavlink(msg)
    time.sleep(1)
    vehicle.flush()


def send_global_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x,  # X velocity in NED frame in m/s
        velocity_y,  # Y velocity in NED frame in m/s
        velocity_z,  # Z velocity in NED frame in m/s
        0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    #for x in range(0,duration):
    vehicle.send_mavlink(msg)
    #time.sleep(1)
    vehicle.flush()


def velocity_flight(vehicle):
    cntr = 0
    # fly forward for 5 seconds
    while cntr < 5:
        send_global_velocity(vehicle, 5, 0, 0)
        time.sleep(1)
        cntr += 1
        print("Moving to North..")
    sleep(1)

    # fly south, in local referance
    cntr = 0
    while cntr < 5:
        send_global_velocity(vehicle, 0, -5, 0)
        time.sleep(1)
        cntr += 1
        print("Moving to West..")
    sleep(1)
    cntr = 0
    while cntr < 5:
        send_global_velocity(vehicle, 0, 5, 0)
        time.sleep(1)
        cntr += 1
        print("Moving to East..")
    sleep(1)

    cntr = 0
    # fly forward for 5 seconds
    while cntr < 5:
        send_global_velocity(vehicle, -5, 0, 0)
        time.sleep(1)
        cntr += 1
        print("Moving south..")
    sleep(1)

    while True:
        sleep(1)


def get_dist_meters(target_loc, destination_loc):
    """
    Returns the ground distance in metres between two Location objects.
     This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = target_loc.lat - destination_loc.lat
    dlon = target_loc.lon - destination_loc.lon
    # Calculate the hypotoneus to get distance inbetween
    return(math.sqrt((dlat * dlat) + (dlon * dlon))*1.13195e5)


def goto(vehicle, targetloc):
    """
    This method fly the drone to the specificied location
    :param targetloc: Endpoint destination location
    :return:
    """
    # Get the target distance in meters
    target_dist = get_dist_meters(targetloc, vehicle.location.global_relative_frame)
    print("Flying to destination {} at a distance of {}".format(targetloc, target_dist))
    vehicle.simple_goto(targetloc)
    # Wait till the drone completes the travel
    while vehicle.mode.name == "GUIDED":
        currentDist = get_dist_meters(targetloc, vehicle.location.global_relative_frame)
        print("Distance Covered : {} meters : {} and TARGET: {}".format(currentDist,
                                                                        vehicle.location.global_relative_frame,
                                                                        targetloc))
        # Accuracy of 1% within the target location
        if currentDist < target_dist * 0.01:
            print("Waypoint Reached, Current distance to target: {}m".format(currentDist))
            time.sleep(2)
            break
        # Check distance to destination, every second
        time.sleep(1)
    # Return once at target/destination
    return None