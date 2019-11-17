# Standard imports
import os
import socket
import argparse
import time
# Library imports
from dronekit import connect, APIException
import dronekit_sitl
import droneUtils


def connectVehicle():
    """
    Returns a connection to a vehicle on a given IP.
    Returns a sitl object.
    TBD: Take flags as simulated or real vehicle.
    """
    # Get command line inputs from the user.
    # This is needed in case of connecting to the real vehicle.
    # cmdopts = argparse.ArgumentParser(description="Parameters to connect plane")
    # cmdopts.add_argument("--connect")
    # args = cmdopts.parse_args()
    conn_str = '127.0.0.1:14551'
    sitl = ''
    # Connect sitl to default localhost.
    # This will also launch SITL and connect to it on 127.0.0.1:14551
    if not conn_str:
        sitl = dronekit_sitl.start_default()
        conn_str = sitl.connection_string()

    # Now, we have connection IP, Connect to the Vehicle.
    print("Connecting to vehicle on: {}".format(conn_str))
    try:
        vehicle = connect(conn_str, wait_ready=True)
    # Bad TCP connection
    except socket.error:
        print("No server exists!")
    # Bad TTY connection
    except OSError as e:
        print("No serial exists! : {} ".format(e))
    # API Error
    except APIException as ae:
        print("Timeout!")
    except:
        print("Some other error!")
    else:
        print("Connected successfully.")
        return vehicle, sitl


def fly_drone_height(height):
    vehicle, sitl = connectVehicle()
    droneUtils.arm_takeoff(vehicle, height)
    while droneUtils.battery_check(vehicle):
        print('Checking Battery Status')
        time.sleep(2)


def fly_drone(height, x, y, z):
    vehicle, sitl = connectVehicle()
    droneUtils.arm_takeoff(vehicle, height)
    #while droneUtils.battery_check(vehicle):
    #    print('Checking Battery Status')
    #    time.sleep(2)
    droneUtils.velocity_flight(vehicle)


#fly_drone_height(10)
fly_drone(10, 500, 0, 0)
