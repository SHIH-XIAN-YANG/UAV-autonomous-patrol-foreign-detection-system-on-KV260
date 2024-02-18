from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '/dev/ttyUSB0', help="Connect protocol")
parser.add_argument('--baud_rate', default = 57600, type=int,help="Set up baud rate")
args = parser.parse_args()
print('Connecting to vehicle on:',args.connect)
print('Baud rate:',args.baud_rate)

def Connect(Protocal, baud_rate):
    return connect(Protocal, baud_rate, wait_ready=False)

def Disconnect(vehical_ins):
    vehical_ins.close()

def arm_and_takeoff(vehicle, TargetAltitude):
    print ("Pre-arm checks")
    # while not vehicle.is_armable:
    #     print (" Waiting for vehicle to initialise...")
    #     time.sleep(1)

    print ("Arming motors")
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)
    print ("Taking off!")
    vehicle.simple_takeoff(TargetAltitude)


def land(vehicle):
    print("Landing")
    while vehicle.armed:
        time.sleep(1)

if __name__=='__main__':
    
    #vehicle = Connect(args.connect, args.baud_rate)
    vehicle = connect("/dev/ttyUSB0", wait_ready=True, baud = 57600)
    arm_and_takeoff(vehicle, 1) # 1m
    time.sleep(10)
    land(vehicle)
