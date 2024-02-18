from dronekit import *
import signal
import sys
import time
import argparse 
import multiprocessing
import Drone
from Drone import vehicle


offset = None  
heading = None
def signal_handler(sig, frame):
    global camera_process 
    global vehicle
    print("Ctrl+C detected. Changing to LAND mode.")
    if vehicle.mode.name == "GUIDED":
        vehicle.mode = VehicleMode("LAND")
    camera_process.terminate()
    sys.exit(0)

def setup(Protocal, baud = 115200):
    global vehicle
    Drone.Connect(Protocal,baud)
    Drone.get_battery_info()
    Drone.set_gimble_angle(-75, 0)
    

def search_railway(TargetAltitude = 1.6):
    global vehicle
    print("Start search railway !")
    vehicle.simple_takeoff(TargetAltitude)
    while True:
        print (" Relative altitude: %s" %vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= TargetAltitude*0.95: 
            print ("Reached target altitude")
            break
        time.sleep(1)
    time.sleep(2)
    Drone.control_vel_xyz(0, 0.1 ,0)
    for _ in range(15) :
        if offset >= 0:
            Drone.control_vel_xyz(0, 0 ,0)
            Drone.control_yaw(heading)
            Drone.control_pos_xyz(0, -offset, 0)
            return 
        time.sleep(1)
    Drone.control_vel_xyz(0, 0 ,0)
    print("Drone can't search railway, land right now !")
    Drone.land()


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default = '/dev/ttyUSB0', help="Connect protocol")
    parser.add_argument('--baud_rate', default = 57600, type=int,help="Set up baud rate")
    args = parser.parse_args()
    signal.signal(signal.SIGINT, signal_handler)

    #Create share memory
    share_memory = multiprocessing.Array('d',[None,None])
    lock = multiprocessing.Lock()

    setup(args.connect, args.baud_rate)
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    Drone.set_mode("GUIDED")
    vehicle.armed   = True
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)
    #**********************NEED****************************************
    camera_process = multiprocessing.Process(target=,args=(share_memory, lock))
    camera_process.start
    time.sleep(3)
    search_railway(1.6)
    time.sleep(3)
    print("Drone is already positioned and ready for rail tracking")

    Drone.set_velocity(0.5)
    while True:
        with lock:
            heading = share_memory[0]
            offset = share_memory[1]
            print(f'[Sharememory] heading: {heading}, offset: {offset}')
        if heading is None and offset is None:
            time.sleep(1)
            continue
        else:
            if abs(heading) > 5:
                Drone.control_yaw(heading)
            #***********************************未來要加上PID*********************
            if offset > 0.15:
                Drone.control_pos_xyz(0, -offset, 0)
            time.sleep(1)
