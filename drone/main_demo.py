from dronekit import *
import mmap
import struct
import signal
import os
import sys
import time
import argparse 
import threading
import Drone


shm_path1 = "/dev/shm/drone_cam_shared_memory"
shm_path2 = "/dev/shm/drone_info_shared_memory"
data_lock = threading.Lock()
shm1 = None
shm2 = None
shm_railway_detect_flag = None
shm_foreign_object_flag = None
offset = None  
heading = None
P = 0.3
def signal_handler(sig, frame):
    global data_lock
    global shm1
    global shm2
    print("\nCtrl+C detected. Changing to LAND mode.")
    # data_lock.release()
    Drone.set_mode("LAND")
    Drone.Disconnect()
    shm1.close()
    shm2.close()
    sys.exit(0)


def write_drone_inf():
    global shm_path2
    global shm2
    # drone_info = f'{str(Drone.vehicle.location.global_frame)[15:]}|{str(Drone.vehicle.rangefinder.distance)[0:5]}|{Drone.vehicle.battery.voltage}|{Drone.vehicle.mode.name}|{str(Drone.vehicle.attitude)[10:]}'
    total_bytes = 200

    if not os.path.exists(shm_path2):
        shm_file = open(shm_path2, "wb")
        shm_file.truncate(total_bytes)
        shm_file.close()
    with open(shm_path2, "r+b") as shm_file:
        shm_file.truncate(total_bytes)
        shm_fd = shm_file.fileno()
        shm2 = mmap.mmap(shm_fd, total_bytes, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)
        while True:
            # print(drone_info)
            # drone_info = f'{Drone.vehicle.location.global_frame}|{Drone.vehicle.rangefinder.distance}|{Drone.vehicle.battery.voltage}|{Drone.vehicle.mode.name}|{Drone.vehicle.attitude}'
            drone_info = f'{str(Drone.vehicle.location.global_frame)[15:]}|{str(Drone.vehicle.rangefinder.distance)[0:5]}|{Drone.vehicle.battery.voltage}|{Drone.vehicle.mode.name}|{str(Drone.vehicle.attitude)[10:]}'
            data_bytes = drone_info.encode("utf-8")
            if len(data_bytes) <= total_bytes:
                shm2.seek(0)
                shm2.write(b'\0'*total_bytes)
                shm2.seek(0)
                shm2.write(data_bytes)
            else:
                print("Drone infomation is too long to fit in shared memory!")
            time.sleep(0.5)

def read_shm():
    global shm_path1
    global data_lock
    global shm1
    global shm_railway_detect_flag, shm_foreign_object_flag, offset, heading
    total_bytes = struct.calcsize('2i2f')
    while not os.path.exists(shm_path1):
        print(f'wait for rail tracking information...')
        time.sleep(1)

    if not os.path.exists(shm_path1):
        shm_file = open(shm_path1, "wb")
        shm_file.truncate(total_bytes)
        shm_file.close()
    with open(shm_path1, "r+b") as shm_file:
        shm_file.truncate(total_bytes)
        shm_fd = shm_file.fileno()  
        shm1 = mmap.mmap(shm_fd, total_bytes, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)
        while True:
            shm1.seek(0)
            data_bytes = shm1.read(total_bytes)
            data = struct.unpack('2i2f', data_bytes)
            data_lock.acquire()
            shm_railway_detect_flag, shm_foreign_object_flag, heading, offset = data
            # print(f'[read_shm]:{shm_railway_detect_flag},{shm_foreign_object_flag},{heading},{offset}')
            if shm_railway_detect_flag == 0:
                heading = None
                offset = None
            data_lock.release()
            time.sleep(0.001)

def drone_initial_setup(Protocal, baud = 57600):
    print('Connecting to pixhawk...')
    Drone.Connect(Protocal,baud)
    Drone.get_battery_info()
    # Drone.set_gimble_angle(-75, 0)
    

def search_railway(TargetAltitude = 1.5, speed = -0.2):
    global data_lock
    global heading
    global offset

    print("Start search railway !")
    Drone.vehicle.simple_takeoff(TargetAltitude)
    while True:
        print (" Relative altitude: %s m" %Drone.vehicle.rangefinder.distance)
        if float(Drone.vehicle.rangefinder.distance) >= TargetAltitude*0.97: 
            print ("Reached target altitude")
            break
        time.sleep(1)
    for _ in range(5):   
        Drone.control_only_pos_xyz(0,0,(float(Drone.vehicle.rangefinder.distance) - Drone.alt))
    for _ in range(30) :
        Drone.control_only_vel_xyz(0, speed ,0)
        data_lock.acquire()
        if not (heading is None and offset is None):
            Drone.control_only_vel_xyz(0, 0 ,0)
            data_lock.release()
            return
        data_lock.release()
    Drone.control_only_vel_xyz(0, 0 ,0)
    print("Drone can't search railway, turn-back and land right now !")
    Drone.set_mode("LAND")
    Drone.Disconnect()
    sys.exit(0)

def position_center():
    global data_lock
    global heading
    global offset
    railway_count = 0
    center_count = 0
    while(1):
        # print(Drone.vehicle.location.global_relative_frame.alt)
        data_lock.acquire()
        if heading is None and offset is None:
            Drone.control_roll_and_yaw(0,0)
            railway_count += 1
        else:
            railway_count = 0
            if abs(heading) < 3 and abs(offset) < 0.1:
                center_count += 1
            else:
                center_count = 0
            Drone.control_roll_and_yaw(P*offset, P*heading)
        data_lock.release()
        time.sleep(0.001)
        if center_count == 3:
            return
        if railway_count == 6:
            print("Drone can't position railway center about for 6 sec,turn-back and land right now !")
            Drone.set_mode("LAND")
            Drone.Disconnect()
            sys.exit(0)

def control_drone(speed = 0.5):
    global shm_foreign_object_flag
    global heading
    global offset
    if shm_foreign_object_flag == 1:
        speed = 0.2
        if abs(heading) >= 3 and abs(offset) >= 0.1:   
            Drone.control_forward_and_roll_and_yaw(P*offset, speed, P*heading)
        elif abs(heading) >= 3 and abs(offset) < 0.1:
            Drone.control_forward_and_yaw(speed, P*heading)
            # print(f"go straight and yaw {-1*heading}")
            # time.sleep(1)
        elif abs(heading) < 3 and abs(offset) >= 0.1:
            Drone.control_forward_and_roll(P*offset, speed)
            # print(f"go straight and roll {-1*offset}")
            # time.sleep(1)
        elif abs(heading) < 3 and abs(offset) < 0.1:
            Drone.control_only_vel_xyz(speed, 0, 0)
            # print("go straight")
            # time.sleep(1)   
    elif shm_foreign_object_flag == 0:
        if abs(heading) >= 3 and abs(offset) >= 0.1:   
            Drone.control_forward_and_roll_and_yaw(P*offset, speed, P*heading)
            # time.sleep(1)
            # print(f"go straight and roll {offset} and yaw {heading}")
        elif abs(heading) >= 3 and abs(offset) < 0.1:
            Drone.control_forward_and_yaw(speed, P*heading)
            # time.sleep(1)
            # print(f"go straight and yaw {heading}")
        elif abs(heading) < 3 and abs(offset) >= 0.1:
            Drone.control_forward_and_roll(P*offset, speed)
            # time.sleep(1)
            # print(f"go straight and roll {offset}")
        elif abs(heading) < 3 and abs(offset) < 0.1:
            Drone.control_only_vel_xyz(speed, 0, 0)
            # time.sleep(1)
            # print("go straight")

if __name__=='__main__':
    count = 0
    speed = 0.5
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default = '/dev/serial/by-id/usb-Holybro_Pixhawk6C_2C0047000A51313038393734-if02', help="Connect protocol")
    parser.add_argument('--baud_rate', default = 57600, type=int,help="Set up baud rate")
    args = parser.parse_args()
    
    

    # regist ctrl+c exit signal
    signal.signal(signal.SIGINT, signal_handler)

    # read rail tracking information from shm
    shm_thread1 = threading.Thread(target = read_shm)
    shm_thread1.daemon = True
    shm_thread1.start()
    # Connect to drone
    drone_initial_setup(args.connect, args.baud_rate)
    # write drone info to shm
    shm_thread2 = threading.Thread(target = write_drone_inf)
    shm_thread2.daemon = True
    shm_thread2.start()
    # while not vehicle.is_armable:
    #     print(" Waiting for vehicle to initialise...")
    #     time.sleep(1)
    Drone.set_mode("GUIDED")
    Drone.vehicle.armed = True
    while not Drone.vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)
    search_railway(Drone.alt, -0.2)
    time.sleep(0.1)
    position_center()
    print("Drone is already positioned and ready for rail tracking")
    Drone.control_only_vel_xyz(speed, 0, 0)
    while True:
        # speed = 0.5
        data_lock.acquire()
        if heading is None and offset is None:
            count += 1
            time.sleep(1)
            print(f"Drone can't detect railway about {count} sec, but go straight")
            Drone.control_only_vel_xyz(speed, 0, 0)
        else:
            count = 0
            control_drone(speed)
        data_lock.release()
        time.sleep(0.001)
        if count == 10:
            print("Drone can't detect the railway for 10 sec! turn-back and land right now !")
            Drone.set_mode("LAND")
            Drone.Disconnect()
            sys.exit(0)
