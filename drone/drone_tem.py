#from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit import *
from pymavlink import mavutil
import time
import math
import mmap
import struct
import signal
import os
import sys
import time
import argparse 
import threading

vehicle = None

def Connect(Protocal, baud_rate):
    global vehicle
    if vehicle == None:
        vehicle = connect(Protocal, wait_ready=True, baud=baud_rate)
        print("Drone connect successful!")
    else:
        print("Drone has connected, Don't connect again !")
    # return vehicle
    # print(vehicle)

def Disconnect():
    global vehicle
    vehicle.close()

def set_mode(mode):
    global vehicle
    vehicle.mode = VehicleMode(mode)

# Get globel longitude,latitude,altitude(from sea-level)
def get_location():
    global vehicle
    print("Global Location: %s" %vehicle.location.global_frame)

def get_relative_altitude():
    global vehicle
    print("Relative altitude: %s" %vehicle.rangefinder.distance)

def get_relative_altitude_GPS():
    global vehicle
    print("Relative altitude: %s" %vehicle.location.global_relative_frame.alt)
# Get relative altitude between vehicle and obstacle from rangefinder
def get_relative_altitude_bet_obstacle():
    global vehicle
    print("Relative altitude: %s" %vehicle.rangefinder.distance)

def get_velocity():
    global vehicle
    print("Groundspeed: %s (m/s)" %vehicle.groundspeed)

def set_velocity(speed):
    global vehicle
    vehicle.groundspeed = speed
    print("Groundspeed: %s (m/s)" %vehicle.groundspeed)    

def get_battery_info():
    global vehicle
    print("Remaining battery: %s" %vehicle.battery.level)

def get_gimbal_angle():
    global vehicle
    print("Gimble angle(roll, pitch, yall): %s" %vehicle.gimbal)

'''
Our gimble is 2D:
Set pitch: pointed straight ahead relative to the front of the vehicle, while -90 points the camera straight down.
Set roll:  roll in degrees relative to the vehicle
'''
def set_gimble_angle(pitch_angle, roll_angle = 0):
    global vehicle
    vehicle.gimbal.rotate(pitch_angle,roll_angle)
    print("Gimble angle(roll, pitch, yall): %s" %vehicle.gimbal)

def disarm():
    global vehicle
    vehicle.armed = False

def Manual_arm():
    global vehicle
    print("Enter the manual mode !")
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)
    vehicle.mode    = VehicleMode("STABILIZE")
    vehicle.armed   = True
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)
    print ("Arming motors")

def arm_and_takeoff_test(TargetAltitude):
    global vehicle
    print("Arm and takeoff test start !")
    #print ("Pre-arm checks")
    # while not vehicle.is_armable:
    #     print (" Waiting for vehicle to initialise...")
    #     time.sleep(1)
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)
    print ("Taking off !")
    vehicle.simple_takeoff(TargetAltitude)
    while True:
        print (" Relative altitude: %s" %vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= TargetAltitude*0.95: 
            print ("Reached target altitude")
            break
        time.sleep(1)

def land():
    global vehicle
    print("Enter LAND mode !")
    vehicle.mode = VehicleMode("LAND")

def return_to_launch_location():
    print("After 2s,Drone will enter return mode! Be carefull, It wont detect obstacles")
    time.sleep(2)
    vehicle.mode = VehicleMode("RTL")

def control_only_yaw(heading):
    global vehicle
    heading = heading * (math.pi/180)
    # if heading > 0:
    #     direction_flag = "cw"
    #     direction = 1   
    # else:
    #     direction_flag = "ccw"
    #     direction = -1

    # msg = vehicle.message_factory.command_long_encode(
    #     0, 0,       
    #     mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
    #     0,          
    #     heading,    
    #     0,      #speed deg/s
    #     direction,  
    #     1,          #relative offset 1
    #     0, 0, 0)    
    # vehicle.send_mavlink(msg)
    print("Drone rotate %d degree in yaw !" %heading)
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000100111000111,                             #-- BITMASK -> Only consider the position(0b0000110111111000)
            0, 0, 0,                                        #-- POSITION
            0, 0, 0,                                        #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            heading, 0)
    vehicle.send_mavlink(msg)
    time.sleep(1)

'''
[parameter 5]:Bitmask indicate which fields should be ignored by the vehicle
              bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate
[parameter 7]: velocity_x(m/s)(positive is forward or North)
[parameter 8]: velocity_y(m/s)(positive is right or East)
[parameter 9]: velocity_z(m/s)(positive is down)
'''
def control_only_pos_xyz(position_x, position_y, position_z):
    global vehicle
    print(f'Drone will move x: {position_x}(m), y: {position_y}(m), z: {position_z}(m)')

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000000111111000,                             #-- BITMASK -> Only consider the position
            position_x, position_y, position_z,             #-- POSITION
            0, 0, 0,                                        #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    time.sleep(1)

'''
From Copter 3.3 the vehicle will stop moving if a new message is not received in approximately 3 seconds.
'''
def control_only_vel_xyz(velocity_x, velocity_y, velocity_z):
    global vehicle
    print(f'Drone velocity x: {velocity_x}(m/s), y: {velocity_y}(m/s), z: {velocity_z}(m/s)')

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000000111000011,                             #-- BITMASK -> Only consider the velocity
            0, 0, 0,                                        #-- POSITION
            velocity_x, velocity_y, velocity_z,             #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    time.sleep(1)

def control_forward_and_roll(position_y, velocity_x):
    global vehicle
    print(f'Drone go forward({velocity_x} m/s) and move y:{position_y} m')

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000000111000000,                             #-- BITMASK -> onsider the position_y and velocity_x
            0, position_y, 0,                               #-- POSITION
            velocity_x, 0, 0,                               #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            0, 0)  
    vehicle.send_mavlink(msg)
    time.sleep(1)

def control_forward_and_yaw(velocity_x, heading):
    global vehicle
    heading = heading * (math.pi/180)
    print(f'Drone go forward({velocity_x} m/s) and rotate {heading} degrees in yaw')
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000000111000000,                             #-- BITMASK -> onsider the position_y and velocity_x
            0, 0, 0,                                        #-- POSITION
            velocity_x, 0, 0,                               #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            heading, 0)  
    vehicle.send_mavlink(msg)
    time.sleep(1)
####################################################################################################################
# def demo1():
#     global vehicle
#     arm_and_takeoff_test(1.2)
#     time.sleep(3)
#     get_relative_altitude_bet_obstacle()
#     for _ in range(20):
#         control_only_vel_xyz(0.3, 0, 0)  
#         time.sleep(1)
#     # control_pos_xyz(0,-0.5, 0)
#     time.sleep(3)
#     land()
#     Disconnect()

# def demo2():
#     global vehicle
#     arm_and_takeoff_test(1.6)
#     time.sleep(3)
#     get_relative_altitude_bet_obstacle()
#     for a in range(2):
#         control_vel_xyz(0,-1,0)
#         time.sleep(2)
#     for a in range(2):
#         control_vel_xyz(0,1,0)
#         time.sleep(2)    
#     time.sleep(3)
#     land()
#     Disconnect()

# def demo3():
#     global vehicle
#     arm_and_takeoff_test(1.6)
#     time.sleep(2)
#     get_relative_altitude_bet_obstacle()
#     control_pos_xyz(1, 0, 0)
#     time.sleep(2)
#     msg = vehicle.message_factory.set_position_target_local_ned_encode(
#             0,
#             0, 0,
#             9,
#             0b0000000111000111, #-- BITMASK -> Only consider the position(0b0000110111111000)
#             0, 0, 0,             #-- POSITION
#             0, 0, 0,                                        #-- VELOCITY
#             0, 0, 0,                                        #-- ACCELERATIONS
#             1.5704, 1.5704)
#     vehicle.send_mavlink(msg)
#     time.sleep(2)
#     control_pos_xyz(1, 0, 0)
#     time.sleep(2)
#     msg = vehicle.message_factory.set_position_target_local_ned_encode(
#             0,
#             0, 0,
#             9,
#             0b0000000111000111, #-- BITMASK -> Only consider the position(0b0000110111111000)
#             0, 0, 0,             #-- POSITION
#             0, 0, 0,                                        #-- VELOCITY
#             0, 0, 0,                                        #-- ACCELERATIONS
#             -3.13,3.13)
#     vehicle.send_mavlink(msg)
#     time.sleep(3)
#     control_pos_xyz(1, 0, 0)
#     time.sleep(2)
#     msg = vehicle.message_factory.set_position_target_local_ned_encode(
#             0,
#             0, 0,
#             9,
#             0b0000000111000111, #-- BITMASK -> Only consider the position(0b0000110111111000)
#             0, 0, 0,             #-- POSITION
#             0, 0, 0,                                        #-- VELOCITY
#             0, 0, 0,                                        #-- ACCELERATIONS
#             1.5704, 1.5704)    
#     vehicle.send_mavlink(msg)
#     time.sleep(2)
#     control_pos_xyz(-1, 0, 0)
#     time.sleep(2)
#     land()
#     Disconnect()

# def demo4():
#     global vehicle
#     arm_and_takeoff_test(1.6)
#     time.sleep(3)
#     get_relative_altitude_bet_obstacle()
#     control_pos_xyz(3,0,0)
#     time.sleep(3)
#     set_mode("RTL")
#     Disconnect()

# def demo5():
#     global vehicle
#     arm_and_takeoff_test(1.6)
#     time.sleep(3)
#     get_relative_altitude_bet_obstacle()
#     msg = vehicle.message_factory.set_position_target_local_ned_encode(
#             0,
#             0, 0,
#             9,
#             0b0000100111000111, #-- BITMASK -> Only consider the position(0b0000110111111000)
#             0, 0, 0,             #-- POSITION
#             0, 0, 0,                                        #-- VELOCITY
#             0, 0, 0,                                        #-- ACCELERATIONS
#             -1.5704, 0)
#     vehicle.send_mavlink(msg)
#     time.sleep(3)
#     land()
#     Disconnect() 
# def demo6():
#     global vehicle
#     arm_and_takeoff_test(0.8)
#     time.sleep(3)
#     get_relative_altitude_bet_obstacle()
#     for _ in range(4):
#         msg = vehicle.message_factory.set_position_target_local_ned_encode(
#                 0,
#                 0, 0,
#                 9,
#                 0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
#                 0, 0, 0,                                        #-- POSITION
#                 1, 0, 0,                                        #-- VELOCITY
#                 0, 0, 0,                                        #-- ACCELERATIONS
#                 0, 0)  
#         vehicle.send_mavlink(msg)
#         time.sleep(2)
#     for _ in range(4):
#         msg = vehicle.message_factory.set_position_target_local_ned_encode(
#                 0,
#                 0, 0,
#                 9,
#                 0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
#                 0, 0, 0,                                        #-- POSITION
#                 -1, 0, 0,                                        #-- VELOCITY
#                 0, 0, 0,                                        #-- ACCELERATIONS
#                 0, 0)  
#         vehicle.send_mavlink(msg)  
#         time.sleep(2)
#     land()
#     Disconnect() 
    
# def demo7():
#     global vehicle
#     arm_and_takeoff_test(1.6)
#     time.sleep(3)
#     get_relative_altitude_bet_obstacle()
#     for _ in range(3):
#         msg = vehicle.message_factory.set_position_target_local_ned_encode(
#                 0,
#                 0, 0,
#                 9,
#                 0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
#                 0, 0.3, 0,                                        #-- POSITION
#                 0.7, 0, 0,                                        #-- VELOCITY
#                 0, 0, 0,                                        #-- ACCELERATIONS
#                 0, 0)    
#         vehicle.send_mavlink(msg)
#         time.sleep(2)
#     for _ in range(3):
#         msg = vehicle.message_factory.set_position_target_local_ned_encode(
#                 0,
#                 0, 0,
#                 9,
#                 0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
#                 0, -0.3, 0,                                        #-- POSITION
#                 0.7, 0, 0,                                        #-- VELOCITY
#                 0, 0, 0,                                        #-- ACCELERATIONS
#                 0, 0)    
#         vehicle.send_mavlink(msg)
#         time.sleep(2)
#     for _ in range(3):
#         msg = vehicle.message_factory.set_position_target_local_ned_encode(
#                 0,
#                 0, 0,
#                 9,
#                 0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
#                 0, 0.3, 0,                                        #-- POSITION
#                 -0.7, 0, 0,                                        #-- VELOCITY
#                 0, 0, 0,                                        #-- ACCELERATIONS
#                 0, 0)    
#         vehicle.send_mavlink(msg)
#         time.sleep(2)
#     for _ in range(3):
#         msg = vehicle.message_factory.set_position_target_local_ned_encode(
#                 0,
#                 0, 0,
#                 9,
#                 0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
#                 0, -0.3, 0,                                        #-- POSITION
#                 -0.7, 0, 0,                                        #-- VELOCITY
#                 0, 0, 0,                                        #-- ACCELERATIONS
#                 0, 0)    
#         vehicle.send_mavlink(msg)
#         time.sleep(2)
    # for _ in range(4):
    #     msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #             0,
    #             0, 0,
    #             9,
    #             0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
    #             0, 0, 0,                                        #-- POSITION
    #             -1, 0, 0,                                        #-- VELOCITY
    #             0, 0, 0,                                        #-- ACCELERATIONS
    #             0, 0)    
    #     vehicle.send_mavlink(msg)
    #     time.sleep(2)
    # for _ in range(2):
    #     msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #             0,
    #             0, 0,
    #             9,
    #             0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
    #             0, 0, 0,                                        #-- POSITION
    #             1, 0, 0,                                        #-- VELOCITY
    #             0, 0, 0,                                        #-- ACCELERATIONS
    # #             0.7854, 0)    
    # #     vehicle.send_mavlink(msg)
    #     time.sleep(2)
    # for _ in range(2):
    #     msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #             0,
    #             0, 0,
    #             9,
    #             0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
    #             0, 0, 0,                                        #-- POSITION
    #             1, 0, 0,                                        #-- VELOCITY
    #             0, 0, 0,                                        #-- ACCELERATIONS
    #             -0.7854, 0)    
    #     vehicle.send_mavlink(msg)
    #     time.sleep(2)
    # for _ in range(4):
    #     msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #             0,
    #             0, 0,
    #             9,
    #             0b0000000111000000, #-- BITMASK -> Only consider the position(0b0000110111111000)
    #             0, 0, 0,                                        #-- POSITION
    #             -1, 0, 0,                                        #-- VELOCITY
    #             0, 0, 0,                                        #-- ACCELERATIONS
    #             0, 0)    
    #     vehicle.send_mavlink(msg)
    #     time.sleep(2)
    # land()
    # Disconnect() 

# if __name__ == '__main__':
#     Connect('/dev/serial/by-id/usb-Holybro_Pixhawk6C_2C0047000A51313038393734-if02', 57600)
#     get_battery_info()
#     demo1()
shm_path = "/dev/shm/drone_cam_shared_memory"
# data_lock = threading.Lock()
shm = None
shm_railway_detect_flag = None
shm_foreign_object_flag = None
offset = None  
heading = None

def signal_handler(sig, frame):
    global vehicle
    global shm
    print("Ctrl+C detected. Changing to LAND mode.")
    # if vehicle.mode.name == "GUIDED":
    vehicle.mode = VehicleMode("LAND")
    Disconnect()
    shm.close()
    sys.exit(0)

def read_shm():
    global shm_railway_detect_flag, shm_foreign_object_flag, offset, heading
    if not os.path.exists(shm_path):
        shm_file = open(shm_path, "wb")
        total_bytes = struct.calcsize('2i2f')
        shm_file.truncate(total_bytes)
        shm_file.close()
    with open(shm_path, "r+b") as shm_file:
        shm_fd = shm_file.fileno()  # 获取文件描述符
        total_bytes = struct.calcsize('2i2f')
        shm = mmap.mmap(shm_fd, total_bytes, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)
        while(1):
            shm.seek(0)
            data_bytes = shm.read(total_bytes)
            data = struct.unpack('2i2f', data_bytes)
            # data_lock.acquire()
            shm_railway_detect_flag, shm_foreign_object_flag, heading, offset = data
            # print(f"{shm_railway_detect_flag},{shm_foreign_object_flag},{heading},{offset}")
            if shm_railway_detect_flag == 0:
                heading = None
                offset = None
            # data_lock.release()
            time.sleep(1)

def drone_initial_setup(Protocal, baud = 57600):
    global vehicle
    Connect(Protocal,baud)
    get_battery_info()
    # Drone.set_gimble_angle(-75, 0)
    

def search_railway(TargetAltitude = 1.2):
    global vehicle
    print("Start search railway !")
    vehicle.simple_takeoff(TargetAltitude)
    while True:
        print (" Relative altitude: %s" %vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= TargetAltitude*0.97: 
            print ("Reached target altitude")
            break
        time.sleep(1)
    time.sleep(2)   
    for _ in range(10) :
        control_only_vel_xyz(0, -0.3 ,0)
        if not (heading is None and offset is None):
            control_only_vel_xyz(0, 0 ,0)
            control_only_yaw(-1*heading)
            control_only_pos_xyz(0, -1*offset, 0)
            print("ss")
            return 
    control_only_vel_xyz(0, 0 ,0)
    print("Drone can't search railway, turn-back and land right now !")
    set_mode("LAND")
    Disconnect()
    sys.exit(0)


if __name__=='__main__':
    count = 0
    speed = 0.3
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default = '/dev/serial/by-id/usb-Holybro_Pixhawk6C_2C0047000A51313038393734-if02', help="Connect protocol")
    parser.add_argument('--baud_rate', default = 57600, type=int,help="Set up baud rate")
    args = parser.parse_args()
    signal.signal(signal.SIGINT, signal_handler)
    shm_thread = threading.Thread(target = read_shm)
    shm_thread.start()
    # drone_initial_setup(args.connect, args.baud_rate)
    Connect(args.connect,args.baud_rate)
    # while not vehicle.is_armable:
    #     print(" Waiting for vehicle to initialise...")
    #     time.sleep(1)
    print(vehicle)
    set_mode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)
    search_railway(1.5)
    print("Drone is already positioned and ready for rail tracking")
    time.sleep(3)
    for _ in range(20):
        control_only_vel_xyz(speed, 0, 0)
    time.sleep(3)
    land()
    Disconnect()
    
#     while True:
#         print(f"{shm_railway_detect_flag},{shm_foreign_object_flag},{heading},{offset}")
#         speed = 0.1
#         if count == 10:
#             print("Drone can't detect the railway! turn-back and land right now !")
#             set_mode("RTL")
#             Disconnect()
#             sys.exit(0)
#         # data_lock.acquire()
#         if heading is None and offset is None:
#             count += 1
#             print(f"Drone can't detect railway for {count} sec, but go straight")
#             control_only_vel_xyz(speed, 0, 0)
#             time.sleep(1)
#         else:
#             count = 0
#             if shm_foreign_object_flag == 1:
#                 speed = 0.05
#                 if abs(heading) >= 0.01:
#                     time.sleep(1)
#                     control_forward_and_yaw(speed, -heading)
#                 if abs(offset) >= 0.01:
#                     time.sleep(1)
#                     control_forward_and_roll(-offset, speed)
#                 if abs(heading) < 0.01 and abs(offset) < 0.01:
#                     control_only_vel_xyz(speed, 0, 0)
#                     time.sleep(1)
#             elif shm_foreign_object_flag == 0:
#                 if abs(heading) >= 3 and abs(offset) >= 0.1:   
#                     control_forward_and_yaw(speed, -heading)
#                     # print(f"go straight and yaw {-heading}")
#                     control_forward_and_roll(-offset, speed)
#                     # print(f"go straight and roll {-offset}")
                    
#                 elif abs(heading) >= 3 and abs(offset) < 0.1:
#                     control_forward_and_yaw(speed, -heading)
#                     # print(f"go straight and yaw {-heading}")
#                     time.sleep(1)
#                 elif abs(heading) < 3 and abs(offset) >= 0.1:
#                     control_forward_and_roll(-offset, speed)
#                     # print(f"go straight and roll {-offset}")
#                     time.sleep(1)
#                 elif abs(heading) < 3 and abs(offset) < 0.1:
#                     control_only_vel_xyz(speed, 0, 0)
#                     time.sleep(1)     
#                     # print("go straight")
#         # data_lock.release()
