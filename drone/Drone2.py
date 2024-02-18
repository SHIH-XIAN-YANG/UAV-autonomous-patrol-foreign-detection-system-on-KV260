#from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit import *
from pymavlink import mavutil
import time
import math

vehicle = None
alt = 2

def Connect(Protocal, baud_rate):
    global vehicle
    if vehicle == None:
        vehicle = connect(Protocal, wait_ready=True, baud=baud_rate)
        print("Drone connect successful!")
    else:
        print("Drone has connected, Don't connect again !")

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
    print("Remaining battery: %s V" %vehicle.battery.voltage)

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
    # print(f"Drone rotate {heading} degrees in yaw, z: {float(vehicle.rangefinder.distance)-alt} (m)!")
    heading = heading * (math.pi/180)
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000100111000011,                             #-- BITMASK -> Only consider the position(0b0000110111111000)
            0, 0, (float(vehicle.rangefinder.distance)-alt),                                        #-- POSITION
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
    # print(f'Drone will move x: {position_x}(m), y: {position_y}(m), z: {position_z}(m)')

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
    # print(f'Drone velocity x: {velocity_x}(m/s), y: {velocity_y}(m/s), z: {velocity_z}(m/s), z: {float(vehicle.rangefinder.distance)-alt} (m)!')

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000000111000011,                             #-- BITMASK -> Only consider the velocity
            0, 0, (float(vehicle.rangefinder.distance) - alt),                                        #-- POSITION
            velocity_x, velocity_y, velocity_z,             #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    time.sleep(1)

def control_roll_and_yaw(velocity_y, heading):
    global vehicle
    # print(f'Drone roll({velocity_y} m/s) and rotate {heading} degrees in yaw, z: {float(vehicle.rangefinder.distance)-alt} (m)!')
    heading = heading * (math.pi/180)
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000000111000000,                             #-- BITMASK -> onsider the position_y and velocity_x
            0, 0, (float(vehicle.rangefinder.distance) - alt),                                        #-- POSITION
            0, velocity_y, 0,                               #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            heading, 0)  
    vehicle.send_mavlink(msg)
    time.sleep(1)

def control_forward_and_roll(position_y, velocity_x):
    global vehicle
    # print(f'Drone go forward({velocity_x} m/s) and move y:{position_y} m, z: {float(vehicle.rangefinder.distance)-alt} (m)!')

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000000111000000,                             #-- BITMASK -> onsider the position_y and velocity_x
            0, position_y, (float(vehicle.rangefinder.distance) - alt),                               #-- POSITION
            velocity_x, 0, 0,                               #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            0, 0)  
    vehicle.send_mavlink(msg)
    time.sleep(1)


def control_forward_and_yaw(velocity_x, heading):
    global vehicle
    # print(f'Drone go forward({velocity_x} m/s) and rotate {heading} degrees in yaw, z: {float(vehicle.rangefinder.distance)-alt} (m)!')
    heading = heading * (math.pi/180)    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000000111000000,                             #-- BITMASK -> onsider the position_y and velocity_x
            0, 0, (float(vehicle.rangefinder.distance) - alt),                                        #-- POSITION
            velocity_x, 0, 0,                               #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            heading, 0)  
    vehicle.send_mavlink(msg)
    time.sleep(1)

def control_forward_and_roll_and_yaw(position_y, velocity_x, heading):
    global vehicle
    # print(f'Drone go forward({velocity_x} m/s) and move y:{position_y} m and rotate {heading} degrees in yaw, z: {float(vehicle.rangefinder.distance)-alt} (m)!')
    heading = heading * (math.pi/180)
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000000111000000,                             #-- BITMASK -> onsider the position_y and velocity_x
            0, position_y, (float(vehicle.rangefinder.distance) - alt),                               #-- POSITION
            velocity_x, 0, 0,                               #-- VELOCITY
            0, 0, 0,                                        #-- ACCELERATIONS
            heading, 0)  
    vehicle.send_mavlink(msg)
    time.sleep(1)

if __name__ == '__main__':
    Connect('/dev/serial/by-id/usb-Holybro_Pixhawk6C_2C0047000A51313038393734-if02', 57600)
    get_battery_info()

