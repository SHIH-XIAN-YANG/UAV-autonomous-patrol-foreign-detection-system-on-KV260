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

def signal_handler(sig, frame):
    global shm1
    global shm2
    print("\nCtrl+C detected. Changing to LAND mode.")
    shm1.close()
    shm2.close()
    sys.exit(0)

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
        shm_fd = shm_file.fileno()  # 获取文件描述符
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
        print('start write drone information')
        while True:
            try:
                drone_info = f'{str(Drone.vehicle.location.global_frame)[15:]}|{str(Drone.vehicle.rangefinder.distance)[0:5]}|{Drone.vehicle.battery.voltage}|{Drone.vehicle.mode.name}|{str(Drone.vehicle.attitude)[9:]}'
                data_bytes = drone_info.encode("utf-8")
                # print(len(data_bytes))
                if len(data_bytes) <= total_bytes:
                    shm2.seek(0)
                    shm2.write(b'\0'*total_bytes)
                    shm2.seek(0)
                    shm2.write(data_bytes)
                else:
                    print("Drone infomation is too long to fit in shared memory!")
                # print(drone_info)
                time.sleep(1)
            except KeyboardInterrupt:
                shm2.close()
                shm_fd.close()
                break


if __name__ == '__main__':
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

    shm_thread1.join()
    shm_thread2.join()

    print('end of drone info write')


# vehicle = connect('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0',wait_ready=False, baud=921600,heartbeat_timeout=120)
# vehicle = connect('/dev/ttyUSB0',wait_ready=False, baud=921600,heartbeat_timeout=120)
# print(vehicle)

# # vehicle.wait_ready(True, raise_exception=False)
# #vehicle.gimbal.rotate(-70,0,0)
# # vehicle is an instance of the Vehicle class
# print("Relative altitude: %s" %vehicle.rangefinder.distance)
# print("Autopilot Firmware version: %s" %vehicle.version)

# print("Autopilot capabilities (supports ftp): %s" %vehicle.capabilities.ftp)

# print("Global Location: %s" % vehicle.location.global_frame)

# print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)

# print("Local Location: %s" % vehicle.location.local_frame)    #NED

# print("Attitude: %s" % vehicle.attitude)

# print("Velocity: %s" % vehicle.velocity)

# print("GPS: %s" % vehicle.gps_0)

# print("Groundspeed: %s" % vehicle.groundspeed)

# print("Airspeed: %s" % vehicle.airspeed)

# print("Gimbal status: %s" % vehicle.gimbal)

# print("Battery: %s" % vehicle.battery)

# print("EKF OK?: %s" % vehicle.ekf_ok)

# print("Last Heartbeat: %s" % vehicle.last_heartbeat)

# print("Rangefinder: %s" % vehicle.rangefinder)

# print("Rangefinder distance: %s" % vehicle.rangefinder.distance)

# print("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)

# print("Heading: %s" % vehicle.heading)

# print("Is Armable?: %s" % vehicle.is_armable)

# print("System status: %s" % vehicle.system_status.state)

# print("Mode: %s" % vehicle.mode.name)    # settable

# print("Armed: %s" % vehicle.armed)    # settable
