import os
import sys
import cv2
import numpy as np
import csv
import math
import time
from multiprocessing import shared_memory
from multiprocessing import Process
from multiprocessing import Queue
import socket
import struct
import subprocess
import signal
import pickle
import random


from utils.frame_segment import FrameSegment
from utils.image_queue import ImageQueue

import threading
import mmap

############ Dronekit ##############
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

######### Global attribute ##########
#### shm and socket fd
yolo_shm = None
yolo_fd = None
fd = None 
shm = None

drone_info_thread = None
tcp_process = None
udp_process = None
queue = Queue()
# drone_info_process = None


UDP_server_socket = None
TCP_server_socket = None
CAM_server_socket = None

UDP_HOST = '0.0.0.0'
UDP_PORT = 8888
TCP_HOST = '0.0.0.0'
TCP_PORT = 8889 
CAM_HOST = '0.0.0.0'
CAM_PORT = 8890 

open_cam = True
image_queue = ImageQueue(max_size=10)

# return attribute --drone-info--
drone_info = {
    'Anomaly' :     None,
    'Location' :    None,
    'Altitude' :    None,
    'Battery':      None,
    'Velocity' :    None,
    'Drone_State' : None,
    'VehicleMode' : None,
    'Attitude' :    None
}


# Define a custom signal handler
def handle_ctrl_c(signal, frame):
    global fd
    global shm
    global yolo_shm
    global yolo_fd
    global UDP_server_socket
    global TCP_server_socket
    global tcp_process
    global udp_process
    
    try:
        if TCP_server_socket:
            TCP_server_socket.close()
        if UDP_server_socket:
            UDP_server_socket.close()
        if CAM_server_socket:
            CAM_server_socket.close()
        # tcp_process.terminate()
        # udp_process.terminate()
        
        print("Exiting the server...")
    except Exception as e:
        print(f'Error at handle_ctrl_c: {str(e)}')
    
    sys.exit(0) 



# Function to handle individual TCP clients
def handle_tcp_client(client_socket):
    global drone_info

    # Process the received data here, you can modify this part as needed.
    # For this example, we'll just echo the data back to the client.

    try:
        while True:
            # Pickle the data and send it to the client
            data_serialized = pickle.dumps(drone_info)
            client_socket.send(data_serialized)
            time.sleep(0.02)
    except Exception as e:
        print(f"Error handle_tcp_client: {str(e)}")


def open_cam_request_handler(client_sock):
    global CAM_HOST
    global CAM_PORT
    global CAM_server_socket
    global open_cam
    global queue

    while True:
        try:
            data = client_sock.recv(256)
            if data:
                recv_data = data.decode('utf-8')

                print(f"{recv_data} camera")

                if recv_data == 'open':
                    open_cam = True
                    queue.put('1')
                else:
                    open_cam = False
                    _ = queue.get()
        except Exception as e:
            print(f'Error open_cam_request_handler: {str(e)}')

    

def tcp_server(queue_anomaly):
    global TCP_HOST
    global TCP_PORT
    global TCP_server_socket
    global drone_info_thread
    global CAM_server_socket
    global queue

    queue = queue_anomaly
    
    drone_info_thread = threading.Thread(target=handle_drone_info)
    drone_info_thread.daemon = True
    drone_info_thread.start()

    TCP_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    TCP_server_socket.bind((TCP_HOST, TCP_PORT))
    TCP_server_socket.listen(5)
    print(f'TCP server is listening on {TCP_HOST}:{TCP_PORT}')

    CAM_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    CAM_server_socket.bind((CAM_HOST, CAM_PORT))
    CAM_server_socket.listen(5)
    print(f'CAM server is listening on {CAM_HOST}:{CAM_PORT}')

    while True:
        try:
            client_socket, client_address = TCP_server_socket.accept()
            cam_socket, cam_addr = CAM_server_socket.accept()

            print(f"Accepted TCP connection from {client_address}")
            print(f"Accepted Camera connection from {cam_addr}")

            # Handle the client in a separate thread
            client_thread = threading.Thread(target=handle_tcp_client, args=(client_socket,))
            client_thread.daemon = True
            client_thread.start()

            cam_thread = threading.Thread(target=open_cam_request_handler, args=(cam_socket))
            cam_thread.daemon = True
            cam_thread.start()

        except KeyboardInterrupt:
            CAM_server_socket.close()
            TCP_server_socket.close()
            UDP_server_socket.close()
            print("Server shutting down.")
            break

        except Exception as e:
            print(f"Error tcp_server: {str(e)}")


def udp_server(queue_anomaly):
    global fd
    global shm
    global UDP_server_socket
    global UDP_PORT
    global UDP_HOST
    global queue

    queue = queue_anomaly

    try:
        UDP_server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        UDP_server_socket.bind((UDP_HOST, UDP_PORT))
        print(f"UDP Server listening on {UDP_HOST}:{UDP_PORT}")

        frame_size = 160 * 160 * 3
        frame_shape = (160, 160, 3)

        while not os.path.exists("/dev/mem"):
            print('wait for frame to open...')
            time.sleep(1) # wait until frame is opened and write to shared memory
            
        # Open shared memory to read frame
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        shm = mmap.mmap(fd, 0x2000000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, mmap.ACCESS_WRITE, 0x76000000)

        frame_segment = FrameSegment(UDP_server_socket)

        counter = 0

        # Accept incoming client connection
        _, client_address = UDP_server_socket.recvfrom(256)  # Receive data from client
        frame_segment.set_sock(client_address)

        # start transfer data
        print('UDP connected')

        while True:
            # Read frame from shared memory
            shm.seek(0)
            frame_data = shm.read(frame_size)
            read_frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(frame_shape)

            image_queue.push(read_frame+128)
            if queue.empty() == False:
                
                frame_segment.udp_frame(image_queue.pop())
                
                counter+=1
            else:
                continue
            

    except KeyboardInterrupt:
        print("Server terminated by the user.")
    except Exception as e:
        print(f"Error udp_server: {str(e)}")
        

def handle_drone_info():
    """
    Read anomaly detection state from shared memory and drone state with dronekit,
    and write it back to the dictionary drone_info.
    """
    global yolo_fd
    global yolo_shm
    global drone_info
    global open_cam
    global queue

    #shared memory: drone info
    drone_shm_path = "/dev/shm/drone_info_shared_memory"
    max_string_size = 128

    while not os.path.exists(drone_shm_path):
        print('wait for drone info shared memory to opened...')
        time.sleep(1)

    with open(drone_shm_path, "rb") as drone_shm_file:
        drone_shm_fd = drone_shm_file.fileno()  
        drone_shm = mmap.mmap(drone_shm_fd, max_string_size, mmap.MAP_SHARED, mmap.PROT_READ)


    cam_shm_path = "/dev/shm/drone_cam_shared_memory"

    while not os.path.exists(cam_shm_path):
        print('wait for yolo inference result to be opened...')
        time.sleep(1) # thread will sleep until shared memory path is opened

        shm_file = open(cam_shm_path, "wb")
        total_bytes = struct.calcsize('2i2f')
        shm_file.truncate(total_bytes)
        shm_file.close()


    shm_file = open(cam_shm_path, "r+b")
    shm_fd = shm_file.fileno()  
    total_bytes = struct.calcsize('2i2f') # calculate shared memory total bytes number
    control_command_shm = mmap.mmap(shm_fd, total_bytes, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)

    print('start write drone info')
    while True:

        control_command_shm.seek(0)
        _, anomaly_detected, _, _ = struct.unpack('2i2f', control_command_shm[:total_bytes])

        if anomaly_detected:
            open_cam = True
            drone_info['Anomaly'] = 'Anomaly detected'
            queue.put('1')
        else:
            open_cam = False
            drone_info['Anomaly'] = 'No object'
            _ = queue.get()

        drone_shm.seek(0)
        data_bytes = drone_shm.read(max_string_size)
        data = data_bytes.decode("utf-8").strip('\0')  
        data = data.split("|")

        # # print(f"Received string in the reader process:{data}")

        # Read drone attributes
        drone_info['Location'] = data[0]
        drone_info['Altitude'] = data[1]
        drone_info['Battery'] = data[2]
        drone_info['VehicleMode'] = data[3]
        drone_info['Attitude'] = data[4]
        

        print(drone_info)

        time.sleep(0.01)
        

def main():
    global tcp_process
    global udp_process
    global queue
    # global drone_info_process
    
    # register a signal to handle interrupted exit (ctrl+c)
    signal.signal(signal.SIGINT, handle_ctrl_c)

    # Create separate threads for TCP and UDP servers
    tcp_process = threading.Thread(target=tcp_server)
    udp_process = threading.Thread(target=udp_server)
    # drone_info_process = Process(target=handle_drone_info)

    tcp_process.start()
    udp_process.start()
    # drone_info_process.start()

if __name__ == "__main__":
    main()
