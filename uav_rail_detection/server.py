import os 
import sys
import numpy as np
import time
import mmap
import threading
import socket
import struct
import pickle
import signal

from utils.frame_segment import FrameSegment
from utils.image_queue import ImageQueue


yolo_shm = None
frame_shm = None
drone_shm = None

drone_info_thread = None
open_cam_thread = None
frame_thread = None

cam_request:bool = None
anomaly_detected:bool = None

frame_socket = None # UDP
open_cam_socket = None #TCP
drone_info_socket = None # UDP

IP = '0.0.0.0'
PORT = 8888

image_queue = ImageQueue(max_size=5) # pass 5 second frame

def handle_ctr_c(signal, frame):

    global yolo_shm


    global drone_shm

    global frame_socket
    global open_cam_socket
    global drone_info_socket

    try:
        yolo_shm.close()
        drone_shm.close()
        frame_socket.close()
        open_cam_socket.close()
        drone_info_socket.close()

    except Exception as e:
        print(f'Error handle_ctr_c: {e}')

    sys.exit(0)

def cam_client_handler(client_socket):
    global cam_request

    while True:
        try:
            data = client_socket.recv(16)
            if data:
                data = data.decode('utf-8')

                print(f'{data} camera')

                if data == 'open':
                    cam_request = True
                else: #close
                    cam_request = False
            time.sleep(1)
        except Exception as e:
            print(f'Error cam_client_handler: {e}')

def open_cam_handler():
    global IP
    global PORT

    global cam_request
    global open_cam_socket

    open_cam_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    open_cam_socket.bind((IP, PORT+2))
    open_cam_socket.listen(2)
    print(f'open camera request socket is listen at {IP}:{PORT+2}')

    while True:
        try:
            client_socket, client_addr = open_cam_socket.accept()
            print(f'Accept camera connection from {client_addr}')

            client_thread = threading.Thread(target=cam_client_handler,args=(client_socket, ))
            client_thread.daemon = True
            client_thread.start()
        except Exception as e:
            print(f'Error open_cam_handler: {e}')


def frame_handler():
    global IP
    global PORT

    global yolo_shm
    global frame_shm
    global anomaly_detected
    global cam_request
    global frame_socket
    

    # open YOLO detection result
    while not os.path.exists("/dev/shm/drone_cam_shared_memory"):
        print('wait for yolo to open...')
        time.sleep(1)
    fd = os.open("/dev/shm/drone_cam_shared_memory", os.O_RDWR | os.O_SYNC)
    yolo_shm = mmap.mmap(fd,struct.calcsize('2i2f'), mmap.MAP_SHARED, mmap.PROT_READ)


    # open frame shared memory
    while not os.path.exists('/dev/mem'):
        print('wait for frame to open...')
        time.sleep(1)
    fd = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
    frame_shm = mmap.mmap(fd, 0x2000000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, mmap.ACCESS_WRITE, 0x76000000)

    # open UDP frame socket
    frame_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame_socket.bind((IP, PORT))

    frame_segment = FrameSegment(frame_socket, PORT, IP)
    
    print('ready to send frame')
    while True:
        frame_socket.settimeout(3)
        try:
            # read yolo detection result
            yolo_shm.seek(0)
            _ , anomaly_detected,_,_ = struct.unpack('2i2f', yolo_shm)

            try:
                # Accept incoming client connection
                _, client_address = frame_socket.recvfrom(16)  # Receive data from client
                frame_segment.set_sock(client_address)
            except socket.timeout:
                print('frame recv timeout')
            
            if anomaly_detected == 1:
                """
                if anomaly detect is true, the frame counter will set to 10
                if no anomaly the counter will decrease util zero
                """
                
                # read frame from shared memory
                frame_shm.seek(0)
                frame_data = frame_shm.read(76800) #160*160*3
                frame_data = np.frombuffer(frame_data, dtype=np.uint8).reshape((160,160,3))
                # image_queue.push(frame_data)

                frame_segment.udp_frame(frame_data+128)
            elif cam_request:

                # read frame from shared memory
                frame_shm.seek(0)
                frame_data = frame_shm.read(76800) #160*160*3
                frame_data = np.frombuffer(frame_data, dtype=np.uint8).reshape((160,160,3))
                # image_queue.push(frame_data)

                frame_segment.udp_frame(frame_data+128)
            else:
                
                frame_segment.send_none()
            time.sleep(0.25)

        except Exception as e:
            print(f'Error frame sender: {e}')
            continue


def drone_info_handler():
    """
    Read anomaly detection state from shared memory and drone state with dronekit,
    and write it back to the dictionary drone_info.
    """
    global IP
    global PORT

    global drone_shm
    global drone_info_socket

    global yolo_shm
    global anomaly_detected

    global cam_request

    # return attribute --drone-info--
    drone_info = {
        'Anomaly' :     None,
        'Location' :    None,
        'Altitude' :    None,
        'Battery':      None,
        'VehicleMode' : None,
        'Attitude' :    None
    }

    # create drone info socket
    drone_info_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    drone_info_socket.bind((IP, PORT+1))

    # read drone info shared memory
    while not os.path.exists("/dev/shm/drone_info_shared_memory"):
        print('wait for drone info shared memory to opened...')
        time.sleep(1)
    with open("/dev/shm/drone_info_shared_memory", "r+b") as shm_file:
        shm_file.truncate(200)
        fd = shm_file.fileno()
        drone_shm = mmap.mmap(fd, 200, mmap.MAP_SHARED, mmap.PROT_READ)

    # open YOLO detection result
    while not os.path.exists("/dev/shm/drone_cam_shared_memory"):
        print('wait for yolo to open...')
        time.sleep(1)
    fd = os.open("/dev/shm/drone_cam_shared_memory", os.O_RDWR | os.O_SYNC)
    yolo_shm = mmap.mmap(fd,struct.calcsize('2i2f'), mmap.MAP_SHARED, mmap.PROT_READ)

    print('ready to send drone info')
    
    while True:
        try:
            # read yolo detection result
            yolo_shm.seek(0)
            _ , anomaly_detected,_,_ = struct.unpack('2i2f', yolo_shm)
            if anomaly_detected:
                drone_info['Anomaly'] = 'Anomaly detected'
            else:
                drone_info['Anomaly'] = 'No object'
                
            drone_shm.seek(0)
            data_bytes = drone_shm.read(200)
            data = data_bytes.decode("utf-8").strip('\0')  
            data = data.split("|")

            # Read drone attributes
            drone_info['Location']    = data[0]
            drone_info['Altitude']    = data[1]
            drone_info['Battery']     = data[2]
            drone_info['VehicleMode'] = data[3]
            drone_info['Attitude']    = data[4]

            loc_list = drone_info['Location'].split(',')
            s = (loc_list[0][0:7], loc_list[1][0:7])
            drone_info['Location'] = s[0] +' '+ s[1]
            att_list = drone_info['Attitude'].split(',')
            drone_info['Attitude'] = ""

            for att in att_list:
                att = att[0:12]
                drone_info['Attitude'] += ' ' + att    

            # Pickle the data and send it to the client
            _, client_address = drone_info_socket.recvfrom(16)
            data_serialized = pickle.dumps(drone_info)
            drone_info_socket.sendto(data_serialized, (client_address))

            time.sleep(2)

        except Exception as e:
            print(f'Error at drone_info_handler: {e}')
            continue

def main():
    global frame_thread
    global open_cam_thread
    global drone_info_thread

    # register a signal to handle interrupted exit (ctrl+c)
    signal.signal(signal.SIGINT, handle_ctr_c)

    frame_thread = threading.Thread(target=frame_handler)
    open_cam_thread = threading.Thread(target=open_cam_handler)
    drone_info_thread = threading.Thread(target=drone_info_handler)

    frame_thread.start()
    open_cam_thread.start()
    drone_info_thread.start()

    frame_thread.join()
    open_cam_thread.join()
    drone_info_thread.join()

if __name__ == "__main__":
    main()






        


    




