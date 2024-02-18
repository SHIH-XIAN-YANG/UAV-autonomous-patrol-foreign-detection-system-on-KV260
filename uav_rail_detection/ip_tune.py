from rail_extraction import RailExtraction
from utils.moving_avg_filter import MovingAverageFilter
from utils.frame_segment import FrameSegment


import cv2
import mmap
import os
# os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"]="0"
import socket

import argparse
import numpy as np
import time
from datetime import datetime

import threading
import signal

rail_extractor = None

#### shm and socket fd
shm = None
fd = None
out = None

yolo_frame = None # share frame between UDP thread and frame processing
server_socket = None

parser = argparse.ArgumentParser()
parser.add_argument('-ip', '--IP', type=str, default='0.0.0.0')
parser.add_argument('-p', '--PORT', type=int, default=7000)
args = parser.parse_args()



# Define a custom signal handler
def handle_ctrl_c(signal, frame):
    global fd
    global shm
    global out
    global rail_extractor

    rail_extractor.save_log()
    out.release()
    # os.close(yolo_fd)
    # os.close(fd)
    # shm.close()
    server_socket.close()
    print("Ctrl+C received. Exiting gracefully.")
    

    # You can perform cleanup operations here if needed.
    # For example, closing files, releasing resources, etc.
    exit(0)


def UDP_server_thread():
    global server_socket
    global yolo_frame
    # Set up UDP server socket
    server_ip = args.IP
    server_port = args.PORT



    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((server_ip, server_port))
    print(f"Server listening on {server_ip}:{server_port}")

    # data, client_address = server_socket.recvfrom(FrameSegment.MAX_DGRAM)  # Receive data from client
    # print(f"Received data from {client_address}: {data.decode('utf-8')}")

    frame_segment = FrameSegment(server_socket)
    # frame_segment.set_sock(server_socket, client_address)

    

    try:
        while True:    
            data, client_addr = server_socket.recvfrom(256)
            # print(f"Received data from {client_addr}: {data.decode('utf-8')}")
            frame_segment.set_sock(client_addr)
            frame_segment.udp_frame(yolo_frame)
            # time.sleep(0.1)

    except Exception as e:
        print(f"An error occurred: {str(e)}")

        

def main():
    global fd
    global shm
    global yolo_frame
    global out
    global rail_extractor

    # register a signal to handle interrupted exit (ctrl+c)
    signal.signal(signal.SIGINT, handle_ctrl_c)

    rail_extractor = RailExtraction()
    rail_extractor.set_moving_avg_step_size(step_size=10)
    rail_extractor.save_enable(False)
    rail_extractor.post_processing_enable(True)
    # rail_extractor.shm_init()

    
    udp_thread = threading.Thread(target=UDP_server_thread)
    udp_thread.daemon = True
    # udp_thread.start()
    
    video_path = './video/rail3.mp4'
    cap = cv2.VideoCapture(video_path)
    
    rail_extractor.cam_initialize()
  

    if not cap.isOpened():
        print("Cannot open camera")
        exit()


    print('cam start')
    
    while(True):

        try:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            frame = cv2.resize(frame, (640,360), cv2.INTER_AREA)


            frame = rail_extractor.rail_extraction(frame)

            cv2.imshow('frame', frame)

            if cv2.waitKey(1) == ord('q'):
                break

            time.sleep(0.1)
        except Exception as e:
            continue


    # rail_extractor.save_log()
    out.release()
    cap.release()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
