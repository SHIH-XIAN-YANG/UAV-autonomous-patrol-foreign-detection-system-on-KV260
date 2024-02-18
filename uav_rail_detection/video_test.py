from rail_extraction import RailExtraction
from utils.moving_avg_filter import MovingAverageFilter
from utils.frame_segment import FrameSegment


import cv2
import mmap
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"]="0"
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
rail_frame = None
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
    global server_socket

    rail_extractor.save_log()
    out.release()
    # os.close(yolo_fd)
    # os.close(fd)
    shm.close()
    server_socket.close()
    print("Ctrl+C received. Exiting gracefully.")
    
    # You can perform cleanup operations here if needed.
    # For example, closing files, releasing resources, etc.
    exit(0)

def UDP_server_thread():
    global server_socket
    global yolo_frame
    global rail_frame
    # Set up UDP server socket
    server_ip = args.IP
    server_port = args.PORT

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((server_ip, server_port))
    print(f"Server listening on {server_ip}:{server_port}")

    frame_segment = FrameSegment(server_socket)
    try:
        while True:    
            data, client_addr = server_socket.recvfrom(256)
            frame_segment.set_sock(client_addr)
            frame_segment.udp_frame(rail_frame)
            # time.sleep(0.1)

    except Exception as e:
        print(f"An error occurred: {str(e)}")
  
def main():
    global fd
    global shm
    global yolo_frame
    global rail_frame
    global out
    global rail_extractor

    # register a signal to handle interrupted exit (ctrl+c)
    signal.signal(signal.SIGINT, handle_ctrl_c)

    # Initialize rail extractor feature
    rail_extractor = RailExtraction()
    rail_extractor.set_moving_avg_step_size(step_size=10)
    rail_extractor.save_enable(True)
    rail_extractor.post_processing_enable(False)
    rail_extractor.shm_init()
    rail_extractor.cam_initialize(exposure=5,brightness=80)
    rail_extractor.set_demo_mode(True)
    
    udp_thread = threading.Thread(target=UDP_server_thread)
    udp_thread.daemon = True
    udp_thread.start()
    
    video_path = './video/rail1.mp4'
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    

    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    shm = mmap.mmap(fd, 0x2000000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, mmap.ACCESS_WRITE, 0x76000000)


    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4
    time_stemp = datetime.now()
    file_name = time_stemp.strftime("%m_%d_%H_%M")

    output_file = f"./run/{file_name}.mp4"
    fps = 15  # Frames per second
    frame_width = 1920  # Width of the frames
    frame_height = 1080  # Height of the frames

    # Initialize the VideoWriter
    out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))
    
    print('cam start')
    try:
        while(True):
            
            ret, frame = cap.read()

            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            out.write(frame) 
            frame = cv2.resize(frame, (640,360), cv2.INTER_AREA)
            

            yolo_frame = cv2.resize(frame, (160, 160), cv2.INTER_AREA)
            # # Write the frame to the output video
            
            rail_frame = rail_extractor.rail_extraction(frame)
        
            shm.seek(0)

            shm.write(yolo_frame-128)

            time.sleep(0.1)

    except Exception as e:
        print(f'Error video_test: {e}')
            


    rail_extractor.save_log()
    out.release()
    cap.release()

if __name__ == '__main__':
    main()
