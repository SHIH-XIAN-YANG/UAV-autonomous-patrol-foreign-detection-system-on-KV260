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


# Define a custom signal handler
def handle_ctrl_c(signal, frame):
    global fd
    global shm
    global out
    global rail_extractor


    rail_extractor.save_log()
    out.release()
  
    shm.close()
    print("Ctrl+C received. Exiting gracefully.")
    exit(0)

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
    rail_extractor.post_processing_enable(True)

    rail_extractor.cam_initialize(exposure=250,brightness=128)
    rail_extraction.set_demo_mode(True)
    
    
    video_path = './video/no_obj.mp4'
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    
    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4
    time_stemp = datetime.now()
    file_name = time_stemp.strftime("%m_%d_%H_%M")

    output_file = f"./run/{file_name}_raw.mp4"
    output_file_IP = f"./run/{file_name}_IP.mp4"
    fps = 15  # Frames per second
    frame_width = 1920  # Width of the frames
    frame_height = 1028  # Height of the frames

    # Initialize the VideoWriter
    out_raw = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))
    out_IP = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

    
    print('cam start')
    
    while(True):
        try:
            ret, frame = cap.read()
    
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            out_raw.write(frame) 

            rail_frame = rail_extractor.rail_extraction(frame)

            out_IP.write(frame)

        except Exception as e:
            print(f'Error video_test: {e}')
            continue


    rail_extractor.save_log()
    out.release()
    cap.release()

if __name__ == '__main__':
    main()
