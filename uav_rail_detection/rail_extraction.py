import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import csv
import math
import time
import datetime
# from multiprocessing import shared_memory
import socket
import subprocess
import struct


from utils.moving_avg_filter import MovingAverageFilter
# from utils.frame_segment import FrameSegment

import threading
import mmap
# from PIL import Image

class RailExtraction():

    control_command_shm = None
    # yolo_shm = None

    def __init__(self):
        '''
        Enter GCS IP and PORT
        '''
        # webcam attribute
        self.fps = 30 #this feature will only enabled when it is webcam mode
        self.brightness = 100
        self.contrast = 50
        self.saturation = 100

        # frame attribute
        self.width = None
        self.height = None
        self.channel = None

        # UDP socket connection
        # s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.fs = FrameSegment(s,port=port,ip=ip)

        # Pre-processing attribute
        self.__blur_ksize = 3
        self.__canny_low = 150
        self.__canny_high = 160

        self.__erode_kernel = np.ones((3, 3), np.uint8)
        self.__dilation_kernel = np.ones((3, 3), np.uint8)
        self.__bibary_threshold = 50

        # Hough transform attribute
        self.__rho = 1
        self.__theta = np.pi / 180
        self.__threshold = 10
        self.__min_line_length = 200 # for 160*160 img size
        self.__max_line_gap = 10
        # Rail attribute
        self.__left_deg_bound = (-90, -45)
        self.__right_deg_bound = (45, 90)

        self.__last_right_frame_deg = 75
        self.__last_left_frame_deg = -75  # To be tued
 
        self.diff_deg = 15 # Threshold of difference between two consecutive ratio

        # self.left_deg_log = []
        # self.right_deg_log = []
        # self.center_deg_log = []


        # self.center_deg_log = []
        # self.center_deg_log_ma = []
        # self.center_rho_log = []
        # self.center_rho_log_ma = []

        self.heading_log = []
        self.offset_log = []
        # self.shm_log = []

        self.__moving_avg_step = 10
        self.moving_avg_filter_deg = MovingAverageFilter(self.__moving_avg_step)
        self.moving_avg_filter_bias = MovingAverageFilter(self.__moving_avg_step)

        self.post_processing_enabled:bool = True 
        self.process_freq = None # process every n frame
        self.shm_enable = False

        # save attribute

        self.save_enabled = False
        self.log_dir = './log/'
        self.output_video_dir = './run/'
        self.out_file = 0
        
        
        self.heading:np.float32 = None
        self.offset:np.float32 = None
        self.rail_distance_pix = None
        self.rail_distance = 1.26
        self.rail_distance_ratio = None 

        self.delta_heading:np.float16 = 0.0 
        self.delta_offset:np.float16 = 0.0
        self.rail_detected_flag:np.int8 = 0
        self.right_rail_detected_flag:np.int8 = 0
        self.left_rail_detected_flag:np.int8 = 0

        self.demo_mode = False

        
    
    def shm_init(self):
        self.shm_enable = True
        # Create share memory to store control command
        # Create a memory-mapped file
        shm_path = "/dev/shm/drone_cam_shared_memory"

        # open control command shared memory, if not exist, create it and allocate size
        if not os.path.exists(shm_path):
            print(f'{shm_path} not yet created')
            shm_file = open(shm_path, "wb")
            total_bytes = struct.calcsize('2i2f')
            shm_file.truncate(total_bytes)
            shm_file.close()

        # open shared memory
        shm_file = open(shm_path, "r+b")
        shm_fd = shm_file.fileno()  # 获取文件描述符
        total_bytes = struct.calcsize('2i2f') # calculate shared memory total bytes number
        self.control_command_shm = mmap.mmap(shm_fd, total_bytes, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)

        # # yolo inference result shared
        # if os.path.exists("/dev/shm/drone_cam_shared_memory"):
        #     yolo_shm_file = open("/dev/shm/drone_cam_shared_memory", "r+b")
        #     yolo_shm_fd = yolo_shm_file.fileno()  # 获取文件描述符

        #     self.yolo_shm = mmap.mmap(yolo_shm_fd, 4, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)
        # else:
        #     print('yolo result shared memory not exist...')



    def set_moving_avg_step_size(self, step_size: int):
        self.__moving_avg_step = step_size
        self.moving_avg_filter_bias.set_step_size(self.__moving_avg_step)
        self.moving_avg_filter_deg.set_step_size(self.__moving_avg_step)
    
    def post_processing_enable(self,enabled:bool=True):
        self.post_processing_enabled = enabled

    def save_enable(self, enable=True):
        """
        determine to save current frame or not
        """
        self.save_enabled = enable
    
    def set_demo_mode(self, demo_mode=True):
        self.demo_mode = demo_mode
    
    def calculate_slope(self, x1, y1, x2, y2):
        if x2 == x1:
            return float('inf')  # Handle vertical lines by returning infinity
        else:
            return (y2 - y1) / (x2 - x1)

    def rail_detection(self, frame: np.ndarray, frame_mode: str):
        """
        detect the rail of the frame
        """

        # Pre-Process
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # histeqaul_img = cv2.equalizeHist(gray)

        blur_img = cv2.GaussianBlur(gray, (self.__blur_ksize, self.__blur_ksize), 0)
        cv2.imshow(f'{frame_mode}blur_img',blur_img)
        # bilat_img = cv2.bilateralFilter(gray,3,50,50)

        #med_blur = cv2.medianBlur(gray,5)
        histeqaul_img = cv2.equalizeHist(gray)
        cv2.imshow(f'{frame_mode}histeqaul_img',histeqaul_img)

        if self.demo_mode:
            erode_img = cv2.erode(histeqaul_img, self.__erode_kernel, iterations=1)
            cv2.imshow(f'{frame_mode}erode',erode_img)
            histeqaul_img = cv2.dilate(erode_img, self.__dilation_kernel, iterations=1)
            cv2.imshow(f'{frame_mode}dilate',histeqaul_img)
        #cv2.imshow('dialate',blur_img)
        # Canny = cv2.Canny(histeqaul_img, 50, 100)

        sobel_y_img = cv2.Sobel(blur_img, -1, 1, 0)
        cv2.imshow(f'{frame_mode}sobel_y_img',sobel_y_img)

        _, binary_img = cv2.threshold(sobel_y_img, self.__bibary_threshold , 255,
                                         cv2.THRESH_BINARY)
        cv2.imshow(f'{frame_mode}binary_img',binary_img)
        #cv2.imshow(f'{frame_mode}_binary', binary_img)
        # _, binary_img = cv2.threshold(blur_img, 0 , 255,
        #                                  cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        
        # line detection base on Hough transform
        lines = cv2.HoughLinesP(binary_img, self.__rho, self.__theta, self.__threshold, 
                                np.array([]),self.__min_line_length, self.__max_line_gap)
        
        # line_image = np.copy(frame) * 0
        rho = None
        deg = None
        
        if lines is not None:


            lines = max(lines, key=lambda coord: math.dist((coord[0][0], coord[0][1]), (coord[0][2], coord[0][3])))

            for line in lines:
                x1, y1, x2, y2 = line
                # slope = self.calculate_slope(x1, y1, x2, y2)
                
                deg = np.rad2deg(math.atan2((y2-y1),(x2-x1)))
                # rho = (x1 * y2 - x2 * y1) / np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                

                if frame_mode=='left':

                    rho = (x1 * y2 - x2 * y1) / np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    if deg > self.__left_deg_bound[0] and deg < self.__left_deg_bound[1] \
                                                    and abs(self.__last_left_frame_deg - deg)<self.diff_deg:
                        # self.left_deg_log.append(deg)
                        
                        # cv2.putText(line_image, str(deg)[0:6], ((x1+x2)//2+15, (y1+y2)//2), \
                        #                 cv2.FONT_HERSHEY_COMPLEX,1, (255, 0, 255), 2, cv2.LINE_AA)

                        # cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
                        self.__last_left_frame_deg = deg

                else:

                    rho = ((x1 + self.width/2) * y2 - (x2 + self.width/2) * y1) / np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    if deg > self.__right_deg_bound[0] and deg < self.__right_deg_bound[1] \
                                                    and abs(self.__last_right_frame_deg - deg)<self.diff_deg:
                        # self.right_deg_log.append(deg)
                        
                        # cv2.putText(line_image, str(deg)[0:6], ((x1+x2)//2+15, (y1+y2)//2), \
                        #                 cv2.FONT_HERSHEY_COMPLEX,1, (255, 0, 255), 2, cv2.LINE_AA)

                        # cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 10)
                        self.__last_right_frame_deg = deg

            # print(f'no line in {frame_mode} frame')

        # frame = cv2.addWeighted(frame, 0.8, line_image, 1, 0)

        # cv2.imshow(frame_mode+'frame', frame)

        return deg, rho, lines
    
    def find_intersection(self, x1, y1, x2, y2, slope1, slope2):
        # Check if the slopes are equal (lines are parallel)
        if slope1 == slope2:
            return None, None  # Lines do not intersect
        
        # Calculate the x-coordinate of the intersection point
        if slope1==None:
            x_intersection = x1
        else:    
            x_intersection = (y2 - y1 + slope1 * x1 - slope2 * x2) / (slope1 - slope2)

        # Calculate the y-coordinate of the intersection point
        y_intersection = y2 + slope2 * (x_intersection - x2)

        return (x_intersection, y_intersection)
    
    

    def find_center_parameter(self, theta1,rho1,theta2,rho2):
        theta1 = np.deg2rad(theta1)
        theta2 = np.deg2rad(theta2)
        try:
            x_left1 = (rho1 + 0 * math.cos(theta1)) / math.sin(theta1)
            x_right1 = (rho2 + 0 * math.cos(theta2)) / math.sin(theta2)

            x_low = (x_left1 + x_right1)/2

            x_left2 = (rho1 + self.height * math.cos(theta1)) / math.sin(theta1)
            x_right2 = (rho2 + self.height * math.cos(theta2)) / math.sin(theta2)
 
            x_high = (x_left2 + x_right2)/2


            theta = math.atan2(self.height - 0, x_high - x_low) 
            rho = x_low * math.sin(theta) + 0 * math.cos(theta)

            slope = (x_low - x_high)/(self.height)
            if x_left1==x_left2:
                slope2 = None
            else:
                slope2 = self.height/(x_left2 - x_left1)
            if x_right1==x_right2:
                slope3 = None
            else:
                slope3 = self.height/(x_right2 - x_right1)
        except Exception as e:
            print(f'rror at find_center_parameter: {e}')
            return self.width/2, 90


        # find distance between two rail
        x1,y1 = self.find_intersection(x_left1,0,self.width/2,self.height/2, slope2,slope)
        x2,y2 = self.find_intersection(x_right1,0,self.width/2,self.height/2, slope3,slope)

        if x1==None:
            return self.width/2, 90
        else:
            self.rail_distance_pix = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        

        return rho, np.rad2deg(theta)

    def post_process(self, frame,left_deg,left_rho, right_deg, right_rho, center_rho, center_deg):

        # Draw boundary on frame 
        shapes = np.zeros_like(frame, np.uint8)
        cv2.rectangle(shapes, (self.width//2 - self.width//20 , 0), (self.width//2 + self.width//20, self.height), (100, 100, 100), cv2.FILLED)
        if left_deg!=0 and right_deg!=0:
            x_left_low = int((left_rho) / math.sin(np.deg2rad(left_deg)))
        
            x_left_top = int((left_rho + self.height * math.cos(np.deg2rad(left_deg))) / math.sin(np.deg2rad(left_deg)))
            x_right_low = int((right_rho) / math.sin(np.deg2rad(right_deg)))
            x_right_top = int((right_rho + self.height * math.cos(np.deg2rad(right_deg))) / math.sin(np.deg2rad(right_deg)))
        else:
            return frame
    
        cv2.line(shapes, (x_left_low, 0),(x_left_top, self.height), (0, 255, 0), 2)
        cv2.line(shapes, (x_right_low, 0),(x_right_top, self.height), (255, 0, 0), 2)
        
        if center_deg != 0:
            x1 = int((center_rho + self.height * math.cos(np.deg2rad(center_deg))) / math.sin(np.deg2rad(center_deg)))
            y1 = int(self.height)
            x2 = int(center_rho / math.sin(np.deg2rad(center_deg)))
            y2 = int(0)
            center_x = (center_rho + self.height/2 * math.cos(np.deg2rad(center_deg))) / math.sin(np.deg2rad(center_deg))
            center_y = self.height/2
            cv2.line(shapes, (x1, y1),(x2, y2), (255, 255, 0), 2)
        else:
            return frame

        # draw center line
        cv2.line(shapes, (self.width//2, 0), (self.width//2, self.height), (255, 255, 255), 1)
        cv2.line(shapes, (0, self.height//2), (self.width, self.height//2), (255, 255, 255), 1)


        cv2.line(shapes, (int(center_x),int(center_y)), (self.width//2, self.height//2), (255, 255, 255), 2)
        cv2.putText(shapes, str(self.delta_offset)[0:5], (int(self.width//2),int(self.height//2+25)),cv2.FONT_HERSHEY_COMPLEX,0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.putText(shapes, '*', (int(center_x-2),int(center_y+2)),cv2.FONT_HERSHEY_COMPLEX,0.2, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(shapes, str(self.delta_heading)[0:4], (int(center_x+9),int(center_y-5)),cv2.FONT_HERSHEY_COMPLEX,0.5, (255, 255, 0), 1, cv2.LINE_AA)

        alpha = 0.4
        mask = shapes.astype(bool)
        frame[mask] = cv2.addWeighted(frame, alpha, shapes, 1 - alpha, 0)[mask]

        return frame

    def center_line_extraction(self,frame, left_deg,left_rho,right_deg, right_rho):
        """
        detect the center of two rails
        """ 

        center_rho, center_deg = self.find_center_parameter(left_deg,left_rho, right_deg, right_rho)

        # self.center_deg_log.append(center_deg)
        try:
            center_x = (center_rho + self.height/2 * math.cos(np.deg2rad(center_deg))) / math.sin(np.deg2rad(center_deg))
            # print(center_deg)
            self.delta_heading = (np.abs(center_deg)-90)
            self.delta_offset = (center_x - self.width/2)
        except Exception as e:
            print(f'Error at center_line_extraction:{e}')
            self.delta_heading = (np.abs(center_deg)-90)
            self.delta_offset = 0

        # linear mapping from pixel to real world distance(m)
        self.rail_distance_ratio  =  self.rail_distance / self.rail_distance_pix
        self.delta_offset = self.delta_offset * self.rail_distance_ratio
        # print(self.delta_offset)

        self.delta_heading = self.moving_avg_filter_deg.update(self.delta_heading)
        self.delta_offset = self.moving_avg_filter_bias.update(self.delta_offset)
        # print(self.delta_heading, self.delta_offset)

        if self.delta_heading > 30:
            self.delta_heading = 0
        if self.delta_offset > 1.2:
            self.delta_offset = 0
        
        if self.post_processing_enabled:
            frame = self.post_process(frame,left_deg,left_rho, right_deg, right_rho, center_rho, center_deg)
        
        return frame, center_deg, center_rho

    def write_shm(self):
        try:
            # write if rail detected to shm
            data_bytes = struct.pack('i', self.rail_detected_flag)
            self.control_command_shm.seek(0)
            self.control_command_shm.write(data_bytes)

            # # Read yolo detection result from shm
            self.control_command_shm.seek(struct.calcsize('i'))
            shared_integer_bytes = self.control_command_shm.read(4)
            shared_integer = int.from_bytes(shared_integer_bytes, byteorder='little')


            # write command (heading/offset) to shm
            data_bytes = struct.pack('2f', self.delta_heading, self.delta_offset)
            self.control_command_shm.seek(2*struct.calcsize('i'))
            self.control_command_shm.write(data_bytes)

            print(f'yolo: {shared_integer} heading: {self.delta_heading} offset: {self.delta_offset}')

            if self.save_enabled:
                self.heading_log.append(self.delta_heading)
                self.offset_log.append(self.delta_offset)

        except:
            print('write control command to shm error')


    def rail_extraction(self, frame: np.ndarray):
        """
        detect the rail of the frame
        """
        self.height, self.width, self.channel = frame.shape
        # self.out_video = cv2.VideoWriter(self.output_video_dir+'anomaly.mp4', self.__fourcc, self.fps, (self.width, self.height), isColor=True)

        # Divide the frame into left and right halves
        left_frame = frame[:, :self.width // 2, :]
        right_frame = frame[:, self.width // 2:, :]

        left_deg, left_rho, left_line = self.rail_detection(left_frame, 'left')
        right_deg, right_rho, right_line = self.rail_detection(right_frame, 'right')


        # print(left_rho, left_deg, right_rho, right_deg)


        if left_line is not None and right_line is not None:
            frame, self.heading, self.offset = self.center_line_extraction(frame, left_deg, left_rho,right_deg, right_rho)
            self.rail_detected_flag = 1
        else:
            self.rail_detected_flag = 0
            self.delta_heading = 0
            self.delta_offset = 0
        
        # write control command to shared memory if shm exist
        if self.shm_enable:
            self.write_shm()

        return frame


    def cam_initialize(self, exposure=250, brightness=128):
        '''
        v4l2-ctl -d /dev/video0 -l

        User Controls

                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=100
                       contrast 0x00980901 (int)    : min=0 max=255 step=1 default=128 value=128
                     saturation 0x00980902 (int)    : min=0 max=255 step=1 default=128 value=128
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                           gain 0x00980913 (int)    : min=0 max=255 step=1 default=0 value=15
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=2 (60 Hz)
      white_balance_temperature 0x0098091a (int)    : min=2000 max=7500 step=10 default=4000 value=5230 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=255 step=1 default=128 value=128
         backlight_compensation 0x0098091c (int)    : min=0 max=1 step=1 default=1 value=0

        Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=3 max=2047 step=1 default=250 value=250 flags=inactive
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=1
                   pan_absolute 0x009a0908 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                  tilt_absolute 0x009a0909 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                 focus_absolute 0x009a090a (int)    : min=0 max=255 step=5 default=0 value=0 flags=inactive
     focus_automatic_continuous 0x009a090c (bool)   : default=1 value=1
                  zoom_absolute 0x009a090d (int)    : min=100 max=500 step=1 default=100 value=100

        '''
        # return_code = subprocess.call('v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=360,pixelformat=1', shell=True)
        # print(f"return code = {return_code}")
        cam_prop = {'brightness': brightness,'auto_exposure': 1 ,'exposure_time_absolute': exposure}
        for key in cam_prop:
            subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cam_prop[key]))], shell=True)

        #subprocess.call(['v4l2-ctl', '-d', '/dev/video0', '-p', '1'])

    def save_log(self):
        if self.shm_enable:

            current_time = datetime.datetime.now()
            timestamp = current_time.strftime("%m_%d_%H_%M")
            self.log_dir  = self.log_dir + timestamp + '/'
            if not os.path.exists(self.log_dir):
                os.mkdir(self.log_dir)

            # Create a figure with two subplots
            # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))  # 1 row, 2 columns of subplots
            plt.subplot(2,1,1)
            # Plot data on the first subplot
            plt.plot(self.heading_log)
            plt.xlabel('time')
            plt.ylabel('heading(degree)')
            # ax1.legend()

            # Plot data on the second subplot
            plt.subplot(2,1,2)
            plt.plot(self.offset_log*100, color='red')
            
            plt.xlabel('time')
            plt.ylabel('offset(cm)')
            # ax2.legend()
            # Adjust layout to prevent overlapping
            plt.tight_layout()

            plt.savefig(self.log_dir + 'command.png')

            print('save to log complete')
        else:
            print('save log not enabled')
        
        
        # with open(self.log_dir + 'heading_log.csv', mode='w', newline='') as csvfile:
        #     # Create a CSV writer object
        #     csv_writer = csv.writer(csvfile)

        #     # Write the list to the CSV file
        #     csv_writer.writerow(self.heading_log)

        # with open(self.log_dir + 'offset_log.csv', mode='w', newline='') as csvfile:
        #     # Create a CSV writer object
        #     csv_writer = csv.writer(csvfile)

        #     # Write the list to the CSV file
        #     csv_writer.writerow(self.offset_log)
        
        # with open(self.log_dir + 'shm_log.txt', mode='w', newline='') as csvfile:
        #     csv_writer = csv.writer(csvfile)

        #     # Write the list to the CSV file
        #     csv_writer.writerow(self.shm_log)

        

        # with open(self.log_dir + 'center_slope.csv', mode='w', newline='') as csvfile:
        #     # Create a CSV writer object
        #     csv_writer = csv.writer(csvfile)

        #     # Write the list to the CSV file
        #     csv_writer.writerow(self.center_deg_log)

        # with open(self.log_dir + 'center_slope_wth_ma.csv', mode='w', newline='') as csvfile:
        #     # Create a CSV writer object
        #     csv_writer = csv.writer(csvfile)

        #     # Write the list to the CSV file
        #     csv_writer.writerow(self.center_deg_log_ma)

        # with open(self.log_dir + 'center_bias.csv', mode='w', newline='') as csvfile:
        #     # Create a CSV writer object
        #     csv_writer = csv.writer(csvfile)

        #     # Write the list to the CSV file
        #     csv_writer.writerow(self.center_bias_log)

        # with open(self.log_dir + 'center_bias_wth_ma.csv', mode='w', newline='') as csvfile:
        #     # Create a CSV writer object
        #     csv_writer = csv.writer(csvfile)

        #     # Write the list to the CSV file
        #     csv_writer.writerow(self.center_bias_log_ma)
def main():

    # Initialize rail extractor feature
    rail_extractor = RailExtraction()
    rail_extractor.set_moving_avg_step_size(step_size=10)
    rail_extractor.save_enable(False)
    rail_extractor.post_processing_enable(True)

    rail_extractor.cam_initialize(exposure=15,brightness=128)
    rail_extractor.set_demo_mode(True)

    cap = cv2.VideoCapture(0)
    
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
            frame = cv2.resize(frame, (640,360))
            #out_raw.write(frame) 

            rail_frame = rail_extractor.rail_extraction(frame)

            cv2.imshow('frame', frame)

            if cv2.waitKey(1) == ord('q'):
                break

        except Exception as e:
            print(f'Error video_test: {e}')
            continue


    rail_extractor.save_log()
    out.release()
    cap.release()

if __name__ == '__main__':
    main()
        




        



    

    


    

