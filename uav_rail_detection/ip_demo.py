import cv2
from rail_extraction import RailExtraction
from utils.moving_avg_filter import MovingAverageFilter

import os
import time
import numpy as np

video_path = './video/rail_no_obj3.mp4'
cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

rail_extractor = RailExtraction()
rail_extractor.set_moving_avg_step_size(step_size=10)
rail_extractor.post_processing_enable(True)
rail_extractor.set_demo_mode(True)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4
out1 = cv2.VideoWriter('grey.mp4', fourcc, 30, (1280, 720),isColor=False)
out2 = cv2.VideoWriter('histeqaul_img.mp4', fourcc, 30, (1280, 720),isColor=False)
out3 = cv2.VideoWriter('blur_img.mp4', fourcc, 30, (1280, 720),isColor=False)
out4 = cv2.VideoWriter('erode_img.mp4', fourcc, 30, (1280, 720),isColor=False)
out5 = cv2.VideoWriter('dilate.mp4', fourcc, 30, (1280, 720),isColor=False)
out6 = cv2.VideoWriter('sobel_y_img.mp4', fourcc, 30, (1280, 720),isColor=False)
out7 = cv2.VideoWriter('binary_img.mp4', fourcc, 30, (1280, 720),isColor=False)
out = cv2.VideoWriter('result.mp4', fourcc, 30, (1280, 720))



while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Pre-Process
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    cv2.imshow('gray',gray)
    out1.write(gray)

    histeqaul_img = cv2.equalizeHist(gray)
    cv2.imshow('histeqaul_img',histeqaul_img)
    out2.write(histeqaul_img)
    
    blur_img = cv2.GaussianBlur(histeqaul_img, (3, 3), 0)
    cv2.imshow('blur_img',blur_img)
    out3.write(blur_img)
    

    erode_img = cv2.erode(blur_img, np.ones((3, 3), np.uint8), iterations=2)
    cv2.imshow('erode_img',erode_img)
    out4.write(erode_img)
    blur_img = cv2.dilate(erode_img, np.ones((5, 5), np.uint8), iterations=3)
    cv2.imshow('dilate',blur_img)
    out5.write(blur_img)

    sobel_y_img = cv2.Sobel(blur_img, -1, 1, 0)
    cv2.imshow('sobel_y_img',sobel_y_img)
    out6.write(sobel_y_img)

    _, binary_img = cv2.threshold(sobel_y_img, 125 , 255,
                                        cv2.THRESH_BINARY)
    cv2.imshow('binary_img',binary_img)
    out7.write(binary_img)
    frame = rail_extractor.rail_extraction(frame)

    

    out.write(frame)

    cv2.imshow('frame',frame)
    if cv2.waitKey(1) == ord('q'):
        break

# rail_extractor.save_log()
out.release()
out7.release()
out1.release()
out2.release()
out3.release()
out4.release()
out5.release()
out6.release()
cap.release()