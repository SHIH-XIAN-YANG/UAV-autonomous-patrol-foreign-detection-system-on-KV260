import cv2
import mmap
import os
import sys
import numpy as np
import threading
import socket
import signal
import time
import struct

import multiprocessing

### global variable ###
fd = None
shm = None
cap = None



# Define a custom signal handler
def handle_ctrl_c(signal, frame):
    global fd
    global shm
    global cap
    os.close(fd)
    shm.close()
    print("Ctrl+C received. Exiting gracefully.")
    cap.release()
    # You can perform cleanup operations here if needed.
    # For example, closing files, releasing resources, etc.
    exit(0)

def main():
    global cap
    global fd
    global shm
    
    # register a signal to handle interrupted exit (ctrl+c)
    signal.signal(signal.SIGINT, handle_ctrl_c)

    video_source = './video_test.mp4'

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    shm = mmap.mmap(fd, 0x2000000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, mmap.ACCESS_WRITE, 0x76000000)

    """
    shm_path = "/dev/shm/drone_cam_shared_memory"

    # 打开共享内存文件，如果不存在则创建它并设置大小
    if not os.path.exists(shm_path):
        shm_file = open(shm_path, "wb")
        total_bytes = struct.calcsize('2i2f')
        shm_file.truncate(total_bytes)
        shm_file.close()

    # 打开共享内存
    shm_file = open(shm_path, "r+b")
    shm_fd = shm_file.fileno()  # 获取文件描述符
    total_bytes = struct.calcsize('2i2f') # calculate shared memory total bytes number


    print('open drone_cam_shared_memory')
    """
    print('cam start')

    # frame = cv2.imread('./test.jpg')



    while(True):
        
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # frame = cv2.imread('./test.jpg')

        # frame = cv2.imread('plane.jpg')
        
        frame = cv2.resize(frame, (160, 160), cv2.INTER_AREA)
        #frame = np.expand_dims(frame,axis=0)

        shm.seek(0)
        
        shm.write(frame-128)

        time.sleep(5)



        # 按下 q 鍵離開迴圈
        # if cv2.waitKey(1) == ord('q'):
        #     break

    # 釋放該攝影機裝置
    cap.release()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
