

import cv2
import numpy as np
import socket
import struct
import math

class FrameSegment(object):
    """ 
    Object to break down image frame segment
    if the size of image exceeds maximum datagram size 
    """
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64  # extract 64 bytes in case UDP frame overflows
    FRAME_WIDTH = 160
    FRAME_HEIGHT = 160
    
    def __init__(self, sock, port, addr="169.254.204.227"):
        self.s = sock
        self.port = port
        self.addr = addr

    def udp_frame(self, img):
        """ 
        Compress image and Break down
        into data segments 
        """
        resized_img = cv2.resize(img, (self.FRAME_WIDTH, self.FRAME_HEIGHT))
        compress_img = cv2.imencode('.jpg', resized_img)[1]
        dat = compress_img.tobytes()
        size = len(dat)
        count = math.ceil(size / self.MAX_IMAGE_DGRAM)
        array_pos_start = 0
        while count:
            array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
            self.s.sendto(struct.pack("B", count) +
                dat[array_pos_start:array_pos_end], 
                (self.addr, self.port)
                )
            array_pos_start = array_pos_end
            count -= 1

def main():
    """ Top level main function """
    # Set up UDP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port = 12345

    fs = FrameSegment(s, port)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        fs.udp_frame(frame)
    cap.release()
    cv2.destroyAllWindows()
    s.close()

if __name__ == "__main__":
    main()
