#!/usr/bin/env python

import cv2
import numpy as np
import socket
import struct
import math
import argparse
import threading 

parser = argparse.ArgumentParser()
parser.add_argument('-ip', '--IP', type=str, default='0.0.0.0')
parser.add_argument('-p', '--PORT', type=int, default=8888)
args = parser.parse_args()




class FrameSegment(object):
    """ Object to break down image frame segment if the size of the image exceeds the maximum datagram size """
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64  # extract 64 bytes in case UDP frame overflows
    FRAME_WIDTH = 160
    FRAME_HEIGHT = 160

    def __init__(self, sock, addr="0.0.0.0"):
        self.sock = sock
        self.addr = addr
    
    def set_sock(self, sock, addr):
        self.sock=sock
        self.addr=addr

    def udp_frame(self, img):
        """ Compress image and break down into data segments """
        resized_img = cv2.resize(img, (self.FRAME_WIDTH, self.FRAME_HEIGHT))
        compress_img = cv2.imencode('.jpg', resized_img)[1]
        dat = compress_img.tobytes()
        size = len(dat)
        count = math.ceil(size / self.MAX_IMAGE_DGRAM)
        array_pos_start = 0
        while count:
            array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
            self.sock.sendto(struct.pack("B", count) +
                             dat[array_pos_start:array_pos_end],
                             self.addr)
            array_pos_start = array_pos_end
            count -= 1

def handle_client(client_socket, client_address):
    frame_segment = FrameSegment(client_socket, client_address)
    frame_segment.set_sock(client_socket, client_address)

    cap = cv2.VideoCapture(0)    

    while True:
        data = client_socket.recv(FrameSegment.MAX_DGRAM)  # Receive data from client
        if not data:
            break

        print(f"Received data from {client_address}: {data.decode('utf-8')}")

        while cap.isOpened():
            ret, frame = cap.read()
            print(type(frame))
            if not ret:
                break

            frame_segment.udp_frame(frame)

    client_socket.close()

def main():
    # Set up UDP server socket
    server_ip = args.IP
    server_port = args.PORT
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((server_ip, server_port))

    print(f"Server listening on {server_ip}:{server_port}")

    data, client_address = server_socket.recvfrom(FrameSegment.MAX_DGRAM)  # Receive data from client
    print(f"Received data from {client_address}: {data.decode('utf-8')}")


    frame_segment = FrameSegment(server_socket)
    frame_segment.set_sock(server_socket, client_address)

    cap = cv2.VideoCapture(0)    

    while True:
        # data = server_socket.recv(FrameSegment.MAX_DGRAM)  # Receive data from client
        # if not data:
        #     break

        # print(f"Received data from {client_address}: {data.decode('utf-8')}")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame_segment.udp_frame(frame)

    server_socket.close()

    # while True:
    #     # Accept incoming client connection
    #     data, client_address = server_socket.recvfrom(FrameSegment.MAX_DGRAM)  # Receive data from client
    #     print(f"Received data from {client_address}: {data.decode('utf-8')}")

    #     # client_thread = threading.Thread(target=handle_client, args=(server_socket, client_address))
    #     # client_thread.start()
        

if __name__ == "__main__":
    main()
