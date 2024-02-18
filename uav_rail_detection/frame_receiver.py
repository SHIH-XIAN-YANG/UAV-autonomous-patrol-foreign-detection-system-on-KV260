#!/usr/bin/env python

from __future__ import division
import os
import cv2
import numpy as np
import socket
import struct
import argparse
import signal

MAX_DGRAM = 2**16

parser = argparse.ArgumentParser()
parser.add_argument('-ip','--IP', type=str,default='192.168.0.194')
parser.add_argument('-p','--PORT', type=int,default=8888)
args = parser.parse_args()

s = None

# Define a custom signal handler
def handle_ctrl_c(signal, frame):
    global s
    print('enter handle ctr...')
    s.close()
    cv2.destroyAllWindows()
    
    print("Ctrl+C received. Exiting gracefully.")
    

    # You can perform cleanup operations here if needed.
    # For example, closing files, releasing resources, etc.
    exit(0)


def dump_buffer(s):
    """ Emptying buffer frame """
    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        print(seg[0])
        if struct.unpack("B", seg[0:1])[0] == 1:
            print("finish emptying buffer")
            break

def main():
    global s
    """ Getting image udp frame &
    concate before decode and output image """

    # register a signal to handle interrupted exit (ctrl+c)
    signal.signal(signal.SIGINT, handle_ctrl_c)

    # Set up socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f'try to connect to {args.IP}, {args.PORT}')

    # Send a "ping" message to the server
    ping_message = b"Ping from client"
    s.sendto(ping_message, (args.IP, args.PORT))
    
    print(f'connected to {args.IP}, {args.PORT}')
    # s.bind((args.IP, args.PORT))
    dat = b''
    dump_buffer(s)

    while True:
        try:
            seg, addr = s.recvfrom(MAX_DGRAM)
            if struct.unpack("B", seg[0:1])[0] > 1:
                dat += seg[1:]
            else:
                dat += seg[1:]
                img = cv2.imdecode(np.frombuffer(dat, dtype=np.uint8), 1)
                img = cv2.resize(img, (640, 480), cv2.INTER_AREA)
                cv2.imshow('frame', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                dat = b''
        except socket.timeout:
            print('No response from server...')

    # cap.release()
    cv2.destroyAllWindows()
    s.close()

if __name__ == "__main__":
    main()