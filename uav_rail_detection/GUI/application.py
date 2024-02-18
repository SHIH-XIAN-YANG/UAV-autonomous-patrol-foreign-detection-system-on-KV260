import sys
import threading
import socket
import threading
import cv2
import struct
import signal
import numpy as np
import threading
import multiprocessing
import pickle
import time

from PyQt5 import QtCore, QtGui, QtWidgets
from GCS import Ui_GCS

from image_queue import ImageQueue

class Application():
    def __init__(self) -> None:
        self.qt_app = QtWidgets.QApplication(sys.argv)
        self.gui = Ui_GCS()
        self.GCS = QtWidgets.QDialog()
        self.gui.setupUi(self.GCS)
        self.gui.Connect_button.clicked.connect(self.connect_to_drone)
        self.gui.open_cam_button.clicked.connect(self.open_cam)
        self.BUFFER_SIZE=1024


        self.server_IP = '192.168.0.243'
        self.server_PORT = 8888
        self.connected:bool = False
        self.udp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_cam_request = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.drone_info_thread = threading.Thread(target=self.receive_drone_info)
        self.drone_info_thread.daemon = True
        
        self.video_frame_thread = threading.Thread(target=self.receive_video, args=(self.server_IP, self.server_PORT))
        self.video_frame_thread.daemon = True

        # Initial a lock when it is disconnect state the thread will be locked
        self.drone_info_lock = threading.Event()
        self.video_frame_lock = threading.Event()

        self.drone_info = {}

        self.cam_opened = False
        
        
        # self.anomaly_queue = multiprocessing.Queue()
        self.image_queue = ImageQueue()
        self.current_frame_cnt = 0
        self.anomaly_detected_flag = False

    def open_cam(self):
        try:
            if self.cam_opened == False:
                message = 'open'
                self.tcp_cam_request.send(message.encode('utf-8'))
                self.cam_opened = True
                self.gui.open_cam_button.setText('Close camera')
                print(f'camera opened')
            else:
                message = 'close'
                self.tcp_cam_request.send(message.encode('utf-8'))
                self.cam_opened = False
                self.gui.open_cam_button.setText('close camera')
                self.gui.video_frame.setText("")
                self.gui.video_frame.setPixmap(QtGui.QPixmap("UI/icon.png"))
                self.gui.video_frame.update()
                
                print(f'camera closed')
        except Exception as e:
            print(f"Error: open_cam {str(e)}")

        

    def receive_video(self, ip_address, port):
        # print(ip_address, port)
        try:
            # Create a socket object and connect to the server using UDP
            # self.udp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_client_socket.settimeout(0.5)
            # # Send a request to the server to initiate video streaming
            # request = b"START_VIDEO_STREAM"  # Replace with your actual request
            # self.udp_client_socket.sendto(request, (ip_address, port))
            # dat = b''
            # self.dump_buffer(self.udp_client_socket)

            # self.gui.Connect_button.setText('Disconnect')

            rcv_frame_count = 0

            message = f'send_{rcv_frame_count}'
            self.udp_client_socket.sendto(message.encode('utf-8'), (ip_address, port))
            
            while not self.video_frame_lock.is_set():

                if self.cam_opened or self.anomaly_detected_flag:
                   
                    rcv_frame_count+=1
                    
                    dat = b''
                    
                    try:
                        while True:
                            data, _ = self.udp_client_socket.recvfrom(self.BUFFER_SIZE)  # Adjust buffer size as needed
                            dat+=data
                            if len(data)<self.BUFFER_SIZE:
                                break
                    except socket.timeout:
                        print('recv time out!')
                        continue

                    try:
                        frame =  cv2.imdecode(np.frombuffer(dat, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frame is not None and frame.shape[0] > 0 and frame.shape[1] > 0:

                            # self.image_queue.push(frame)
                            
                            # if self.anomaly_detected_flag:
                            #     self.image_queue.save_images(str(self.drone_info['Location'])[15:])
                            frame = cv2.resize(frame, (640, 480), cv2.INTER_AREA)
                            height, width, channel = frame.shape
                            bytes_per_line = 3 * width
                            q_image = QtGui.QImage(frame.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
                            pixmap = QtGui.QPixmap.fromImage(q_image)

                            # Update the QLabel with the received video frame
                            self.gui.video_frame.setPixmap(pixmap)
                            self.gui.video_frame.update()
                    except Exception as e:
                        # Handle connection errors here (e.g., display an error message)
                        print(f"Error receive_video: {str(e)}")
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    self.gui.video_frame.setText("")
                    self.gui.video_frame.setPixmap(QtGui.QPixmap("UI/icon.png"))
                    self.gui.video_frame.update()

        except Exception as e:
            # Handle connection errors here (e.g., display an error message)
            print(f"Error receive_video: {str(e)}")

    def receive_drone_info(self):
        try:
            while not self.drone_info_lock.is_set():

                # Receive the pickled data from the server
                data_serialized = self.tcp_client_socket.recv(2**16)  # Adjust the buffer size if needed

                if not data_serialized:
                    # Server has closed the connection or no more data to receive
                    break

                self.drone_info = pickle.loads(data_serialized)
                # print(self.drone_info)
                if self.drone_info['Anomaly']=='Anomaly detected':
                    self.anomaly_detected_flag = True

                self.gui.Anomaly.setText(self.drone_info['Anomaly'])
                self.gui.Altitude.setText(self.drone_info['Altitude'])
                self.gui.Location.setText(str(self.drone_info['Location']))
                self.gui.Battery.setText(str(self.drone_info['Battery'])+' V')
                # self.gui.Velocity.setText(str(self.drone_info['Velocity']))
                self.gui.Attitude.setText(str(self.drone_info['Attitude']))
                self.gui.VehiclaMode.setText(str(self.drone_info['VehicleMode']))
                # self.gui.Drone_State.setText(str(self.drone_info['Drone_State']))
                time.sleep(0.025)

        except Exception as e:
            print(f"Error receive_drone_info: {str(e)}")

    def GUI_reset(self):
        self.gui.video_frame.setText("")
        self.gui.video_frame.setPixmap(QtGui.QPixmap("UI/icon.png"))
        self.gui.video_frame.update()

        self.gui.Anomaly.setText('-')
        self.gui.Altitude.setText('-')
        self.gui.Location.setText('-')
        self.gui.Battery.setText('-')
        self.gui.Attitude.setText('-')
        self.gui.VehiclaMode.setText('-')

    def connect_to_drone(self):

        self.server_IP = self.gui.IP.text()
        self.server_PORT = int(self.gui.Port.text())
        
        try:
        
            if self.connected is not True:

                self.tcp_client_socket.connect((self.server_IP , self.server_PORT+1))
                self.tcp_cam_request.connect((self.server_IP, self.server_PORT+2))
                # self.gui.Connect_button.setText('Connect')
                self.gui.Connect_button.setText('Disconnect')

                # release event triger to restart the thread
                self.drone_info_lock.clear()
                self.video_frame_lock.clear()

                self.drone_info_thread = threading.Thread(target=self.receive_drone_info)
                self.drone_info_thread.daemon = True
                self.video_frame_thread = threading.Thread(target=self.receive_video, args=(self.server_IP, self.server_PORT))
                self.video_frame_thread.daemon = True
                self.video_frame_thread.start()
                self.drone_info_thread.start()

                # Enable the open camera button
                self.gui.open_cam_button.setEnabled(True)

                # set the connected flag True
                self.connected = True
                
            else:
                self.GUI_reset()
                
                # trigger the event to stop the current thread
                self.drone_info_lock.set()
                self.video_frame_lock.set()

                self.video_frame_thread.join()
                self.drone_info_thread.join()

                # close current socket
                self.udp_client_socket.close()
                self.tcp_client_socket.close()
                self.tcp_cam_request.close()

                self.connected = False
                self.gui.Connect_button.setText('Connect')
                self.gui.open_cam_button.setDisabled(True)
                

                

        except KeyboardInterrupt:
            print("Main process received a keyboard interrupt.")
            self.drone_info_lock.set()
            self.video_frame_lock.set()
        
        self.video_frame_thread.join()
        self.drone_info_thread.join()

            
            

    def close_window(self):
        self.GCS.close()
        self.tcp_cam_request.close()
        self.udp_client_socket.close()
        self.tcp_client_socket.close()
        # sys.exit(self.qt_app.exec())
        # close the main thread the other thread will be terminate as well
        sys.exit(0)

    def run(self):
        self.GCS.show()
        sys.exit(self.qt_app.exec())



if __name__ == "__main__":
    application = Application()
    application.run()
    # sys.exit(application.qt_app.exec_())
