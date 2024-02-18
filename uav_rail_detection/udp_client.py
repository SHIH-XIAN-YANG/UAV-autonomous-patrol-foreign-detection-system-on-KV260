import socket
import cv2
import numpy as np

HOST = '192.168.50.25'  # 服务器的IP地址
PORT = 7000
BUFFER_SIZE = 1024
TIMEOUT = 0.5 # 50ms
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

s.settimeout(TIMEOUT)

i=0
while True:
    message = f'send_{i}'
    i+=1
    s.sendto(message.encode('utf-8'), (HOST, PORT))
    received_data = b""
    try:
        while True:
            chunk, _ = s.recvfrom(BUFFER_SIZE)
            received_data += chunk
            if len(chunk) < BUFFER_SIZE:
                break
    except socket.timeout:
        print("recvfrom time out!")
        continue

    # if len(received_data) > 0:
    try:
        image = cv2.imdecode(np.frombuffer(received_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        
        if image is not None and image.shape[0] > 0 and image.shape[1] > 0:
            # image = cv2.resize(image,(320,320))
            cv2.imshow('Received Image', image)
        else:
            # print("無法成圖")
            continue
    except:
            continue
    # 显示图像
    # cv2.imshow('Received Image', image)
    key = cv2.waitKey(1)  # 1秒钟的延迟，单位是毫秒

    if key == ord('q'):  # 如果按下 'q' 键则退出循环
        break
cv2.destroyAllWindows()