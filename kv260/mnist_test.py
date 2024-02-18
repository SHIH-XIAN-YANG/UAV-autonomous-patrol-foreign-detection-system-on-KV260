import cv2
import mmap
import os
import numpy as np
import time

img_dir = './mnist_test_data/'

fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)

shm = mmap.mmap(fd, 0x2000000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, mmap.ACCESS_WRITE, 0x76000000)

while True:
    for filename in os.listdir(img_dir):
        if filename.endswith('.jpg'):
            image_path = os.path.join(img_dir, filename)
            
            image = cv2.imread(image_path)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            #image = np.expand_dims(image, axis=0)

            # print(image.shape)
            # Convert image data to bytes and write to shared memory
            #image_bytes = image.astype(np.int8)
            
            shm.seek(0)
            shm.write(image - 128)

            print(f"Image written to shared memory: ",image_path)
    
        # Add some delay between iterations to avoid high CPU usage
        #cv2.waitKey(10000)
        time.sleep(1)
#cv2.destroyAllWindows()

