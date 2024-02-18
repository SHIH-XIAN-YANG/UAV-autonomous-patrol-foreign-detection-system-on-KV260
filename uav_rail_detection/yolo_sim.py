import mmap
import os
import struct
import time

shm_path = "/dev/shm/drone_cam_shared_memory"

# Check if the shared memory file exists, and if not, create it
if not os.path.exists(shm_path):
    # Create an empty file of the required size
    with open(shm_path, "wb") as shm_file:
        shm_file.write(b"\0" * struct.calcsize('2i2f'))

# Open the shared memory segment
fd = os.open(shm_path, os.O_RDWR | os.O_SYNC)
yolo_shm = mmap.mmap(fd, struct.calcsize('2i2f'), mmap.MAP_SHARED, mmap.PROT_WRITE)
yolo_shm.seek(4)
# Write integer values 1 and 0 into the shared memory
yolo_shm.write(struct.pack('i', 0))  # Write the integers 1 and 0
           
print("Wrote 0 into shared memory.")


while True:
    key = input()
    if key == 's':
        break
    else:
        print("Press 's' to start writing 1 to yolo...")

flag = False
try:
    while True:
        key = input()
        if key == 's':
            yolo_shm.seek(4)  # Move the cursor to the beginning of the shared memory
            yolo_shm.write(struct.pack('i', 0))  # Write the integers 1 and 0
            yolo_shm.flush()  # Make sure the data is written to the shared memory
            print('No object')
        else:
            yolo_shm.seek(4)  # Move the cursor to the beginning of the shared memory
            yolo_shm.write(struct.pack('i', 1))  # Write the integers 1 and 0
            yolo_shm.flush()  # Make sure the data is written to the shared memory    
            print('Anomaly detected')        
except KeyboardInterrupt:
    yolo_shm.close()

finally:
    yolo_shm.close()
