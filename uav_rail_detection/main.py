#%%
from rail_extraction import RailExtraction
from utils.moving_avg_filter import MovingAverageFilter
from multiprocessing import shared_memory
import sys
import threading

import cv2


def run():
    rail_extract = RailExtraction()

    rail_extract.set_moving_avg_step_size(step_size=10)
    rail_extract.save_enable(False)
    rail_extract.post_processing_enable(True)
    # rail_extract.shm_init()
    # rail_extract.cam_initialize(exposure=15,brightness=100)
    rail_extract.set_demo_mode(True)

    # rail_extract.sock_listen()

    test_video_dir = "./video/"

    
    cap = cv2.VideoCapture(test_video_dir + 'rail1.mp4')

    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for saving video (XVID is a common choice)
    # out = cv2.VideoWriter('output.mp4', fourcc, 15.0, (160, 160))  # Output filename, codec, frames per second, and frame size

    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break

        frame = cv2.resize(frame, (640,360), cv2.INTER_AREA)

        processed_frame = rail_extract.rail_extraction(frame)

        #if rail_extract.frame_counter:
            #out.write(processed_frame)

        cv2.imshow('result', processed_frame)
        

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release() 
    #out.release()
    cv2.destroyAllWindows()
    # shm = shared_memory.SharedMemory(name="control command", create=True, size=sys.getsizeof(control_command))

if __name__ == '__main__':
    run()


#%%


# cap = cv2.VideoCapture(test_video_dir + '20230813_184929.mp4')

    
# while True:
#     ret, frame = cap.read()
    
#     if not ret:
#         break

#     processed_frame = rail_extract.rail_extraction(frame)

#     cv2.imshow('result', processed_frame)
#     cv2.waitKey(1)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()


# rail_extract.process_video(0)

#%%

# rail_extract.save_log()
# %%
