import cv2
from PIL import Image
import numpy as np
from rail_extraction import RailExtraction

# Load the video
test_video_dir = "../low_light_data/20230813_190427.mp4"
# Define a function to adjust Y value
def adjust_y(ycc_frame, adjustment_factor):
    # Convert to YCrCb color space
    ycc_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
    ycc_frame[:, :, 0] = cv2.add(ycc_frame[:, :, 0], adjustment_factor)
    # Convert back to BGR color space
    output_frame = cv2.cvtColor(ycc_frame, cv2.COLOR_YCrCb2BGR)
    return output_frame

def adjust_gamma(frame, gamma=1.0):
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255
                      for i in np.arange(0, 256)]).astype(np.uint8)
    return cv2.LUT(frame, table)

def add_noise(frame, mean=0,stddev=25):
    noise = np.random.normal(mean, stddev, frame.shape).astype(np.uint8)
    noisy_image = cv2.add(frame, noise)
    return noisy_image

cap = cv2.VideoCapture(test_video_dir)
if not cap.isOpened():
    print("Error: videoCap failed.")
    exit()

while cap.isOpened():

    ret, frame = cap.read()
    if not ret:
        break

    # Adjust Y value to lower the light (you can change the adjustment factor)
    adjusted_frame = adjust_y(frame, 100)  # You can experiment with the adjustment factor


    # Apply gamma correction (you can adjust the gamma value)
    gamma_corrected_frame = adjust_gamma(adjusted_frame, gamma=1)

    # noisy_frmae = add_noise(gamma_corrected_frame, 0, 0.2)

    rail_extract = RailExtraction()

    rail_extract.set_moving_avg_step_size(step_size=10)
    result = rail_extract.rail_extraction(gamma_corrected_frame)

    cv2.imshow('result',result)
    cv2.waitKey(1)

# Release the video capture and writer
cap.release()
# out.release()
cv2.destroyAllWindows()