import cv2

def save_frames(input_video_path, output_folder):
    # Open the video file
    cap = cv2.VideoCapture(input_video_path)

    # Check if the video opened successfully
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    # Get some video properties
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    print(f"Total frames: {frame_count}")
    print(f"Frames per second: {fps}")

    # Create the output folder if it doesn't exist
    import os
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Loop through each frame and save it
    for frame_number in range(frame_count):
        if frame_number%10==0:
            ret, frame = cap.read()
            frame_number = frame_number
            # Check if the frame was read successfully
            if not ret:
                print(f"Error reading frame {frame_number}")
                break

            # Save the frame
            frame_filename = f"{output_folder}/frame_{frame_number:04d}.jpg"
            cv2.imwrite(frame_filename, frame)

            # Display progress
            if frame_number % 100 == 0:
                print(f"Processed {frame_number}/{frame_count} frames")

    # Release the video capture object
    cap.release()

    print("Frames saved successfully.")

# Example usage
input_video_path = 'C:\\Users\\Samuel\\Desktop\\uav_rail_anomaly_detection_system\\uav_rail_detection\\beito\\rail1_no_obj_1.mp4'
output_folder = 'C:\\Users\\Samuel\\Download\\beito_dataset\\'
save_frames(input_video_path, output_folder)
