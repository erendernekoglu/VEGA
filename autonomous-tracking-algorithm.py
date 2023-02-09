import cv2
import numpy as np

# Define the tracking algorithm
def track_uav(frame, roi):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Define the HSV color range for the UAV
    lower_hsv = np.array([0, 0, 0])
    upper_hsv = np.array([180, 255, 80])
    
    # Threshold the frame to extract the UAV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    
    # Use the CAMShift algorithm to track the UAV
    track_window = cv2.CamShift(mask, roi, term_crit)
    center, roi = cv2.meanShift(mask, roi, term_crit)
    
    # Draw a rectangle around the UAV
    x, y, w, h = track_window
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
    # Return the updated frame
    return frame

# Load the video
video = cv2.VideoCapture('uav_video.avi')

# Define the termination criteria for the CAMShift algorithm
term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

# Select the initial ROI for the UAV
_, frame = video.read()
roi = cv2.selectROI(frame, False)

# Process each frame of the video
while True:
    # Read the next frame
    _, frame = video.read()
    
    # Break the loop if there are no more frames
    if frame is None:
        break
    
    # Track the UAV in the current frame
    frame = track_uav(frame, roi)
    
    # Display the updated frame
    cv2.imshow('UAV Tracking', frame)
    
    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video and destroy all windows
video.release()
cv2.destroyAllWindows()
