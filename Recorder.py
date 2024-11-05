import cv2
import os
import time
# import Mavcode

# Telemetry_thread = threading.Thread(target=Mavcode.Telemetry)
# Telemetry_thread.start()

# Initialize video capture
cap = cv2.VideoCapture(0)  # 0 for default camera, or provide the camera's index
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
cap.set(cv2.CAP_PROP_FPS,30)
dirpath = os.path.join(os.getcwd(),"Recordings")
# Set up the video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(os.path.join(dirpath, time.strftime("%Y%m%d-%H%M%S")+".mp4"), fourcc, 30.0, (960, 540))  # Adjust the resolution and frame rate as needed

# Initialize variables
image_counter = 0



while True:
    # Capture video frame-by-frame
    ret, frame = cap.read()
    frame=cv2.flip(frame,-1)
    # height, width = frame.shape[:2] 
  
# Drawing the lines 
    cv2.line(frame,(960,500),(960,580),(0,0,255),3)
    cv2.line(frame,(920,540),(1000,540),(0,0,255),3)
    frame= cv2.resize(frame, (960, 540))   

    # Display the frame
    cv2.namedWindow('MPSTME AeroXperts', cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('MPSTME AeroXperts', 960,540)
    cv2.imshow('MPSTME AeroXperts', frame)

    # Check if a key is pressed
    key = cv2.waitKey(10)

    if key == ord('q'):  # Press 'q' to quit
        break
    elif key == ord('s'):  # Press 's' to save an image
        image_filename = f'image_{image_counter}.jpg'
        savepath = os.path.join(dirpath, image_filename)
        cv2.imwrite(savepath, frame)
        print(f'Saved {savepath}')
        image_counter += 1

    # Write the frame to the video
    out.write(frame)

# Release the video capture and writer
cap.release()
out.release()
# Telemetry_thread.join()   

# Close all OpenCV windows
cv2.destroyAllWindows()
