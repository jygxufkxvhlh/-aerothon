import cv2
import numpy as np

# Load the image
# cam=cv2.VideoCapture(0)

cam= cv2.VideoCapture('test1.mp4')
while True:
        ret,image=cam.read()
        # image = cv2.imread('t01.jpg')
        image=cv2.resize(image,(640,480))
        imaged=cv2.flip(image,-1)

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply adaptive thresholding
        thresholded = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 13, 2)

        # Invert the thresholded image
        inverted = cv2.bitwise_not(thresholded)

        # Find and extract blobs in the inverted image
        contours, _ = cv2.findContours(inverted, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Define a list to store filtered blobs
        filtered_blobs = []

        for contour in contours:
            # Calculate the center of gravity (COG) of each blob
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            # Filter blobs based on their COG proximity
            # You can adjust the threshold based on your requirements
            proximity_threshold = 100  # Adjust this threshold as needed
            close_to_another_blob = False

            for blob in filtered_blobs:
                if np.sqrt((cX - blob[0]) ** 2 + (cY - blob[1]) ** 2) < proximity_threshold:
                    close_to_another_blob = True
                    break

            if close_to_another_blob:
                continue

            # Additional filtering criteria (e.g., size filtering)
            blob_area = cv2.contourArea(contour)
            if blob_area < 80:  # Adjust the minimum area as needed
                continue

            # Check if the blob is inside another blob
            inside_another_blob = False

            for inner_blob in filtered_blobs:
                if cv2.pointPolygonTest(inner_blob[2], (cX, cY), False) > 0:
                    inside_another_blob = True
                    break

            if inside_another_blob:
                continue

            # Add the blob to the list of filtered blobs
            filtered_blobs.append((cX, cY, contour))

        # Draw the filtered blobs on the original image
        for (cX, cY, contour) in filtered_blobs:
            cv2.drawContours(image, [contour], -1, (0, 0,255), 2)

        # Display the result
        cv2.imshow("Filtered Image", image)
        cv2.imshow("Inverted Image", inverted)
        key=cv2.waitKey(10)
        if key==27:
            break
cv2.destroyAllWindows()
