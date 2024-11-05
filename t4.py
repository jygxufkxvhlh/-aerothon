import cv2

# Load the image
# image = cv2.imread('bullseye_image.jpg')
cam= cv2.VideoCapture('test2.mp4')
# cam=cv2.VideoCapture(0)

while True:
    ret,image=cam.read()
    image=cv2.resize(image,(640,480))
    image=cv2.flip(image,-1)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply adaptive thresholding to highlight the target
    thresholded = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    # Invert the thresholded image
    inverted = cv2.bitwise_not(thresholded)

    # Find and extract blobs in the inverted image
    contours, _ = cv2.findContours(inverted, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Define a list to store filtered blobs
    filtered_blobs = []

    for contour in contours:
        # Calculate the area of each blob
        blob_area = cv2.contourArea(contour)

        # Define a size threshold to filter out small objects
        if 1000 < blob_area < 2000:  # Adjust these values as needed
            filtered_blobs.append(contour)

    # Draw the filtered blobs on the original image
    cv2.drawContours(image, filtered_blobs, -1, (0, 255, 0), 2)

    # Display the result
    cv2.imshow("Filtered Image", image)
    key=cv2.waitKey(10)
    if key==27:
        break
cv2.destroyAllWindows()
