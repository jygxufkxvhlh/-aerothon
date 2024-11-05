#!/usr/bin/env python
import cv2
import numpy as np

def nothing(pos):
    pass

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
# cap = cv2.VideoCapture('test2.mp4')
x_o = 320
y_o = 240
x_d_p = 0.0
y_d_p = 0.0

while True:
    _, img = cap.read()
    img = cv2.resize(img, (640, 480))

    # Converting frame (BGR) to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the lower and upper HSV ranges for both colors
    orange_lower = np.array([5, 100, 100], np.uint8)
    orange_upper = np.array([15, 255, 255], np.uint8)
    blue_lower = np.array([95, 100, 100], np.uint8)
    blue_upper = np.array([105, 255, 255], np.uint8)

    # Create masks for both colors
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    # Morphological transformation, Dilation
    kernel = np.ones((5, 5), "uint8")
    orange_mask = cv2.dilate(orange_mask, kernel)
    blue_mask = cv2.dilate(blue_mask, kernel)

    img = cv2.circle(img, (x_o, y_o), 5, (255, 0, 0), -1)

    # Tracking the Orange Color
    orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(orange_contours) > 0:
        orange_contour = max(orange_contours, key=cv2.contourArea)
        area = cv2.contourArea(orange_contour)
        if area > 800:
            x, y, w, h = cv2.boundingRect(orange_contour)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            img = cv2.circle(img, ((2 * x + w) // 2, (2 * y + h) // 2), 5, (255, 0, 0), -1)
            img = cv2.line(img, (x_o, y_o), ((2 * x + w) // 2, (2 * y + h) // 2), (0, 255, 0), 2)

            x_d = int((x + w) / 2)
            y_d = int((y + h) / 2)

            s = 'Orange Color Detected at x:' + str(x_d) + ' y:' + str(y_d)
            cv2.putText(img, s, (x - 20, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            if (abs(x_d - x_d_p) > 1 or abs(y_d - y_d_p) > 1):
                x_d_p = x_d
                y_d_p = y_d

    # Tracking the Blue Color
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(blue_contours) > 0:
        blue_contour = max(blue_contours, key=cv2.contourArea)
        area = cv2.contourArea(blue_contour)
        if area > 800:
            x, y, w, h = cv2.boundingRect(blue_contour)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            img = cv2.circle(img, ((2 * x + w) // 2, (2 * y + h) // 2), 5, (0, 0, 255), -1)
            img = cv2.line(img, (x_o, y_o), ((2 * x + w) // 2, (2 * y + h) // 2), (0, 255, 0), 2)

            x_d = int((x + w) / 2)
            y_d = int((y + h) / 2)

            s = 'Blue Color Detected at x:' + str(x_d) + ' y:' + str(y_d)
            cv2.putText(img, s, (x - 20, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            if (abs(x_d - x_d_p) > 1 or abs(y_d - y_d_p) > 1):
                x_d_p = x_d
                y_d_p = y_d

    cv2.imshow("Orange Mask", orange_mask)
    cv2.imshow("Blue Mask", blue_mask)
    cv2.imshow("Color Tracking", img)

    if cv2.waitKey(10) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()