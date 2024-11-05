#!/usr/bin/env python
import cv2
import numpy as np

def nothing(pos):
	pass

cap=cv2.VideoCapture(0,cv2.CAP_DSHOW)
# cap=cv2.VideoCapture('test2.mp4')
x_o= 320
y_o= 240
x_d=0.0
y_d=0.0
x_d_p=0.0
y_d_p=0.0

while(1):
	_, img = cap.read()
	img=cv2.resize(img,(640,480))
	    
	#converting frame(img i.e BGR) to HSV (hue-saturation-value)
	hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
 
 
	orange_lower=np.array([5,69,173],np.uint8)
	orange_upper=np.array([31,250,238],np.uint8)
	blue_lower=np.array([113,28,161],np.uint8)
	blue_upper=np.array([141,75,255],np.uint8)
	# blue_lower=np.array([26,144,144],np.uint8)
	# blue_upper=np.array([255,255,255],np.uint8)


	blue=cv2.inRange(hsv,blue_lower,blue_upper)
	orange=cv2.inRange(hsv,orange_lower,orange_upper)
	
	#Morphological transformation, Dilation  	
	kernal = np.ones((5 ,5), "uint8")


	blue=cv2.dilate(blue,kernal)
	orange=cv2.dilate(orange,kernal)

	img=cv2.circle(img,(x_o,y_o),5,(255,0,0),-1)

			
	#Tracking the Blue Color
	(blue_contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	if len(blue_contours)>0:
		contour= max(blue_contours,key=cv2.contourArea)
		area = cv2.contourArea(contour)
		if area>800: 
			x,y,w,h = cv2.boundingRect(contour)	
			img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
			img=cv2.circle(img,((2*x+w)//2,(2*y+h)//2),5,(255,0,0),-1)
			img=cv2.line(img,(x_o,y_o),((2*x+w)//2,(2*y+h)//2),(0,255,0),2)
		
			x_d= int((x+w)/2)
			y_d= int((y+h)/2)    
			
			s= 'x_d:'+ str(x_d)+ 'y_d:'+str(y_d)+' Target'
			
			cv2.putText(img,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)
		

		
			if (abs(x_d-x_d_p)> 1 or abs(y_d-y_d_p)>1):
			
				x_d_p=x_d
				y_d_p=y_d
			
	(orange_contours,hierarchy)=cv2.findContours(orange,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	if len(orange_contours)>0:
		contour= max(orange_contours,key=cv2.contourArea)
		area = cv2.contourArea(contour)
		if area>800: 
			x,y,w,h = cv2.boundingRect(contour)	
			img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
			img=cv2.circle(img,((2*x+w)//2,(2*y+h)//2),5,(255,0,0),-1)
			img=cv2.line(img,(x_o,y_o),((2*x+w)//2,(2*y+h)//2),(0,255,0),2)
		
			x_d= int((x+w)/2)
			y_d= int((y+h)/2)    
			
			s= 'x_d:'+ str(x_d)+ 'y_d:'+str(y_d)+' Hotspot'
			
			cv2.putText(img,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)
		

		
			if (abs(x_d-x_d_p)> 1 or abs(y_d-y_d_p)>1):
			
				x_d_p=x_d
				y_d_p=y_d
			
		
	
	cv2.imshow("Target Mask",blue)
	cv2.imshow("Hotspot Mask",orange)
	cv2.imshow("Color Tracking",img)
	if cv2.waitKey(10)== ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
