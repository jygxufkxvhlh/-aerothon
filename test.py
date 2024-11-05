
import cv2
import os
import sys
import threading
import numpy as np
import time

PIC_ALT=4
DROP_ALT=3
FLY_ALT=5
# PIC_ALT=10
# DROP_ALT=20
# FLY_ALT=30
COUNT=0

COORD=None
DETECTION_CLASS=None
WIDTH=640
HEIGHT=480
CENT_X=WIDTH//2
CENT_Y=HEIGHT//2
webcam=cv2.VideoCapture('aerothon.mp4')
WIDTH=webcam.get(cv2.CAP_PROP_FRAME_WIDTH)
HEIGHT=webcam.get(cv2.CAP_PROP_FRAME_HEIGHT)

# CENT_X=WIDTH//2
# CENT_Y=HEIGHT//2
CENT_X=2520//2
CENT_Y=1080//2

MOVEMENT=1
MIN_DIST=10

vid_cam=True
cam_cam=False


def video():
    
    x_d=0.0
    y_d=0.0
    x_d_p=0.0
    y_d_p=0.0
    
    global vid_cam,cam_cam,COUNT,COORD,DETECTION_CLASS
    # webcam=cv2.VideoCapture('aerothon2.mp4')
    
    # webcam=cv2.VideoCapture(0,cv2.CAP_DSHOW)
    # webcam.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH)
    # webcam.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)
    # webcam.set(cv2.CAP_PROP_FPS,30)
    # webcam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
    # webcam.set(cv2.CAP_PROP_EXPOSURE,0.01)

	#Video Recording
    # dirpath = os.path.join(os.getcwd(),"Recordings")
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # out = cv2.VideoWriter(os.path.join(dirpath,time.strftime("%Y%m%d-%H%M%S")+".mp4"), fourcc, 30.0, (640,480))  
    
    while vid_cam:
        # timer=cv2.getTickCount()
        _,img=webcam.read()
        # img=cv2.flip(img,-1) 
        # fps=round(cv2.getTickFrequency()/(cv2.getTickCount()-timer))
        # cv2.putText(image,str(fps),(75,50),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,255),2) 
        
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
        orange_lower=np.array([154,102,156],np.uint8)
        orange_upper=np.array([172,125,222],np.uint8)
        blue_lower=np.array([26,144,144],np.uint8)
        blue_upper=np.array([255,255,255],np.uint8)  
        
        blue=cv2.inRange(hsv,blue_lower,blue_upper)
        orange=cv2.inRange(hsv,orange_lower,orange_upper)
	
        #Morphological transformation, Dilation  	
        kernal = np.ones((5 ,5), "uint8")


        blue=cv2.dilate(blue,kernal)
        orange=cv2.dilate(orange,kernal)


        img=cv2.circle(img,(CENT_X,CENT_Y),5,(255,0,0),-1)
        
        #Tracking the Blue Color
        (blue_contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        (orange_contours,hierarchy)=cv2.findContours(orange,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        if len(blue_contours)>0:
            contour= max(blue_contours,key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area>800: 
                x,y,w,h = cv2.boundingRect(contour)	
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                img=cv2.circle(img,((2*x+w)//2,(2*y+h)//2),5,(255,0,0),-1)
                img=cv2.line(img,(CENT_X,CENT_Y),((2*x+w)//2,(2*y+h)//2),(0,255,0),2)
            
                x_d= int(x+w/2)
                y_d= int(y+h/2) 
                COORD=[x_d,y_d]    
                DETECTION_CLASS="Target"
                
                
                s= 'x_d:'+ str(x_d)+ 'y_d:'+str(y_d)+' Target'
                
                cv2.putText(img,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)      

            
                if (abs(x_d-x_d_p)> 1 or abs(y_d-y_d_p)>1):
                
                    x_d_p=x_d
                    y_d_p=y_d
                
            
        
        if len(orange_contours)>0:
            contour= max(orange_contours,key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area>800: 
                x,y,w,h = cv2.boundingRect(contour)	
                img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                img=cv2.circle(img,((2*x+w)//2,(2*y+h)//2),5,(255,0,0),-1)
                img=cv2.line(img,(CENT_X,CENT_Y),((2*x+w)//2,(2*y+h)//2),(0,255,0),2)
            
                x_d= int(x+w/2)
                y_d= int(y+h/2)    
                COORD=[x_d,y_d]  
                DETECTION_CLASS="Hotspot"
                
                s= 'x_d:'+ str(x_d)+ 'y_d:'+str(y_d)+' Hotspot'
                
                cv2.putText(img,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)        

            
                if (abs(x_d-x_d_p)> 1 or abs(y_d-y_d_p)>1):
                
                    x_d_p=x_d
                    y_d_p=y_d
                    
        else:
            COORD=None
            DETECTION_CLASS=None
            
        
        cv2.imshow("Target",blue)
        cv2.imshow("Hotspot",orange)
        cv2.namedWindow('Tracking',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Tracking',1920,1080)
        # cv2.resize(img,(640,480))
        cv2.imshow("Tracking",img)
        key=cv2.waitKey(10)
        if key==27:
            break            
        # out.write(img)
        if cam_cam:
            image_filename = f'image_{COUNT}.jpg'
            savepath = os.path.join(dirpath, image_filename)
            cv2.imwrite(savepath, img)
            COUNT+=1
            cam_cam=False
    
    vid_cam=False
    out.release()
    webcam.release()
    cv2.destroyAllWindows()


camera_thread = threading.Thread(target=video)
camera_thread.start()


while True:    
    if not vid_cam:
        camera_thread.join()        
        sys.exit()
    
    
    if COORD !=None :
        #Switch to GUIDED mode
        # Mavcode.change_mode('GUIDED')
        print("Coords: ",COORD," ",DETECTION_CLASS)

            
        #Distance from taget to center
        dist=(((CENT_X-COORD[0])**2 +(CENT_Y-COORD[1])**2)**0.5)//4

        #Check the quadrant of the target
        X_POS=CENT_X<COORD[0]
        Y_POS=CENT_Y>COORD[1]

        
        #Try to bring the target in the center of camera
        while COORD!=None and dist>MIN_DIST:          
        
            print("Distance: ",dist)
            time.sleep(1)
            if (X_POS and Y_POS):
                # print("Decrease X\nDecrease Y")
                print("Quadrant: 1")
                # Mavcode.send_movement_command_XYZ(MOVEMENT,MOVEMENT,0)
            elif (X_POS and not Y_POS):
                #print("Decrease X\nIncrease Y")
                print("Quadrant: 2")
                # Mavcode.send_movement_command_XYZ(-MOVEMENT,MOVEMENT,0)
            elif (not X_POS and Y_POS):
                #print("Increase X\nDecrease Y")
                print("Quadrant: 3")
                # Mavcode.send_movement_command_XYZ(MOVEMENT,-MOVEMENT,0)
            elif (not X_POS and not Y_POS):
                # print("Increase X\nIncrease Y")
                print("Quadrant: 4")
                # Mavcode.send_movement_command_XYZ(-MOVEMENT,-MOVEMENT,0)
            
            try:
                #Distance from taget to center
                dist=(((CENT_X-COORD[0])**2 +(CENT_Y-COORD[1])**2)**0.5)//4

                #Check the quadrant of the target
                X_POS=CENT_X<COORD[0]
                Y_POS=CENT_Y>COORD[1]
            except:
                print("Change mode to AUTO")
                
        print("Doing task")

        
# import time

# for i in range(10,0,-1):
#     print("Detection starting in ",i)
#     time.sleep(1)

# print("Detection started")
