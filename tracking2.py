# import the necessary packages

import time,sys,os
import cv2

import numpy as np
import struct
import socket
import random

calibrated = False
lower_green=np.array([0,0,0])
upper_green=np.array([180,125,80])

# initialize the camera
camid = str(sys.argv[1])
cam = cv2.VideoCapture(int(camid))
print("init camera on /dev/video"+camid)
#os.system('v4l2-ctl --set-ctrl=exposure_auto=3 -d /dev/video'+camid)

xdim = 320
ydim = 240

cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cam.set(cv2.CAP_PROP_EXPOSURE,.001);
cam.set(cv2.CAP_PROP_FRAME_WIDTH,xdim);
cam.set(cv2.CAP_PROP_FRAME_HEIGHT,ydim);

# # set up network socket/addresses
# host = 'localhost'
# Lport = 4000
# Rport = 5000
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# sock.bind(("", Lport))
# print ("Active on port: " + str(Lport))
# robot_address = (host, Rport)

buffer = 40 # color range buffer
xtarget = 0

kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))
# allow the camera to warmup



def SendToRobot(left, right, error, P, I, D):
    global sock
    data = str(left)+";"+str(right)+";"+str(error)+";"+str(P)+";"+str(I)+";"+str(D)
    send_msg = str(str(data)).encode()
    try:
          sock.sendto(send_msg, robot_address)
          #print send_msg
    except Exception as e:
          print("FAIL - RECONNECT.." + str(e.args))
          try:
                  print("sending " + send_msg)
                  sock.sendto(send_msg, robot_address)
          except:
                  print("FAILED.....Giving up :-(")

def FindColor(lower_col, upper_col, min_area):
    global imgHSV
    # find the colored regions
    mask=cv2.inRange(imgHSV,lower_col,upper_col)
#    cv2.imshow("mask",mask)

    # this removes noise by eroding and filling in
    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
    a, conts, h = cv2.findContours(maskClose, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # Finding bigest  area and save the contour
    targets = []
    for cont in conts:
        area = cv2.contourArea(cont)
        if area > min_area:
            targets.append(cont)
    return targets



def on_mouse_click (event, x, y, flags, frame):
    global calibrated,lower_green,upper_green
    if event == cv2.EVENT_LBUTTONUP:
        print(frame[y,x].tolist())
        lower_green=np.array([frame[y,x].tolist()[0]-buffer,frame[y,x].tolist()[1]-buffer,frame[y,x].tolist()[2]-buffer])
        upper_green=np.array([frame[y,x].tolist()[0]+buffer,frame[y,x].tolist()[1]+buffer,frame[y,x].tolist()[2]+buffer])
        calibrated = True


while True:
        ret, full_img = cam.read()
        hsv = cv2.cvtColor(full_img,cv2.COLOR_BGR2HSV)
        cv2.putText(full_img, str("CLICK ON target"), (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (20, 255, 255), 2)
        cv2.imshow('frame', full_img)
        cv2.setMouseCallback('frame', on_mouse_click, hsv)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if calibrated:
            break

cv2.destroyAllWindows()

while True:

    cv2.imshow("robotimgPi", full_img)

    ret, full_img=cam.read()
    imgHSV = cv2.cvtColor(full_img,cv2.COLOR_BGR2HSV)
    key = cv2.waitKey(1) & 0xFF

    targets = FindColor(lower_green, upper_green, 70)

    slopes = []
    yints = []
    heights = []
    xpos = []

    for cont in targets:
        # create a rectangle to represent the line and find
        # the angle of the rectangle on the screen.
        greenbox = cv2.minAreaRect(cont)

        rows,cols = full_img.shape[:2]
        [vx,vy,x,y] = cv2.fitLine(cont, cv2.DIST_L2,0,0.01,0.01)
        lefty = int((-x*vy/vx) + y)
        righty = int(((cols-x)*vy/vx)+y)

        slope = float(righty-lefty)/((cols-1)-0)
        slopes.append(slope)
        yints.append(lefty)

        try:
            cv2.line(full_img,(cols-1,righty),(0,lefty),(255,0,0),2)
        except:
            pass


        (x_min, y_min), (w_min, h_min), lineang = greenbox
        drawgreenbox = cv2.boxPoints(greenbox)
        drawgreenbox = np.int0(drawgreenbox)
        cv2.drawContours(full_img,[drawgreenbox],-1,(0,255,0),3)

        # draw line with the estimate of location and angle
        cv2.line(full_img, (int(x_min),int(y_min)), (xdim/2,ydim), (200,0,200),2)
        cv2.circle(full_img,(int(x_min),int(y_min)),3,(200,0,200),-1)

        heights.append(y_min)
        xpos.append(x_min)


    cnt = 0
    while cnt < len(slopes):
        cnt2 = 0
        while cnt2 < len(slopes):
            if slopes[cnt2]-slopes[cnt] !=0:
                dx = (yints[cnt]-yints[cnt2])/(slopes[cnt2]-slopes[cnt])
                dy = slopes[cnt2] * dx + yints[cnt2]
                if dy < heights[cnt] and dy < heights[cnt2]:
                    cv2.circle(full_img,(int(dx),int(dy)),5,(200,200,200),-1)
                    cv2.circle(full_img,(int((xpos[cnt]+xpos[cnt2])/2),int((heights[cnt]+heights[cnt2])/2.0)),5,(200,200,200),-1)
                    cv2.line(full_img, (int((xpos[cnt]+xpos[cnt2])/2),int((heights[cnt]+heights[cnt2])/2.0)), (xdim/2,ydim), (200,200,200),2)
                    xtarget = (xpos[cnt]+xpos[cnt2])/2 - xdim/2
                else:
                    cv2.circle(full_img,(int(dx),int(dy)),5,(200,200,0),-1)
            if len(slopes) == 1:
                cv2.circle(full_img,(int(xpos[cnt]),int((heights[cnt]))),5,(200,200,200),-1)
                cv2.line(full_img, (int(xpos[cnt]),int(heights[cnt])), (xdim/2,ydim), (200,200,200),2)
                xtarget = xpos[cnt] - xdim/2
            cnt2 = cnt2 + 1
        cnt = cnt + 1
    cv2.putText(full_img, "dx="+str(xtarget), (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (20, 255, 255), 2)

    print("dx="+str(xtarget))




    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
    	break
