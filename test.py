# import the necessary packages

import time,sys,os
import cv2

# initialize the camera
camid = "2"
cam = cv2.VideoCapture(int(camid))
#print("init camera on /dev/video"+camid)
#os.system('v4l2-ctl --set-ctrl=exposure_auto=3 -d /dev/video'+camid)
cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
cam.set(cv2.CAP_PROP_FRAME_WIDTH,320);
cam.set(cv2.CAP_PROP_FRAME_HEIGHT,240);

while True:
        #cam.set(cv2.CAP_PROP_EXPOSURE,.001);
        ret, cap_img=cam.read()
        cv2.imshow('dark', cap_img)
        #cam.set(cv2.CAP_PROP_EXPOSURE,.03);
        #ret, cap2_img=cam.read()
        #cv2.imshow('bright', cap2_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
