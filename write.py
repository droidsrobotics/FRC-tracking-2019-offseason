import cv2

# Define the codec and create VideoWriter object

#works
#cap_send = cv2.VideoCapture('v4l2src device=/dev/video2 ! video/x-raw,framerate=20/1,width=320,height=240 ! videoscale ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

cap_send = cv2.VideoCapture('v4l2src device=/dev/video2 ! video/x-raw,framerate=30/1,width=320,height=240 ! videoscale ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

#cap_send = cv2.VideoCapture('v4l2src device=/dev/video2 ! video/x-raw,width=640,height=480,framerate=30/1 ! appsink',cv2.CAP_GSTREAMER);


#out_send = cv2.VideoWriter('appsrc ! videoconvert ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=5000', cv2.CAP_GSTREAMER,0, 20, (320,240), True)

# works
#out_send = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=192.168.1.17 port=5000',cv2.CAP_GSTREAMER,0, 20, (320,240), True)

#out_send = cv2.VideoWriter('appsrc ! videoconvert ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.17 port=5000',cv2.CAP_GSTREAMER,0, 20, (320,240), True)

out_send = cv2.VideoWriter('appsrc ! videoconvert ! video/x-raw,format=YUY2,width=320,height=240,framerate=30/1 ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.17 port=5000',cv2.CAP_GSTREAMER,0,30,(320,240),True);


if not cap_send.isOpened() or not out_send.isOpened():
    print('VideoCapture or VideoWriter not opened')
    exit(0)
        
while True:
    ret, frame = cap_send.read()

    if not ret:
        print('empty frame')
        break

    out_send.write(frame)

    cv2.imshow('send', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything if job is finished
cap_send.release()
out_send.release()
cv2.destroyAllWindows()
