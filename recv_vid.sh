#gst-launch-1.0 udpsrc port=5000 ! application/x-rtp ! rtph264depay ! avdec_h264 ! autovideosink sync=false
gst-launch-1.0 udpsrc port=6666 ! application/x-rtp ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
