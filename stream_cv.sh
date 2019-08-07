#gst-launch-1.0 -v udpsrc port=5000 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! autovideosink
gst-launch-1.0 udpsrc port=5000 ! application/x-rtp ! rtph264depay ! avdec_h264 ! autovideosink sync=false

