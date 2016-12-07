#detector.py
#Siddharth S Jha

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import rospy 
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

RED_THRES=230

time.sleep(0.1)
 
# allow the camera to warmup
if __name__ == '__main__':
	rospy.init_node('vision')
    rate = rospy.Rate(15)
	 
	pub = rospy.Publisher('angles', Point32, queue_size=100)
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# grab the raw NumPy array representing the image, then initialize the timestamp
		# and occupied/unoccupied text
		image = frame.array
	 
		key = cv2.waitKey(1) & 0xFF
	 
		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)
		
		# WRITE ANGLE DETECTION CODE HERE #

		numred = (frame[...,...,2] > RED_THRES) 
		xy_val =  numred.nonzero()
		y_val = math.floor(median(xy_val[0]))
		x_val = math.floor(median(xy_val[1]))

		#ANGLE CALCULATION FOR RED BODY

		numgreen = (frame[...,...,1] > GREEN_THRES) 
		xy_val =  numgreen.nonzero()
		y_val = math.floor(median(xy_val[0]))
		x_val = math.floor(median(xy_val[1]))

		#ANGLE CALCULATION FOR GREEN BODY

		numblue = (frame[...,...,0] > BLUE_THRES) 
		xy_val =  numblue.nonzero()
		y_val = math.floor(median(xy_val[0]))
		x_val = math.floor(median(xy_val[1]))

		#ANGLE CALCULATION FOR BLUE BODY

		angles = Point32(beta1,beta2,beta3)
	    pub.publish(angles)
	 
		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			break


