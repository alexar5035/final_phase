#!/usr/bin/env python3
# from lab 4
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")

# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# raise flag
	img_received = True
	
# processes the image via techniques shown in class using HSV color space and produces a mono-color image. Labels the pixels corresponding to the ball with a value equal to 255 and other pixels with 0.	
def process(image):
	# RGB to HSV
	hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
	# HSV boundaries:
	# lower yellow
	l_yellow = np.array([10,5,100])
	# upper yellow 
	u_yellow = np.array([40, 255, 255])
	# mask using boundaries
	y_mask = cv2.inRange(hsv, l_yellow, u_yellow)
	# make the circle to filter out extra noise in surrounding area
	circle = np.zeros((720,1280), dtype="uint8")
	cv2.circle(circle, (535,300), 150, 255, -1)
	# overlay filter on masked_img
	filtered_img = cv2.bitwise_and(y_mask, circle)
	return filtered_img
	

if __name__ == '__main__':
	# define the node subscribers and publishers
	rospy.init_node('detect_ball', anonymous = True)
	# subscribes to the correct topic and recieves image frames from the recorded bag file (same as the real camera)
	
	# define a subscriber 
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	
	# define a publisher
	img_pub = rospy.Publisher("/ball_2D", Image, queue_size = 1)
	
	# set loop frequency
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		# processing the image streamed
		if img_received:
			image1 = process(rgb_img)
			# convert to ros msg and publish
			img_msg = CvBridge().cv2_to_imgmsg(image1, encoding="mono8")
			#publish the image
			img_pub.publish(img_msg)
		# pause until next iteration
		rate.sleep()
