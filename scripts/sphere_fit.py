#!/usr/bin/env python3
# from lab 6
import rospy
import numpy as np
import math
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from std_msgs.msg import Bool

# initial matrices
a_matrix = []
b_matrix = []
recieved = False

def receive(data):
	global a_matrix
	global b_matrix
	global recieved
	
	a_matrix = []
	b_matrix = []
	
	# loop through point_data to create A and B matrices
	# A = 2Xn, 2Yn, 2Zn, 1
	# B = Xn^2 + Yn^2 + Zn^2
	for point in data.points:
			a_matrix.append([2*point.x, 2*point.y, 2*point.z, 1])
			b_matrix.append([point.x**2 + point.y**2 + point.z**2])
	recieved = True
			

def filter(data, fil_out, fil_gain):
	#initialize
	fil_in = data
	# filter equation
	fil_out = (fil_gain*fil_in) + (1-fil_gain)*fil_out
	return fil_out
	
tracking = None
def track(data):
	global tracking
	tracking = data.data
	
if __name__ == '__main__':
	# ball detection node initialized
	rospy.init_node('sphere_fit', anonymous = True)
	# subscriber for point_data
	sp_data = rospy.Subscriber("/xyz_cropped_ball", XYZarray, receive)
	#subscriber for track
	rospy.Subscriber('/TrackBall', Bool, track)
	# publisher fot sphere paramaters
	sp_pub = rospy.Publisher("/sphere_params", SphereParams, queue_size = 10)
	# loop rate
	rate = rospy.Rate(10)
	
	# initialize variables
	fil_outx = 0.0
	fil_outy = 0.0
	fil_outz = 0.0
	fil_outr = 0.0
	
	pt_gain = 0.0005
	r_gain = 0.005
	
	delay = 0
	first_fil = True
	off_print = True
	
	# set fil gain
	#fil_gain = 0.05 # how much of the most recent input is included 
	
	while not rospy.is_shutdown():
		if recieved:
			A = np.array(a_matrix)
			B = np.array(b_matrix)
			if A.shape[0] == B.shape[0] and len(A.shape) == 2 and len(B.shape) == 2:
				# calc P
				P = np.linalg.lstsq(A, B, rcond = None)[0]
				# get the center sphere params for P
				xc = P[0]
				yc = P[1]
				zc = P[2]
				# radius calculation: SQRT(P[3] + Xc^2 + Yc^2 + Zc^2)
				r = math.sqrt(P[3] + xc**2 + yc**2 + zc**2)
				
				if tracking:
					# shows msg if filter is disabled
					off_print = True 
					# if it is the first time filtering
					if first_fil:
						fil_outx = xc
						fil_outy = yc
						fil_outz = zc
						fil_outr = r
						print("ball filter is enabled")
						# set bool to false since a filtering is happening
						first_fil = False
					# else if a filter has already occured
					else:
						# recieve the filtered data
						xc = filter(xc, fil_outx, pt_gain)
						fil_outx = xc
						yc = filter(yc, fil_outy, pt_gain)
						fil_outy = yc
						zc = filter(zc, fil_outz, pt_gain)
						fil_outz = zc
						r = filter(r, fil_outr, r_gain)
						fil_outr = r
				# else if not tracking
				else:
					if off_print: 
						print("ball filter is disabled")
						off_print = False
					first_fil = True
				sp_data = SphereParams()
				# add sphere params to publisher
				sp_data.xc = xc
				sp_data.yc = yc
				sp_data.zc = zc
				sp_data.radius = r		
				# publish sphere params
				sp_pub.publish(sp_data)		
		rate.sleep()

