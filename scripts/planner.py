#!/usr/bin/env python3
# from phase 1
import rospy
import math

# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# import sphere parameters
from robot_vision_lectures.msg import SphereParams

# transformation imports
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs

# pre-transformation sphere data
xraw = 0
yraw = 0
zraw = 0
radius = 0

def get_sData(data):
	global xraw
	global yraw
	global zraw
	global radius
	
	#center of sphere data
	xraw = data.xc
	yraw = data.yc
	zraw = data.zc
	radius = data.radius
	
def coordinates(xraw, yraw, zraw, radius):
	# add a ros listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.tfListener(tfBuffer)
	
	# 10Hz freq
	loop_rate = rospy.Rate(10)
	
	flag = False
	
	qrot = Quaternion()
	
	while not flag:
		# try 
		try:
			trans = tfBuffer.lookup_transformation("base", "camera_color_optical_frame", rospy.Time())
			flag = True
		except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print("frames unavailable")
			loop_rate.Sleep()
			continue
			
	# points in the camera frame
	pcam = tf2_geometry_msgs.PointStamped()
	pcam.header.frame_id = "camera_color_optical_frame"
	pcam.header.stamp = rospy.get_rostime()

	pcam.point.x = xraw
	pcam.point.y = yraw
	pcam.point.z = zraw

	# convert to base frame coords
	pbase = tfBuffer.transform(pcam, 'base', rospy.Duration(1.0))

	# coords from new frame
	x = pbase.point.x
	y = pbase.point.y
	z = pbase.point.z

	return x, y, z, radius

# defines a new point
def new_point(plan, x_lin, y_lin, z_lin, x_ang, y_ang, z_ang):
	plan_point = Twist()
	# linear points
	plan_point.linear.x = x_lin
	plan_point.linear.y = y_lin
	plan_point.linear.z = z_lin
	# angular points
	plan_point.angular.x = x_ang
	plan_point.angular.y = x_ang
	plan_point.angular.z = x_ang
	
	plan_points.append(plan_point)
	
	
# gets current robot position

# initialize current position to all 0.0
current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
def get_pos(data):
	global current
	current[0] = data.linear.x
	current[1] = data.linear.y
	current[2] = data.linear.z
	current[3] = data.angular.x
	current[4] = data.angular.y
	current[5] = data.angular.z
	
# enable or disable ball plan
planning = False

def plan(data):
	global planning
	planning = data.data

# check if the ball is being tracked
tracking = False

def track(data):
	global tracking
	tracking = data.data


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('planner', anonymous = True)
	
	# add a subscriber for SpherePrams
	rospy.Subscriber('sphere_params', SphereParams, get_sData)
	
	# add a subscriber for current position
	rospy.Subscriber('/ur5e/toolpose', Twist, get_pos)
	
	# add a subscriber for plan
	rospy.Subscriber('/BallPlan', Bool, plan)
	
	# add a subscriber for tracking
	rospy.Subscriber('/TrackBall', Bool, track)
	
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	delay = 0
	
	while not rospy.is_shutdown():
	
		if planning and tracking:
			print("\nGenerating new plan")
			valid = False
			while not valid:
				if 0 not in [xraw, yraw, zraw, radius]:
					valid = True
				updated_coords = coordinates(xraw, yraw, zraw, radius)
			print("\nBall values before transformation")
			print("x: {}, y: {}, z:{}, radius: {}".format(xraw, yraw, zraw, radius))
			# define the new/updated coordinates
			x = updated_coords[0]
			y = updated_coords[1]
			z = updated_coords[2]
			r = updated_coords[3]
			
			print("\nBall values after transformation")
			print("x: {}, y: {}, z:{}, radius: {}".format(x, y, z, r))
			
			# new plan
			motion = Plan() 
		
			# angular values stay the same (current[3], current[4], current[5])
			# first point : initial position
			new_point(motion, current[0], current[1], current[2], current[3], current[4], current[5])
		
			# second point :  over ball
			new_point(motion, x, y, current[3], current[4], current[5])
		
			# third point : pick up ball
			new_point(motion, x, y, (z)+r, current[3], current[4], current[5])
		
			# fourth point : lift up ball
			new_point(motion, x, y, 0.3, current[3], current[4], current[5])
		
			# fifth point : move to drop location
			new_point(motion, 0.3, -0.35, 0.3, current[3], current[4], current[5])
		
			# sixth point : drop ball
			new_point(motion, 0.3, -0.35, (z)+r, current[3], current[4], current[5])
		
			# seventh point : move back up
			new_point(motion, 0.3, -0.35, 0.3, current[3], current[4], current[5])
		
			# stop publishing plan when tracking is disabled
			while planning:
				# publish the plan
				plan_pub.publish(motion)
			
				# wait for 0.1 seconds until the next loop and repeat
				loop_rate.sleep()
		else:
			if delay == 0:
				if not planning:
					print("motion planning disabled.")
				if not tracking:
					print("motion tracking disabled.")
				delay = 10
			else:
				# set a less overwhelming stream of messages
				delay -= 1
				loop_rate.sleep()	
			
