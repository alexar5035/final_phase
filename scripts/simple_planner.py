#!/usr/bin/env python3

import rospy
# transformation imports
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
# import sphere params
from robot_vision_lectures.msg import SphereParams 
# import plan, twist and bool messages
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

xraw = 0
yraw = 0
zraw = 0
radiusraw = 0 
recieved_sdata = False 
r_avail = False
ball_avail = False
current = [0.0]*6

# Adds points to plan
def new_point(linX, linY, linZ, angX, angY, angZ, mode, plan):
	point = Twist()
	gripper = UInt8()
		
	point.linear.x = linX
	point.linear.y = linY
	point.linear.z = linZ
	point.angular.x = angX
	point.angular.y = angY
	point.angular.z = angZ
	gripper.data = mode
		
	plan.points.append(point)
	plan.modes.append(gripper)

# Gets sphere raw data
def get_sData(data):	
	global xraw
	global yraw
	global zraw
	global radiusraw
	global recieved_sdata
	global ball_avail
	
	if not ball_avail:
		xraw = data.xc
		yraw = data.yc
		zraw = data.zc
		radiusraw = data.radius
	ball_avail = True 
	
def get_pos(data):
	global current
	global r_avail
	
	if not r_avail:
		current[0] = data.linear.x
		current[1] = data.linear.y
		current[2] = data.linear.z
		current[3] = data.angular.x
		current[4] = data.angular.y
		current[5] = data.angular.z
	r_avail = True

def rqt_listener(data):
	global rqt_toggle
	rqt_toggle = data.data

def pause_listener(data):
	global pause_toggle
	pause_toggle = data.data
		
if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# Subscriber for sphere parameters
	rospy.Subscriber('sphere_params', SphereParams, get_sData)
	# Publisher for sending joint positions
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# Subscriber to cancel plan
	rqt_toggle = rospy.Subscriber('/rqt_toggle', Bool, rqt_listener)
	# Subscriber to pause
	pause_toggle = rospy.Subscriber('/pause_toggle', Bool, pause_listener)
	rospy.Subscriber('/ur5e/toolpose', Twist, get_pos)
	# Set a 10Hz frequency
	loop_rate = rospy.Rate(10)
	# Check if a plan has been created 
	planned = False
	# Define plan
	plan = Plan()
	while not rospy.is_shutdown():
		# ros transform listener
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		if not planned and ball_avail and r_avail: 
			# get most recent transformation between the camera frame and the base frame
			try:
				trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print('Frames not available')
				loop_rate.sleep()
				continue
			# Define points in camera frame
			ptcam = tf2_geometry_msgs.PointStamped()
			ptcam.header.frame_id = 'camera_color_optical_frame'
			ptcam.header.stamp = rospy.get_rostime()

			ptcam.point.x = xraw
			ptcam.point.y = yraw
			ptcam.point.z = zraw

			# Convert points to base frame
			ptbase = tfBuffer.transform(ptcam,'base', rospy.Duration(1.0))
			x = ptbase.point.x
			y = ptbase.point.y
			z = ptbase.point.z
			radius = radiusraw

			# Print coor before and after transform 
			print("Before tranformed: \n", "x: ", xraw, "y: ", yraw, "z: ", zraw, "radius: ", radiusraw, "\n")
			print("Transformed: \n", "x: ", x, "y: ", y, "z: ", z, "radius: ", radius, "\n")

			# angular values stay the same
			# roll
			x_ang = current[3]
			# pitch
			y_ang = current[4]
			# yaw
			z_ang = current[5]
			
			opened = 1
			closed = 2
			stayed = 0
			y_off = -0.01
			# First point: starting position 
			new_point(current[0], current[1], current[2], x_ang, y_ang, z_ang, stayed, plan)
			new_point(x, y+y_off, z+.1, x_ang, y_ang, z_ang, stayed, plan)
			# Second point: grab the ball
			new_point(x, y+y_off, z+.02, x_ang, y_ang, z_ang, stayed, plan)
			new_point(x, y+y_off, z+.02, x_ang, y_ang, z_ang, closed, plan)
			new_point(x, y+y_off, z+.1, x_ang, y_ang, z_ang, stayed, plan)
			# Third point: up and to the right
			new_point(0.3, -0.35, 0.3, x_ang, y_ang, z_ang, plan)
			# Fourth point: Decrease z to drop ball 
			new_point(0.3, -0.35, z+radius, x_ang, y_ang, z_ang, stayed, plan)
			new_point(0.3, -.35, z+.02, x_ang, y_ang, z_ang, stayed, plan)
			# Fifth point: decrease z, drop ball
			new_point(0.3, -0.35, 0.3, x_ang, y_ang, z_ang, opened, plan)
			# Sixth point: drop to stay
			new_point(-0.014, -0.35, 0.3, x_ang, y_ang, z_ang, stayed, plan)
			planned = True
			
		# publish the plan
		plan_pub.publish(plan)
		
		loop_rate.sleep()
		
