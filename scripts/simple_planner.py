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

xraw = 0
yraw = 0
zraw = 0
radiusraw = 0 
recieved_sdata = False 

# Adds points to plan
def new_point(linX, linY, linZ, angX, angY, angZ, plan):
	point = Twist()
		
	point.linear.x = linX
	point.linear.y = linY
	point.linear.z = linZ
	point.angular.x = angX
	point.angular.y = angY
	point.angular.z = angZ
		
	plan.points.append(point)

# Gets sphere raw data
def get_sData(data):	
	global xraw
	global yraw
	global zraw
	global radiusraw
	
	xraw = data.xc
	yraw = data.yc
	zraw = data.zc
	radiusraw = data.radius
	recieved_sdata = True 
	
if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# Subscriber for sphere parameters
	rospy.Subscriber('sphere_params', SphereParams, get_sData)
	# Publisher for sending joint positions
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# Set a 10Hz frequency
	loop_rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		# add a ros transform listener
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		if True: 
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

			# Define plan
			plan = Plan()
			# angular values stay the same
			x_ang = 3.14
			y_ang = 0.0
			z_ang = 1.57
			# Starting position 
			new_point(0.3, -0.35, 0.3, x_ang, y_ang, z_ang, plan)
			# Position with x, y, z + radius
			new_point(x, y, z+radius, x_ang, y_ang, z_ang, plan)
			# Turn right 
			new_point(0.3, -0.35, 0.3, x_ang, y_ang, z_ang, plan)
			# Decrease z to drop ball 
			new_point(0.3, -0.35, z+radius, x_ang, y_ang, z_ang, plan)
			# Back to Start
			new_point(0.3, -0.35, 0.3, x_ang, y_ang, z_ang, plan)
			
			# publish the plan
			plan_pub.publish(plan)
			loop_rate.sleep()
		
