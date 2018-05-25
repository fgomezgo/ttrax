#!/usr/bin/env python

# Libraries
import math
from math import sin, cos, pi
import numpy as np 
import roslib
import rospy 
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState
from tf import TransformBroadcaster
from nav_msgs.msg import Odometry


# Base Controller Variables
width_robot = 2
vr = 0.0
right_vel = 0.0
right_vel_old = 0.0

vl = 0.0
left_vel = 0.0
left_vel_old = 0.0

distance_left = 0.0
distance_right = 0.0

#ticks_per_meter = 100;
vel_enc_right = 0.0
vel_enc_left = 0.0

x = 0.0
y = 0.0
th = 0.0
odom_quat = Quaternion

#Callback to recieve velocity commands
def cmd_velCallback(twist_aux):
	pupr = rospy.Publisher('drive_system_right/setpoint', Float64, queue_size=10)
	pupl = rospy.Publisher('drive_system_left/setpoint', Float64, queue_size=10)
	twist = twist_aux
	vel_x = twist_aux.linear.x
	vel_th = twist_aux.angular.z
	right_vel = 0.0
	left_vel = 0.0

	if(vel_x == 0):
		#drift
		right_vel = vel_th * width_robot / 2.0
		left_vel = (-1) * right_vel
	elif(vel_th == 0):
		#frente/reversa 
		left_vel = right_vel = vel_x
	else:
		#curvas
		left_vel = vel_x - vel_th * width_robot / 2.0
		righ_vel = vel_x + vel_th * width_robot / 2.0
	#vl = left_vel
	#vr = right_vel
	pupr.publish(left_vel*0.6)
	pupl.publish(right_vel*0.6)

#callback de los encoders para saver la velocidad
def encvelR(vel):
	vel_enc_right = vel
def encvelL(vel):
	vel_enc_left = vel

#Update la velocidad real del robot
def realVel():
	rospy.init_node('base_contol', anonymous=True)
	r = rospy.Rate(10)	
	rospy.Subscriber("cmd_vel",Twist,cmd_velCallback)
	rospy.Subscriber("rover_right/state/data",Float64,encvelR)
	rospy.Subscriber("rover_left/state/data",Float64,encvelL)
	last_time = rospy.get_rostime()
	while not rospy.is_shutdown():
		#Commented code. Calculates its odometry of the robot
		'''
		dxy = 0.0 
		dth = 0.0
		current_time = rospy.get_rostime()
		dt = (current_time.nsecs - last_time.nsecs)*1000000000
		last_time = current_time
		dt
		if dt == 0:
			velxy = 0
			velth = 0
		else: 
			velxy = dxy / dt
			velth = dth / dt

		#odometria
		if (vel_enc_right == 0):
			distance_left = 0.0
			distance_right = 0.0
		else:
			distance_left = vel_enc_left * dt
			distance_right = vel_enc_left  * dt

		dxy = (distance_left + distance_right) / 2.0
		dth = (distance_right - distance_left) / width_robot
		if(dxy != 0):
			x += dxy * cosf(dth)
			y += dxy * sinf(dth)
		if(dth != 0):
			dth += dth
		
		odom_quat = tf.transformations.quaternion_from_euler(0,0,dth)
		'''
		
		r.sleep()
		rospy.spin()	


if __name__ == '__main__':
	try:
		realVel()
	except rospy.ROSInterruptException: 
		pass
