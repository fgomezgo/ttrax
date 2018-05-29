#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav2d_operator.msg import cmd
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from math import *

#Constants for GPS to m conversion in Tec
#LONG_TO_M = 1.0436052429987237
#LAT_TO_M = 1.0392838770711008
#Constants for GPS to m conversion in Las Vegas
LONG_TO_M = 0.8950320145268125
LAT_TO_M = 1.1104623806596103

#Factor of conversion
GPS_FACTOR = 0.00001

#Object to store GPS coordinates
class GPS_Coord:
    def __init__(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude
#Object to describe 2D vectors
class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def magnitude(self):
        return sqrt(self.x**2+self.y**2)
#Methods for vectors
def dotProduct(a,b):
    return a.x*b.x+a.y*b.y
def crossProduct(a,b):
    return a.x*b.y-a.y*b.x
def innerAngle(a,b):
    return acos(dotProduct(a,b)/(a.magnitude()*b.magnitude()))

#cmd Twist
cmd = Twist()

#Global cmd publisher
cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)

#To calculate cmd's Turn
#Vector formed between current position and target
toTarget = Vector(1,0)
toHeading = Vector(1,0)

#Flag to indicate thata the current target has been reached
targetReached = False

#Flag for the first time it enters the gpsDataCallback
firstTimeInCallback = True

#Init && Target
init = GPS_Coord(0,0)
target = GPS_Coord(0,0)

#Variables for parameters
#Get Parameters
maxVel = rospy.get_param("max_vel",0.8)
minVel = rospy.get_param("min_vel",0.2)
minDistanceForMaxVel = rospy.get_param("min_distance_for_max_vel",3.0)
minDistanceErrorInTarget = rospy.get_param("min_distance_error_in_target",1.0)
minAngleToTurnInBothDirections = rospy.get_param("min_angle_to_turn_in_both_directions",2.8)

def targetCallback(msg):
    #Get GPS Target
    target.longitude    = 	msg.longitude
    target.latitude     = 	msg.latitude 
def gpsDataCallback(msg):
    global firstTimeInCallback
    global toTarget
    rospy.loginfo('============================ GPS DATA CB ============================')
    if firstTimeInCallback:
        #Add the current position
        init.longitude = msg.longitude
        init.latitude = msg.latitude
        rospy.loginfo('Initial Position: '+str(init.longitude)+' '+str(init.latitude))
        firstTimeInCallback = False
    #Ignore target if is set to 0,
    if target.longitude == 0 or  target.latitude == 0:
        rospy.loginfo('No new target yet define!')
        return
    #Calculate vector between position and target
    xGPS = target.longitude-msg.longitude
    yGPS = target.latitude-msg.latitude
    xGPS = (xGPS/GPS_FACTOR)*LONG_TO_M
    yGPS = (yGPS/GPS_FACTOR)*LAT_TO_M

    toTarget = Vector(xGPS,yGPS)
    rospy.loginfo('Position: '+str(msg.longitude)+' '+str(msg.latitude))
    rospy.loginfo('Target:   '+str(target.longitude)+' '+str(target.latitude))
    rospy.loginfo('Coord:    '+str(xGPS)+' '+str(yGPS))

    distance = toTarget.magnitude()
    rospy.loginfo('Distance: '+str(distance))
    #Threshold to slowdown velocity
    if distance > minDistanceForMaxVel:
        cmd.linear.x = maxVel
    else:
        cmd.linear.x = max((maxVel/minDistanceForMaxVel)*distance,minVel)
    cmd_pub.publish(cmd)

    #End if minError is reached
    if distance < minDistanceErrorInTarget:
        #Stop Rover
        cmd.linear.x = cmd.angular.z = 0.0
        cmd_pub.publish(cmd)
        targetReached = True
    rospy.loginfo('Velocity: '+str(cmd.linear.x))
    rospy.loginfo('=====================================================================')
def gpsHeadingCallback(msg):
    rospy.loginfo("============================ HEADING CB ============================")
    rospy.loginfo("Heading:         "+str(msg.data))
    #Tranform heading
    heading = (msg.data*pi)/180.0 + pi/2.0
    rospy.loginfo("Heading:         "+str(heading))
    #Calculate angle between heading and toTarget
    toHeading = Vector(cos(heading),sin(heading))
    angle = innerAngle(toTarget,toHeading)
    cP = crossProduct(toTarget,toHeading)

    rospy.loginfo("Vector Target:   "+str(toTarget.x)+' '+str(toTarget.y))
    rospy.loginfo("Vector Heading:  "+str(toHeading.x)+' '+str(toHeading.y))
    '''
    if angle < minAngleToTurnInBothDirections:
        if cP < 0:
            angle *= -1.0
    }
    '''
    rospy.loginfo("CrossProduct:    "+str(cP))
    rospy.loginfo("Angle Target:    "+str(atan2(1,0)*180/pi))
    rospy.loginfo("Angle:           "+str(angle*180/pi))

    cmd.angular.z = angle/pi
    rospy.loginfo("Turn:            "+str(cmd.angular.z))

    cmd_pub.publish(cmd)
    rospy.loginfo("===================================================================")
def gpsImuCallback(msg):
	heading = msg.data
	#rospy.loginfo("IMU Heading:     "+heading)

if __name__ == '__main__':
    rospy.init_node('remote_gps', anonymous=True)
    rate = rospy.Rate(5) # 5hz

    #Subscribers
    #Target Coordinates (longitude, latitude)
    rospy.Subscriber('GPS_goal/target', NavSatFix, targetCallback)
    #Gps data (longitude, latitude)
    rospy.Subscriber("GPS_goal/gps_data", NavSatFix, gpsDataCallback)
    #Gps heading
    rospy.Subscriber("GPS_goal/gps_heading", Float64, gpsHeadingCallback)
    #IMU heading
    rospy.Subscriber("GPS_goal/IMU_heading", Float64, gpsImuCallback)

    #Initialize cmd Mode
	#cmd.Mode = 0
    cmd.angular.z = 0
    cmd.linear.x = 0
    rospy.loginfo("Waiting first callback...")
    #rospy.spinOnce()
    #Wait untill first entry
    while firstTimeInCallback:
        #rospy.spinOnce()
        pass
    rospy.loginfo("DONE")

    while not rospy.is_shutdown():
        if targetReached:
            rospy.loginfo("Target reached!")
            targetReached = False
            target.longitude = target.latitude = 0.0
        #rospy.spinOnce();	





