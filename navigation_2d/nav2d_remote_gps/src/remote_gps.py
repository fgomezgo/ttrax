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

#Flag to indicate thata the current target has been reached
targetReached = False

#Flag for the first time it enters the gpsDataCallback
firstTimeInCallback = True

#Flag to know if theres a target
isTarget = False

#Init && Target && Current
init    =   GPS_Coord(0,0)
target  =   GPS_Coord(0,0)
current =   GPS_Coord(0,0)
prev    =   GPS_Coord(0,0)

#GPS Heading Float
GPS_heading = 0
Real_heading = 0

#Variables for parameters
#Get Parameters
maxVel = rospy.get_param("max_vel",0.8)
minVel = rospy.get_param("min_vel",0.2)
minDistanceForMaxVel = rospy.get_param("min_distance_for_max_vel",3.0)
minDistanceErrorInTarget = rospy.get_param("min_distance_error_in_target",1.0)
minAngleToTurnInBothDirections = rospy.get_param("min_angle_to_turn_in_both_directions",2.8)

def targetCallback(msg):
    global target
    global isTarget
    #Get GPS Target
    target.longitude    = 	msg.longitude
    target.latitude     = 	msg.latitude
    isTarget = True

def gpsDataCallback(msg):
    global current
    global prev
    global firstTimeInCallback
    if msg.longitude == 0 and msg.latitude == 0:
        return 
    if firstTimeInCallback:
        init.longitude = msg.longitude
        init.latitude = msg.latitude
        firstTimeInCallback = False
        #In First one Prev = Curr
        current.longitude    = 	msg.longitude
        current.latitude     = 	msg.latitude
        prev.longitude       = 	msg.longitude
        prev.latitude        = 	msg.latitude
    else:
        prev.longitude       = current.longitude
        prev.latitude        = current.latitude
        current.longitude    = 	msg.longitude
        current.latitude     = 	msg.latitude


def gpsHeadingCallback(msg):
    global GPS_heading
    global Real_heading
    GPS_heading = msg.data
    Real_heading = (GPS_heading*pi)/180.0 + pi/2.0

def gpsImuCallback(msg):
	heading = msg.data
	#rospy.loginfo("IMU Heading:     "+heading)
        
if __name__ == '__main__':
	 
    rospy.init_node('remote_gps', anonymous=True)
    rate = rospy.Rate(1) # 1hz

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

    #Wait untill first entry
    while firstTimeInCallback:
        pass
    rospy.loginfo("DONE.")

    while not rospy.is_shutdown():
        if targetReached:
            rospy.loginfo("Target reached!!!!!!!!")
            targetReached = False
            target.longitude = target.latitude = 0
        #Print Heading
        rospy.loginfo('============================ GPS DATA ============================')
        rospy.loginfo('Initial Position:   '+str(init.longitude)+' '+str(init.latitude))
        if isTarget:
            ### Linear Velocity ###
            #Calculate vector between current and target
            xGPS = target.longitude-current.longitude
            yGPS = target.latitude-current.latitude
            xGPS = (xGPS/GPS_FACTOR)*LONG_TO_M
            yGPS = (yGPS/GPS_FACTOR)*LAT_TO_M
            #create vector toTarget with xGPS and yGPS
            toTarget = Vector(xGPS,yGPS)
            #calculate distance from current to target
            distance = toTarget.magnitude()
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
                isTarget = False
            ### Angular Velocity ###
            #Calculate angle between heading and toTarget
            toHeading = Vector(cos(Real_heading),sin(Real_heading))
            angle = innerAngle(toTarget,toHeading)
            cP = crossProduct(toTarget,toHeading)
            #Caluculate Turn's direction based on cross product
            if angle < minAngleToTurnInBothDirections:
                if cP > 0:
                    angle *= -1.0
            #Calculate Turn
            cmd.angular.z = angle/pi
            cmd_pub.publish(cmd)
            #Calculate Sim Heading
            prevToCurr = Vector(current.longitude-prev.longitude,current.latitude-prev.latitude)
            #flag to validate heading
            isHeading = False
            Simu_heading = 0
            if prevToCurr.y != 0 or prevToCurr.x != 0:
                Simu_heading = atan2(prevToCurr.y,prevToCurr.x)
                isHeading = True
            
            #Print info
            rospy.loginfo('Current:        '+str(current.longitude)+' '+str(current.latitude))
            rospy.loginfo('Target:         '+str(target.longitude)+' '+str(target.latitude))
            rospy.loginfo('Coord:          '+str(xGPS)+' '+str(yGPS))
            rospy.loginfo('Distance:       '+str(distance))
            rospy.loginfo('Velocity (cmd): '+str(cmd.linear.x))
            rospy.loginfo(' ')
            rospy.loginfo('GPS Heading:    '+str(GPS_heading))    
            rospy.loginfo('Real Heading:   '+str(Real_heading*180/pi))
            if isHeading:
                rospy.loginfo('Simu Heading:   '+str(Simu_heading*180/pi))
            else:
                rospy.loginfo('Simu Heading:   No HEADING!')
            rospy.loginfo('Vector Target:  '+str(toTarget.x)+' '+str(toTarget.y))
            rospy.loginfo('Vector Heading: '+str(toHeading.x)+' '+str(toHeading.y))
            rospy.loginfo('CrossProduct:   '+str(cP))
            rospy.loginfo(' ')
            rospy.loginfo('Angle Target:   '+str(atan2(toTarget.y,toTarget.x)*180/pi))
            rospy.loginfo('Angle Heading:  '+str(Real_heading*180/pi))
            rospy.loginfo('Angle Between:  '+str(angle*180/pi))
            rospy.loginfo('Turn (cmd):     '+str(cmd.angular.z))
        else:
            rospy.loginfo('No new target yet defined!')
        rospy.loginfo('==================================================================')
        rate.sleep()





