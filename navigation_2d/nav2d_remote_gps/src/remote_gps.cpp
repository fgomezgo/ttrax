#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <nav2d_operator/cmd.h>
#include <nav2d_navigator/commands.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

//Constants for GPS to m conversion in Tec
//const double LONG_TO_M = 1.0436052429987237;
//const double LAT_TO_M = 1.0392838770711008;
//Constants for GPS to m conversion in Las Vegas
const double LONG_TO_M = 0.8950320145268125;
const double LAT_TO_M = 1.1104623806596103;
const double GPS_FACTOR = 0.00001;

const double scale = 1.0;

//Pi
const double PI = 3.14159265359;

struct Target{
	double longitude, latitude;
	Target(){
		longitude = latitude = 0.0;
	}
};

struct Vector{
	double x,y;
	Vector(){}
	//Constructor
	Vector(const double &_x, const double &_y){
		x = _x;
		y = _y;
	}
	//Vector's magnitude
	double magnitude(){
		return sqrt(x*x+y*y);
	}
};
//Methods for vectors

//Dot product
double dotProduct(const Vector &a, const Vector &b){
	return a.x*b.x+a.y*b.y;
}
//Cross product
double crossProduct(const Vector &a, const Vector &b){
	return a.x*b.y-a.y*b.x;
}
//Angle between 2 vectors
double angleBetween(Vector a, Vector b){
	return acos(dotProduct(a,b)/(a.magnitude()*b.magnitude()));
}


geometry_msgs::Twist cmd;

//Publishers
ros::Publisher cmd_pub;

//To calculate cmd's Turn
//Vector formed between current position and target
Vector toTarget = Vector(1,0);
Vector toHeading = Vector(1,0);

//Flag to indicate thata the current target has been reached
bool targetReached = false;

//Flag for the first time it enters the gpsDataCallback
bool firstTimeInCallback = true;

//Init && Target
Target init ,target;

//Parameters for gpsDataCallback
//Percentage Threshold of Max Velocity for Autonomous mode
double maxVel; // default 80%
//Percentage Threshold of Min Velocity for Autonomous mode
double minVel; // default 20%
//Minimum distance to target at which velocity is kept maximal
double minDistanceForMaxVel; // default 3 meters
//Minimum error in distance to target that is acceptable when reaching a target
double minDistanceErrorInTarget; // default 1 meters

//Parameters for gpsHeadingCallback
//Minimum angle in radians where it is safe to turn in both directions (Right && Left) 
double minAngleToTurnInBothDirections; // default 2.8 Radians

void targetCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	//Get GPS Target
	target.longitude 	= 	msg->longitude;
	target.latitude 	= 	msg->latitude;
}

void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	ROS_INFO("============================ GPS DATA CB ============================");
	if(firstTimeInCallback){
		//Add && Printf the current position
		init.longitude 	= msg->longitude;
		init.latitude 	= msg->latitude;
		ROS_INFO("Initial Position : %lf %lf\n",init.longitude,init.latitude);
		firstTimeInCallback = false;
		//return;
	}
	//Ignore target if is set to 0,0
	if(target.longitude == 0.0 || target.latitude == 0.0){
		ROS_INFO("No new target yet defined!");
		return;
	}

	//Calculate vector between position and target
	//toTarget = Vector(target.longitude-msg->longitude-2*(init.longitude),target.latitude-msg->latitude-2*(init.latitude));
	double xGPS = target.longitude-msg->longitude;
	double yGPS = target.latitude-msg->latitude;
	xGPS = (xGPS/GPS_FACTOR)*LONG_TO_M;
	yGPS = (yGPS/GPS_FACTOR)*LAT_TO_M;

	toTarget = Vector(xGPS,yGPS);
	ROS_INFO("Position :  	%lf %lf", msg->longitude, msg->latitude);
	ROS_INFO("Target   :  	%lf %lf", target.longitude, target.latitude);
	ROS_INFO("Coord	   :    %lf %lf", xGPS,yGPS);
	//ROS_INFO("Position in map ->  %lf %lf", msg->longitude-init.longitude, msg->latitude-init.latitude);
	
	//Calculate distance between current position (msg) and target
	double distance = toTarget.magnitude();

	ROS_INFO("Distance : 	%lf", distance);
	//Threshold to slowdown velocity
	if(distance > minDistanceForMaxVel)
		cmd.linear.x = maxVel;
	else
		cmd.linear.x = std::max((maxVel/minDistanceForMaxVel)*distance,minVel);
	cmd_pub.publish(cmd);

	//End if minError is reached
	if(distance < minDistanceErrorInTarget){
		//Stop Rover
		cmd.linear.x = cmd.angular.z = 0.0;
		cmd_pub.publish(cmd);
		targetReached = true;
	}
	ROS_INFO("Velocity :	%lf", cmd.linear.x);
	ROS_INFO("=====================================================================");

}

void gpsHeadingCallback(const std_msgs::Float64::ConstPtr& msg)
{
	if(target.longitude == 0.0 || target.latitude == 0.0){
	return;
}
	ROS_INFO("============================ HEADING CB ============================");
	ROS_INFO("Heading	:	 %lf", msg->data);
	double heading = (msg->data*PI)/180.0;
	heading += PI/2.0;
	ROS_INFO("Heading	:	 %lf", heading);
	heading -= PI/2.0;
	//Calculate angle between heading and toTarget
	toHeading = Vector(cos(heading),sin(heading));
	double angle = angleBetween(Vector(toTarget.x*scale,toTarget.y*scale),Vector(toHeading.x*scale,toHeading.y*scale));
	double cP = crossProduct(Vector(toTarget.x*scale,toTarget.y*scale),Vector(toHeading.x*scale,toHeading.y*scale));

	ROS_INFO("Vector Target 	: 	x %lf y %lf", toTarget.x, toTarget.y);
	ROS_INFO("Vector Heading 	:	x %lf y %lf", toHeading.x, toHeading.y);
	/*
	if(angle < minAngleToTurnInBothDirections){
		if(cP < 0)
			angle *= -1.0;
	}*/
	ROS_INFO("CrossProduct		:	%lf", cP);
	ROS_INFO("Angle 			:	%.8lf", angle);

	cmd.angular.z = (angle)/(PI);
	ROS_INFO("Turn				:	%lf", cmd.angular.z);

	cmd_pub.publish(cmd);
	ROS_INFO("===================================================================");
}

void gpsImuCallback(const std_msgs::Float64::ConstPtr& msg)
{
	double heading = msg->data;
	//ROS_INFO("IMU Heading %lf", heading);
}

int main(int argc, char** argv)
{
	//Node Initialization
	ros::init(argc, argv, "remote_gps");
	
	//Node handle
	ros::NodeHandle nh;
	
	//Ros Rate 5Hz
	ros::Rate loop_rate(5);	

	//Get Parameters
	nh.param("max_vel",maxVel,0.8);
	nh.param("min_vel",minVel,0.2);
	nh.param("min_distance_for_max_vel",minDistanceForMaxVel,3.0);
	nh.param("min_distance_error_in_target",minDistanceErrorInTarget,1.0);
	nh.param("min_angle_to_turn_in_both_directions",minAngleToTurnInBothDirections,1.7);

	//Publishers
	cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	
	//Subscribers
	//Target Coordinates (longitude, latitude)
	ros::Subscriber target_coordinates = nh.subscribe("GPS_goal/target", 10, targetCallback);
	//Gps data (longitude, latitude)
	ros::Subscriber sub_gps_data = nh.subscribe("GPS_goal/gps_data", 10, gpsDataCallback);
	//Gps heading
	ros::Subscriber sub_gps_heading = nh.subscribe("GPS_goal/gps_heading", 10, gpsHeadingCallback);
	//IMU heading
	ros::Subscriber sub_imu_heading = nh.subscribe("GPS_goal/IMU_heading", 10, gpsImuCallback);

	//Initialize cmd Mode
	//cmd.Mode = 0;
	cmd.angular.z = 0;
	cmd.linear.x = 0;
	ROS_INFO("Waiting first callback...");
	ros::spinOnce();
	//Wait untill first entry
	while(firstTimeInCallback){
		ros::spinOnce();
	}
	ROS_INFO("DONE");

	while(ros::ok()){
		if(targetReached){
			ROS_INFO("Target reached!");
			targetReached = false;
			target.longitude = target.latitude = 0.0;
		}
		ros::spinOnce();	
	}
	
	return 0;
}
