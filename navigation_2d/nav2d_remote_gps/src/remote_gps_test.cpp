#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <nav2d_operator/cmd.h>
#include <nav2d_navigator/commands.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>


//Pi
const double PI = 3.14159265359;

struct Target{
	double longitude, latitude;
	Target(){}
} target;

struct Vector{
	double x,y;
	Vector(){}
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


nav2d_operator::cmd cmd;


//Publishers
ros::Publisher cmd_pub;

//To calculate cmd's Turn

//Vector formed between current position and target
Vector toTarget = Vector(1,0);
Vector toHeading = Vector(1,0);

//Flag to end program
bool endProgram = false;


void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	//Calculate vector between position and target
	toTarget = Vector(target.longitude-msg->longitude,target.latitude-msg->latitude);
	ROS_INFO("Position x %lf y %lf", msg->longitude, msg->latitude);
	
	//Calculate distance between the 2 points
	double distance = toTarget.magnitude();
	ROS_INFO("Distance %lf", distance);
	//Threshold to slowdown velocity
	if(distance > 3)
		cmd.Velocity = 0.5;
	else
		cmd.Velocity = 0.16*distance;
	cmd_pub.publish(cmd);

	//End if minError is reached
	if(distance < 1){
		cmd.Velocity = 0.0;
		cmd.Turn = 0.0;
		cmd_pub.publish(cmd);
		endProgram = true;
	}
}

void gpsHeadingCallback(const std_msgs::Float64::ConstPtr& msg)
{
	double heading = msg->data;
	//ROS_INFO("Heading %lf", heading);
	if(heading < 0)
		heading += 2*PI;
	ROS_INFO("Heading %lf", heading);
	
	//Calculate angle between heading and toTarget
	toHeading = Vector(cos(heading),sin(heading));
	double angle = angleBetween(toTarget,toHeading);
	double cP = crossProduct(toTarget,toHeading);
	ROS_INFO("Vector Target x %lf y %lf", toTarget.x, toTarget.y);
	ROS_INFO("Vector Heading x %lf y %lf", toHeading.x, toHeading.y);
	if(angle < 1.7){
		if(cP < 0)
			angle *= -1.0;
	}
	ROS_INFO("CrossProduct %lf", cP);
	ROS_INFO("Angle %lf", angle);

	cmd.Turn = (angle)/(PI);
	ROS_INFO("Turn %lf", double(cmd.Turn));

	cmd_pub.publish(cmd);
}




int main(int argc, char** argv)
{
	//Node Initialization
	ros::init(argc, argv, "remote_gps");

	//Node handle
	ros::NodeHandle nh;

	//Publishers
	cmd_pub = nh.advertise<nav2d_operator::cmd>("cmd", 100);
	
	//Subscribers
	//Gps data (longitude, latitude)
	ros::Subscriber sub_gps_data = nh.subscribe("GPS_goal/gps_data", 100, gpsDataCallback);
	//Gps heading
	ros::Subscriber sub_gps_heading = nh.subscribe("GPS_goal/gps_heading", 100, gpsHeadingCallback);

	//Initialize cmd Mode
	cmd.Mode = 0;
	cmd.Turn = 0;
	cmd.Velocity = 0;

	//Printf the current position
	printf("The current position is: 0 0\n");
	//Read Goal in Map
	
	printf("Give me the coordinates (longitude, latitude) ");
	scanf("%lf%lf",&target.longitude,&target.latitude);

	while(ros::ok()){
		if(endProgram){
			ROS_INFO("Target reached!");
			return 0;
		}
		ros::spinOnce();	
	}
	
	return 0;
}
