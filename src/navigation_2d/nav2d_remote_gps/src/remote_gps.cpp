#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <nav2d_operator/cmd.h>
#include <nav2d_navigator/commands.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

//Constants for GPS to m conversion
const double LONG_TO_M = 1.7286720451827369;
const double LAT_TO_M = 1.849419214323556;
const double GPS_FACTOR = 0.00001;


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
		double xGPS = x/GPS_FACTOR*LONG_TO_M;
		double yGPS = y/GPS_FACTOR*LAT_TO_M;
		return sqrt(xGPS*xGPS+yGPS*yGPS);
		//return sqrt(x*x+y*y);
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

//Flag for the first time it enters the gpsDataCallback
bool firstTimeInCallback = true;

//Target init
Target init;

void gpsDataCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	if(firstTimeInCallback){
		//Add && Printf the current position
		init.longitude = msg->longitude;
		init.latitude = msg->latitude;
		ROS_INFO("The current position is: %lf %lf\n",init.longitude,init.latitude);
		firstTimeInCallback = false;
		return;
	}
	//Calculate vector between position and target
	toTarget = Vector(target.longitude-msg->longitude-2*(init.longitude),target.latitude-msg->latitude-2*(init.latitude));
	ROS_INFO("Position x %lf y %lf", msg->longitude, msg->latitude);
	ROS_INFO("Position in map x %lf y %lf", msg->longitude-init.longitude, msg->latitude-init.latitude);
	
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

void gpsImuCallback(const std_msgs::Float64::ConstPtr& msg)
{
	double heading = msg->data;
	ROS_INFO("IMU Heading %lf", heading);
}

int main(int argc, char** argv)
{
	//Node Initialization
	ros::init(argc, argv, "remote_gps");
	
	//Node handle
	ros::NodeHandle nh;
	
	//Ros Rate 5Hz
	ros::Rate loop_rate(5);	

	//Publishers
	cmd_pub = nh.advertise<nav2d_operator::cmd>("cmd", 100);
	
	//Subscribers
	//Gps data (longitude, latitude)
	ros::Subscriber sub_gps_data = nh.subscribe("GPS_goal/gps_data", 10, gpsDataCallback);
	//Gps heading
	ros::Subscriber sub_gps_heading = nh.subscribe("GPS_goal/gps_heading", 10, gpsHeadingCallback);
	//IMU heading
	ros::Subscriber sub_imu_heading = nh.subscribe("GPS_goal/IMU_heading", 10, gpsImuCallback);

	//Initialize cmd Mode
	cmd.Mode = 0;
	cmd.Turn = 0;
	cmd.Velocity = 0;
	ROS_INFO("Waiting first callback");
	ros::spinOnce();
	//Wait untill first entry
	while(firstTimeInCallback){
		ros::spinOnce();
	}
	ROS_INFO("DONE");
	
	//Read Goal in Map
	ROS_INFO("Give me the coordinates (longitude, latitude) ");
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